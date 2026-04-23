library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- RushTunnelFPGA.vhd  —  Top level
-- ================================================================
-- Complete wiring of all game modules.
--
-- Ship screen-position computation (once per frame, not per pixel):
--   ship_x = 400 + (cos(ship_scr_angle) * SHIP_RADIUS) >> 8
--   ship_y = 240 + (sin(ship_scr_angle) * SHIP_RADIUS) >> 8
--   where SHIP_RADIUS = 285 pixels from tunnel centre
--
-- Two trig_rom instances share the same entity:
--   u_trig_tun  : tunnel rotation  → cos_t, sin_t
--   u_trig_ship : ship screen pos  → cos_s, sin_s
--
-- game_active : game_state(0) = '1' only for "01" (PLAYING).
--
-- zone_advance : placeholder ('0') until game_fsm outputs it.
--   Wire to game_fsm's zone_unlock output in a later step.
-- ================================================================

entity RushTunnelFPGA is
    port (
        CLK_50 : in  std_logic;
        RED    : out std_logic_vector(3 downto 0);
        GREEN  : out std_logic_vector(3 downto 0);
        BLUE   : out std_logic_vector(3 downto 0);
        SYNC   : out std_logic_vector(1 downto 0);
        GP_L_1 : out std_logic;
        GP_C_1 : out std_logic;
        GP_D_1 : in  std_logic
    );
end RushTunnelFPGA;

architecture rtl of RushTunnelFPGA is

    constant FRAME_CLOCKS : integer := 692640;
    constant SPEED_DIV    : integer := 1;
    constant SHIP_RADIUS  : integer := 285;

    -- VGA
    signal hsync_s  : std_logic;
    signal vsync_s  : std_logic;
    signal video_on : std_logic;
    signal hpos     : std_logic_vector(9 downto 0);
    signal vpos     : std_logic_vector(9 downto 0);

    -- Frame counter / scroll / hue / tick
    signal frame_cnt  : integer range 0 to FRAME_CLOCKS - 1 := 0;
    signal speed_cnt  : integer range 0 to SPEED_DIV - 1    := 0;
    signal scroll     : unsigned(7 downto 0) := (others => '0');
    signal hue_cnt    : unsigned(9 downto 0) := (others => '0');
    signal frame_tick : std_logic := '0';

    -- SNES
    signal buttons : std_logic_vector(15 downto 0);

    -- Game FSM
    signal game_state     : std_logic_vector(1 downto 0);
    signal tunnel_angle   : std_logic_vector(15 downto 0);
    signal ship_scr_angle : std_logic_vector(15 downto 0);
    signal ship_rel_angle : std_logic_vector(15 downto 0);
    signal reset_game     : std_logic;
    signal difficulty     : std_logic_vector(3 downto 0);

    -- Trig ROM outputs
    signal cos_t : signed(8 downto 0);
    signal sin_t : signed(8 downto 0);
    signal cos_s : signed(8 downto 0);
    signal sin_s : signed(8 downto 0);

    -- Ship screen position (registered, updated each clock from stable trig)
    signal ship_x_r : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal ship_y_r : unsigned(9 downto 0) := to_unsigned(240, 10);

    -- Obstacle manager
    signal obs0_depth, obs1_depth, obs2_depth, obs3_depth : std_logic_vector(7 downto 0);
    signal obs0_face,  obs1_face,  obs2_face,  obs3_face  : std_logic_vector(2 downto 0);
    signal obs0_act,   obs1_act,   obs2_act,   obs3_act   : std_logic;
    signal collision : std_logic;

    -- Scoring rings
    signal ring0_depth_s, ring1_depth_s : std_logic_vector(7 downto 0);
    signal ring0_act_s,   ring1_act_s   : std_logic;
    signal ring_collected_s             : std_logic;  -- pulse to game_fsm (future)

    -- Stage manager
    signal num_faces_s        : std_logic_vector(3 downto 0);
    signal transition_act_s   : std_logic;

begin

    SYNC(0) <= vsync_s;
    SYNC(1) <= hsync_s;

    -- ================================================================
    -- VGA controller
    -- ================================================================
    u_vga : entity work.vga_controller
        port map (
            clk_50   => CLK_50,
            rst      => '0',
            hsync    => hsync_s,
            vsync    => vsync_s,
            video_on => video_on,
            hpos     => hpos,
            vpos     => vpos
        );

    -- ================================================================
    -- Trig ROMs
    -- ================================================================
    u_trig_tun : entity work.trig_rom
        port map (
            angle   => tunnel_angle,
            sin_val => sin_t,
            cos_val => cos_t
        );

    u_trig_ship : entity work.trig_rom
        port map (
            angle   => ship_scr_angle,
            sin_val => sin_s,
            cos_val => cos_s
        );

    -- ================================================================
    -- Ship screen position: 400 + cos_s*SHIP_RADIUS/256
    -- ================================================================
    process(CLK_50)
        variable sx_prod : signed(18 downto 0);
        variable sy_prod : signed(18 downto 0);
        variable sx_off  : signed(9 downto 0);
        variable sy_off  : signed(9 downto 0);
        variable sx_pos  : signed(10 downto 0);
        variable sy_pos  : signed(10 downto 0);
    begin
        if rising_edge(CLK_50) then
            sx_prod := cos_s * to_signed(SHIP_RADIUS, 10);
            sy_prod := sin_s * to_signed(SHIP_RADIUS, 10);
            sx_off  := resize(shift_right(sx_prod, 8), 10);
            sy_off  := resize(shift_right(sy_prod, 8), 10);
            sx_pos  := to_signed(400, 11) + resize(sx_off, 11);
            sy_pos  := to_signed(240, 11) + resize(sy_off, 11);
            ship_x_r <= unsigned(sx_pos(9 downto 0));
            ship_y_r <= unsigned(sy_pos(9 downto 0));
        end if;
    end process;

    -- ================================================================
    -- Stage manager (polygon selection + blackout transitions)
    -- zone_advance wired '0' until game_fsm exposes it
    -- ================================================================
    u_stage : entity work.stage_manager
        port map (
            clk_50            => CLK_50,
            frame_tick        => frame_tick,
            zone_advance      => '0',
            reset_game        => reset_game,
            num_faces         => num_faces_s,
            transition_active => transition_act_s
        );

    -- ================================================================
    -- Tunnel renderer
    -- ================================================================
    u_tunnel : entity work.tunnel_renderer
        port map (
            clk_50            => CLK_50,
            video_on          => video_on,
            hpos              => hpos,
            vpos              => vpos,
            scroll            => std_logic_vector(scroll),
            hue_cnt           => hue_cnt,
            num_faces         => num_faces_s,
            transition_active => transition_act_s,
            cos_t             => cos_t,
            sin_t             => sin_t,
            ship_x            => ship_x_r,
            ship_y            => ship_y_r,
            game_state        => game_state,
            obs0_depth        => obs0_depth,
            obs0_face_sector  => obs0_face,
            obs0_active       => obs0_act,
            obs1_depth        => obs1_depth,
            obs1_face_sector  => obs1_face,
            obs1_active       => obs1_act,
            obs2_depth        => obs2_depth,
            obs2_face_sector  => obs2_face,
            obs2_active       => obs2_act,
            obs3_depth        => obs3_depth,
            obs3_face_sector  => obs3_face,
            obs3_active       => obs3_act,
            ring0_depth       => ring0_depth_s,
            ring0_active      => ring0_act_s,
            ring1_depth       => ring1_depth_s,
            ring1_active      => ring1_act_s,
            red               => RED,
            green             => GREEN,
            blue              => BLUE
        );

    -- ================================================================
    -- SNES controller
    -- ================================================================
    u_snes : entity work.snes_controller
        port map (
            clk_50   => CLK_50,
            gp_latch => GP_L_1,
            gp_clk   => GP_C_1,
            gp_data  => GP_D_1,
            buttons  => buttons
        );

    -- ================================================================
    -- Game FSM
    -- ================================================================
    u_fsm : entity work.game_fsm
        port map (
            clk_50         => CLK_50,
            frame_tick     => frame_tick,
            buttons        => buttons,
            collision      => collision,
            game_state     => game_state,
            tunnel_angle   => tunnel_angle,
            ship_scr_angle => ship_scr_angle,
            ship_rel_angle => ship_rel_angle,
            reset_game     => reset_game,
            difficulty     => difficulty
        );

    -- ================================================================
    -- Obstacle manager
    -- ================================================================
    u_obs : entity work.obstacle_manager
        port map (
            clk_50          => CLK_50,
            frame_tick      => frame_tick,
            game_active     => game_state(0),
            reset_game      => reset_game,
            ship_rel_angle  => ship_rel_angle,
            difficulty      => difficulty,
            num_faces       => num_faces_s,
            obs0_depth       => obs0_depth,
            obs0_face_sector => obs0_face,
            obs0_active      => obs0_act,
            obs1_depth       => obs1_depth,
            obs1_face_sector => obs1_face,
            obs1_active      => obs1_act,
            obs2_depth       => obs2_depth,
            obs2_face_sector => obs2_face,
            obs2_active      => obs2_act,
            obs3_depth       => obs3_depth,
            obs3_face_sector => obs3_face,
            obs3_active      => obs3_act,
            collision        => collision,
            ring0_depth     => ring0_depth_s,
            ring0_active    => ring0_act_s,
            ring1_depth     => ring1_depth_s,
            ring1_active    => ring1_act_s,
            ring_collected  => ring_collected_s
        );

    -- ================================================================
    -- Frame counter, scroll, hue_cnt, frame_tick
    -- ================================================================
    process(CLK_50)
    begin
        if rising_edge(CLK_50) then
            frame_tick <= '0';
            if frame_cnt = FRAME_CLOCKS - 1 then
                frame_cnt  <= 0;
                frame_tick <= '1';
                hue_cnt    <= hue_cnt + 1;
                if speed_cnt = SPEED_DIV - 1 then
                    speed_cnt <= 0;
                    scroll    <= scroll + 1;
                else
                    speed_cnt <= speed_cnt + 1;
                end if;
            else
                frame_cnt <= frame_cnt + 1;
            end if;
        end if;
    end process;

end rtl;
