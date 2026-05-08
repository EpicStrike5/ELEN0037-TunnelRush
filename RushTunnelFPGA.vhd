library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- RushTunnelFPGA.vhd  —  Top level
-- ================================================================
-- Module wiring:
--
--   vga_controller  → hpos/vpos/video_on/hsync/vsync
--   trig_rom (×2)   → cos/sin for tunnel rotation + ship position
--   spaceship       → tunnel_angle, ship_rel_angle, game_state …
--   obstacle_manager → obs*/ring* depth/face/active  (num_faces="1000")
--   tunnel          → red_raw/green_raw/blue_raw  (2-clock pipeline)
--   crt_effects     → crt_r/crt_g/crt_b           (1 more clock = 3 total)
--   start_screen    → sq_px/py/half/active/col     (particle animation)
--   screen_overlay  → RED/GREEN/BLUE               (1 more clock = 4 total)
--     IDLE     : start screen (particle squares + circle + "PRESS START")
--     PLAYING  : tunnel pass-through
--     GAME OVER: "GAME OVER" text on black background
--   snes_controller → buttons
--
-- Frame counter drives: frame_tick (1 pulse/frame), scroll (1/frame),
-- hue_cnt (1/frame — palette colour cycle ~3.6 s at 72 Hz).
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

    constant FRAME_CLOCKS : integer := 692640;   -- 800×600 @ 72 Hz total pixels
    constant SPEED_DIV    : integer := 1;        -- scroll every N frames
    constant SHIP_RADIUS  : integer := 285;      -- ship orbit radius (px from VP)

    -- ── VGA ──────────────────────────────────────────────────────────
    signal hsync_s  : std_logic;
    signal vsync_s  : std_logic;
    signal video_on : std_logic;
    signal hpos     : std_logic_vector(9 downto 0);
    signal vpos     : std_logic_vector(9 downto 0);

    -- ── Frame timing ─────────────────────────────────────────────────
    signal frame_cnt  : integer range 0 to FRAME_CLOCKS - 1 := 0;
    signal speed_cnt  : integer range 0 to SPEED_DIV - 1    := 0;
    signal scroll     : unsigned(7 downto 0) := (others => '0');
    signal hue_cnt    : unsigned(9 downto 0) := (others => '0');
    signal frame_tick : std_logic := '0';

    -- ── SNES ─────────────────────────────────────────────────────────
    signal buttons : std_logic_vector(15 downto 0);

    -- ── Spaceship FSM ─────────────────────────────────────────────────
    signal game_state     : std_logic_vector(1 downto 0);
    signal tunnel_angle   : std_logic_vector(15 downto 0);
    signal ship_scr_angle : std_logic_vector(15 downto 0);
    signal ship_rel_angle : std_logic_vector(15 downto 0);
    signal reset_game     : std_logic;
    signal difficulty     : std_logic_vector(4 downto 0);

    -- ── Trig ROMs ────────────────────────────────────────────────────
    signal cos_t : signed(8 downto 0);
    signal sin_t : signed(8 downto 0);
    signal cos_s : signed(8 downto 0);
    signal sin_s : signed(8 downto 0);

    -- ── Ship screen position ──────────────────────────────────────────
    signal ship_x_r : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal ship_y_r : unsigned(9 downto 0) := to_unsigned(300, 10);

    -- ── Obstacle manager ──────────────────────────────────────────────
    signal obs0_depth, obs1_depth, obs2_depth, obs3_depth,
           obs4_depth, obs5_depth, obs6_depth, obs7_depth : std_logic_vector(7 downto 0);
    signal obs0_face,  obs1_face,  obs2_face,  obs3_face,
           obs4_face,  obs5_face,  obs6_face,  obs7_face  : std_logic_vector(2 downto 0);
    signal obs0_act,   obs1_act,   obs2_act,   obs3_act,
           obs4_act,   obs5_act,   obs6_act,   obs7_act   : std_logic;
    signal collision        : std_logic;   -- from obstacle_manager (unused, always '0')
    signal ring0_depth_s, ring1_depth_s : std_logic_vector(7 downto 0);
    signal ring0_act_s,   ring1_act_s   : std_logic;
    signal ring0_face_s                 : std_logic_vector(2 downto 0);
    signal ring_collected_s             : std_logic;   -- from obstacle_manager (unused)

    -- ── Pixel-based collision from tunnel renderer ────────────────────
    signal ship_in_obs_s     : std_logic := '0';  -- ship center inside obstacle fill
    signal ship_on_coin_s    : std_logic := '0';  -- ship center overlaps coin band
    signal ship_on_coin_prev : std_logic := '0';  -- previous cycle (for rising-edge detect)

    -- ── BCD score counter (3 digits: hundreds / tens / ones) ─────────────
    signal score_hundreds : unsigned(3 downto 0) := (others => '0');
    signal score_tens     : unsigned(3 downto 0) := (others => '0');
    signal score_ones     : unsigned(3 downto 0) := (others => '0');

    -- Octagon fixed — obstacle_manager still takes num_faces port
    constant NUM_FACES_OCT : std_logic_vector(3 downto 0) := "1000";  -- 8

    -- ── Tunnel → CRT intermediate ─────────────────────────────────────
    signal red_raw   : std_logic_vector(3 downto 0);
    signal green_raw : std_logic_vector(3 downto 0);
    signal blue_raw  : std_logic_vector(3 downto 0);

    -- ── CRT → screen_overlay intermediate ────────────────────────────
    signal crt_r : std_logic_vector(3 downto 0);
    signal crt_g : std_logic_vector(3 downto 0);
    signal crt_b : std_logic_vector(3 downto 0);

    -- ── Start screen particle data ────────────────────────────────────
    signal sq_px_s     : std_logic_vector(65 downto 0);
    signal sq_py_s     : std_logic_vector(65 downto 0);
    signal sq_half_s   : std_logic_vector(47 downto 0);
    signal sq_active_s : std_logic_vector(5  downto 0);
    signal sq_col_s    : std_logic_vector(17 downto 0);

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
    -- Trig ROMs (two instances, shared entity)
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
    -- Ship screen position
    --   ship_x = 400 + cos_s × SHIP_RADIUS / 256
    --   ship_y = 300 + sin_s × SHIP_RADIUS / 256
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
            sy_pos  := to_signed(300, 11) + resize(sy_off, 11);
            ship_x_r <= unsigned(sx_pos(9 downto 0));
            ship_y_r <= unsigned(sy_pos(9 downto 0));
        end if;
    end process;

    -- ================================================================
    -- Tunnel renderer (consolidated — replaces coord_transform,
    -- tunnel_core, tunnel_background, colour_palette, tunnel_renderer,
    -- stage_manager)
    -- ================================================================
    u_tunnel : entity work.tunnel
        port map (
            clk_50            => CLK_50,
            video_on          => video_on,
            hpos              => hpos,
            vpos              => vpos,
            cos_t             => cos_t,
            sin_t             => sin_t,
            scroll            => std_logic_vector(scroll),
            hue_cnt           => hue_cnt,
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
            obs4_depth        => obs4_depth,
            obs4_face_sector  => obs4_face,
            obs4_active       => obs4_act,
            obs5_depth        => obs5_depth,
            obs5_face_sector  => obs5_face,
            obs5_active       => obs5_act,
            obs6_depth        => obs6_depth,
            obs6_face_sector  => obs6_face,
            obs6_active       => obs6_act,
            obs7_depth        => obs7_depth,
            obs7_face_sector  => obs7_face,
            obs7_active       => obs7_act,
            ring0_depth       => ring0_depth_s,
            ring0_active      => ring0_act_s,
            ring0_face_sector => ring0_face_s,
            ring1_depth       => ring1_depth_s,
            ring1_active      => ring1_act_s,
            red               => red_raw,
            green             => green_raw,
            blue              => blue_raw,
            ship_in_obs       => ship_in_obs_s,
            ship_on_coin      => ship_on_coin_s
        );

    -- ================================================================
    -- CRT post-processing
    -- Output goes to crt_r/g/b (not directly to pins — screen_overlay
    -- is the final stage that drives RED/GREEN/BLUE).
    -- ================================================================
    u_crt : entity work.crt_effects
        port map (
            clk_50    => CLK_50,
            red_in    => red_raw,
            green_in  => green_raw,
            blue_in   => blue_raw,
            hpos      => hpos,
            vpos      => vpos,
            hue_cnt   => std_logic_vector(hue_cnt),
            scroll    => std_logic_vector(scroll),
            red_out   => crt_r,
            green_out => crt_g,
            blue_out  => crt_b
        );

    -- ================================================================
    -- Start screen particle animation
    -- Runs at all times; output is only used by screen_overlay when
    -- game_state = "00" (IDLE).
    -- ================================================================
    u_start : entity work.start_screen
        port map (
            CLK_50     => CLK_50,
            frame_tick => frame_tick,
            buttons    => buttons,
            sq_px      => sq_px_s,
            sq_py      => sq_py_s,
            sq_half    => sq_half_s,
            sq_active  => sq_active_s,
            sq_col     => sq_col_s
        );

    -- ================================================================
    -- Screen overlay — final rendering stage
    -- Muxes between start screen, tunnel pass-through, and game-over
    -- text based on game_state.  Drives RED/GREEN/BLUE directly.
    -- ================================================================
    u_overlay : entity work.screen_overlay
        port map (
            clk_50     => CLK_50,
            frame_tick => frame_tick,
            hpos       => hpos,
            vpos      => vpos,
            video_on  => video_on,
            game_state=> game_state,
            crt_r     => crt_r,
            crt_g     => crt_g,
            crt_b     => crt_b,
            sq_px     => sq_px_s,
            sq_py     => sq_py_s,
            sq_half   => sq_half_s,
            sq_active => sq_active_s,
            sq_col    => sq_col_s,
            score     => std_logic_vector(score_hundreds & score_tens & score_ones),
            red_out   => RED,
            green_out => GREEN,
            blue_out  => BLUE
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
    -- Spaceship FSM
    -- ================================================================
    u_ship : entity work.spaceship
        port map (
            clk_50         => CLK_50,
            frame_tick     => frame_tick,
            buttons        => buttons,
            collision      => ship_in_obs_s,  -- pixel-based from tunnel.vhd
            game_state     => game_state,
            tunnel_angle   => tunnel_angle,
            ship_scr_angle => ship_scr_angle,
            ship_rel_angle => ship_rel_angle,
            reset_game     => reset_game,
            difficulty     => difficulty
        );

    -- ================================================================
    -- Obstacle manager (entity name unchanged, num_faces fixed = 8)
    -- ================================================================
    u_obs : entity work.obstacle_manager
        port map (
            clk_50           => CLK_50,
            frame_tick       => frame_tick,
            game_active      => game_state(0),
            reset_game       => reset_game,
            ship_rel_angle   => ship_rel_angle,
            difficulty       => difficulty,
            num_faces        => NUM_FACES_OCT,
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
            obs4_depth       => obs4_depth,
            obs4_face_sector => obs4_face,
            obs4_active      => obs4_act,
            obs5_depth       => obs5_depth,
            obs5_face_sector => obs5_face,
            obs5_active      => obs5_act,
            obs6_depth       => obs6_depth,
            obs6_face_sector => obs6_face,
            obs6_active      => obs6_act,
            obs7_depth       => obs7_depth,
            obs7_face_sector => obs7_face,
            obs7_active      => obs7_act,
            collision        => collision,
            ring0_depth      => ring0_depth_s,
            ring0_active     => ring0_act_s,
            ring0_face_sector => ring0_face_s,
            ring1_depth      => ring1_depth_s,
            ring1_active     => ring1_act_s,
            ring_collected   => ring_collected_s
        );

    -- ================================================================
    -- Frame counter / scroll / hue_cnt / frame_tick
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

    -- ================================================================
    -- BCD score counter — increments on the RISING EDGE of ship_on_coin_s
    -- (pixel-based coin contact from tunnel.vhd).  Edge detection ensures
    -- only one increment per coin passage even though ship_on_coin_s stays
    -- high for several cycles while the coin band crosses the ship pixel.
    -- Resets to 000 when spaceship FSM asserts reset_game.
    -- ================================================================
    process(CLK_50)
    begin
        if rising_edge(CLK_50) then
            ship_on_coin_prev <= ship_on_coin_s;
            if reset_game = '1' then
                score_ones     <= (others => '0');
                score_tens     <= (others => '0');
                score_hundreds <= (others => '0');
            elsif ship_on_coin_s = '1' and ship_on_coin_prev = '0' then
                -- Rising edge: coin band just reached ship center
                if score_ones = 9 then
                    score_ones <= (others => '0');
                    if score_tens = 9 then
                        score_tens <= (others => '0');
                        if score_hundreds /= 9 then
                            score_hundreds <= score_hundreds + 1;
                        end if;
                    else
                        score_tens <= score_tens + 1;
                    end if;
                else
                    score_ones <= score_ones + 1;
                end if;
            end if;
        end if;
    end process;

end rtl;
