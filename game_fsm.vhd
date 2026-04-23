library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- game_fsm.vhd
-- ================================================================
-- Manages game state and the continuous angular motion model.
--
-- ANGLE SYSTEM (16-bit unsigned, 0-65535 = 0°-360°):
--   tunnel_angle     : absolute rotation of the tunnel on screen
--   ship_scr_angle   : absolute angle of the ship on screen
--                      (= tunnel_angle + rel_angle)
--   ship_rel_angle   : ship's angle IN the tunnel frame
--                      used by obstacle_manager for collision
--
-- MOTION MODEL (per frame_tick):
--   Button held → tun_vel accelerates toward ±MAX_VEL
--   Released    → tun_vel decays toward 0  (drag = vel >> DRAG_SHIFT)
--   tunnel_angle  += tun_vel
--   ship_scr_angle += tun_vel + (tun_vel >> 1)   [1.5× tunnel speed]
--   ship_rel_angle  = ship_scr_angle - tunnel_angle
--                   = accumulates (tun_vel >> 1) per frame
--
-- TUNABLE:
--   MAX_VEL    int  512   max angular velocity (512/65536*360 ≈ 2.8°/frame)
--   ACCEL      int   32   velocity increment per frame when button held
--   DRAG_SHIFT int    3   drag = vel >> 3  (half-life ≈ 5 frames)
-- ================================================================

entity game_fsm is
    port (
        clk_50         : in  std_logic;
        frame_tick     : in  std_logic;
        buttons        : in  std_logic_vector(15 downto 0);
        collision      : in  std_logic;
        -- State outputs
        game_state     : out std_logic_vector(1 downto 0);  -- "00"=IDLE "01"=PLAY "10"=OVER
        -- Angle outputs
        tunnel_angle   : out std_logic_vector(15 downto 0);
        ship_scr_angle : out std_logic_vector(15 downto 0);
        ship_rel_angle : out std_logic_vector(15 downto 0);
        -- Control outputs
        reset_game     : out std_logic;
        difficulty     : out std_logic_vector(3 downto 0)
    );
end game_fsm;

architecture rtl of game_fsm is

    constant BTN_START  : integer := 3;
    constant BTN_LEFT   : integer := 6;
    constant BTN_RIGHT  : integer := 7;

    constant MAX_VEL    : integer := 512;
    constant ACCEL      : integer := 32;
    constant DRAG_SHIFT : integer := 3;
    constant DIFF_FRAMES: integer := 300;

    type state_t is (S_IDLE, S_PLAYING, S_GAME_OVER);
    signal state : state_t := S_IDLE;

    -- Angles (unsigned 16-bit, wrapping)
    signal tun_ang_r  : unsigned(15 downto 0) := (others => '0');
    signal ship_ang_r : unsigned(15 downto 0) := (others => '0');

    -- Signed velocity (range -MAX_VEL .. +MAX_VEL)
    signal tun_vel    : signed(15 downto 0) := (others => '0');

    signal difficulty_r : unsigned(3 downto 0) := (others => '0');
    signal diff_cnt     : integer range 0 to DIFF_FRAMES - 1 := 0;

    signal buttons_prev : std_logic_vector(15 downto 0) := (others => '0');
    signal reset_r      : std_logic := '0';

begin

    game_state     <= "00" when state = S_IDLE      else
                      "01" when state = S_PLAYING   else
                      "10";
    tunnel_angle   <= std_logic_vector(tun_ang_r);
    ship_scr_angle <= std_logic_vector(ship_ang_r);
    ship_rel_angle <= std_logic_vector(ship_ang_r - tun_ang_r);   -- wrapping subtraction
    difficulty     <= std_logic_vector(difficulty_r);
    reset_game     <= reset_r;

    process(clk_50)
        variable start_edge : std_logic;
        variable left_held  : std_logic;
        variable right_held : std_logic;
        variable new_vel    : signed(15 downto 0);
        variable ship_vel   : signed(15 downto 0);
        variable drag       : signed(15 downto 0);
    begin
        if rising_edge(clk_50) then
            reset_r <= '0';

            if frame_tick = '1' then
                start_edge := buttons(BTN_START) and not buttons_prev(BTN_START);
                left_held  := buttons(BTN_LEFT);
                right_held := buttons(BTN_RIGHT);
                buttons_prev <= buttons;

                case state is

                    -- ---- IDLE: wait for Start ----
                    when S_IDLE =>
                        tun_vel     <= (others => '0');
                        difficulty_r <= (others => '0');
                        diff_cnt    <= 0;
                        if start_edge = '1' then
                            state  <= S_PLAYING;
                            reset_r <= '1';
                            -- Reset angles to zero
                            tun_ang_r  <= (others => '0');
                            ship_ang_r <= (others => '0');
                        end if;

                    -- ---- PLAYING: physics update ----
                    when S_PLAYING =>
                        -- Velocity update
                        new_vel := tun_vel;
                        if right_held = '1' then
                            new_vel := tun_vel + ACCEL;
                            if new_vel > MAX_VEL then new_vel := to_signed(MAX_VEL, 16); end if;
                        elsif left_held = '1' then
                            new_vel := tun_vel - ACCEL;
                            if new_vel < -MAX_VEL then new_vel := to_signed(-MAX_VEL, 16); end if;
                        else
                            -- Drag: subtract vel/8, keeping sign
                            drag    := shift_right(tun_vel, DRAG_SHIFT);
                            new_vel := tun_vel - drag;
                        end if;
                        tun_vel <= new_vel;

                        -- Angle update
                        -- tunnel  moves at new_vel
                        -- ship    moves at new_vel + new_vel/2  (1.5×)
                        ship_vel   := new_vel + shift_right(new_vel, 1);
                        tun_ang_r  <= tun_ang_r  + unsigned(resize(new_vel,  16));
                        ship_ang_r <= ship_ang_r + unsigned(resize(ship_vel, 16));

                        -- Difficulty ramp
                        if diff_cnt = DIFF_FRAMES - 1 then
                            diff_cnt <= 0;
                            if difficulty_r /= "1111" then
                                difficulty_r <= difficulty_r + 1;
                            end if;
                        else
                            diff_cnt <= diff_cnt + 1;
                        end if;

                        -- Collision → game over
                        if collision = '1' then
                            state   <= S_GAME_OVER;
                            tun_vel <= (others => '0');
                        end if;

                    -- ---- GAME OVER: wait for restart ----
                    when S_GAME_OVER =>
                        if start_edge = '1' then
                            state        <= S_IDLE;
                            difficulty_r <= (others => '0');
                        end if;

                end case;
            end if;
        end if;
    end process;

end rtl;
