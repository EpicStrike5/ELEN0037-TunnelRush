library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- spaceship.vhd  —  Operius physics FSM  (with tunnel-lag inertia)
-- ================================================================
-- Motion model
-- ─────────────
--   ship_rel_ang_r  — true fast position (used only internally as the
--                     lag source; never exported directly).
--
--   tunnel_disp_r   — lagged displayed position.  Recovers toward
--                     ship_rel_ang_r at 1/2^LAG_SHIFT per frame.
--                     This IS what is exported as ship_rel_angle.
--
--   ship_scr_angle  = BOTTOM_ANGLE + (ship_rel_ang_r − tunnel_disp_r)
--                     Ship sprite appears offset from the bottom spoke
--                     by the current lag amount.
--
--   tunnel_angle    = BOTTOM_ANGLE − tunnel_disp_r
--                     Tunnel counter-rotates using the lagged position.
--
--   ship_rel_angle  = tunnel_disp_r   ← KEY DESIGN CHOICE
--                     Collision detection and obstacle placement both
--                     use the VISUAL (lagged) position.  The player
--                     dies when their visual position is on a blocked
--                     face — never "for no reason".
--
-- Visual inertia effect
-- ─────────────────────
--   Hold button → ship_rel_ang_r races ahead, tunnel lags →
--   sprite swings off the bottom spoke (≤ ~17° / ~86 px at MAX_VEL).
--   Release → drag kills velocity in ~3 frames, tunnel catches up
--   over ~8 frames → sprite drifts back to bottom.
--
-- Speed tuning (change these constants)
-- ────────────────────────────────────────
--   INIT_DIFF    4   starting difficulty (0 = slowest, 15 = fastest)
--                    ↑ crank this up to test higher starting speeds
--   MAX_VEL    400   max angular speed per frame  (was 256)
--   ACCEL       32   velocity increment per frame held  (was 24)
--   DRAG_SHIFT   2   drag = vel >> 2  (~3-frame half-life)
--   LAG_SHIFT    3   tunnel recovery = diff >> 3 per frame (~8-frame τ)
--   DIFF_FRAMES 300  frames between automatic difficulty increments
-- ================================================================

entity spaceship is
    port (
        clk_50         : in  std_logic;
        frame_tick     : in  std_logic;
        buttons        : in  std_logic_vector(15 downto 0);
        collision      : in  std_logic;
        -- State
        game_state     : out std_logic_vector(1 downto 0);
        -- Angles (16-bit unsigned, 0-65535 = 0°-360°)
        tunnel_angle   : out std_logic_vector(15 downto 0);
        ship_scr_angle : out std_logic_vector(15 downto 0);
        ship_rel_angle : out std_logic_vector(15 downto 0);  -- = tunnel_disp_r
        -- Control
        reset_game     : out std_logic;
        difficulty     : out std_logic_vector(4 downto 0)
    );
end spaceship;

architecture rtl of spaceship is

    constant BTN_START    : integer := 3;
    constant BTN_LEFT     : integer := 7;
    constant BTN_RIGHT    : integer := 6;

    -- ── Speed / feel ─────────────────────────────────────────────────
    constant INIT_DIFF    : integer := 10;    -- starting difficulty (0-20)
    constant MAX_DIFF     : integer := 20;    -- difficulty cap
    -- MAX_VEL / ACCEL are per-difficulty variables (see lookup in S_PLAYING).
    -- Rule: at every level, half-turn frames < obstacle travel frames.
    constant DRAG_SHIFT   : integer := 2;     -- drag = vel >> 2  (~3-frame half-life)
    constant LAG_SHIFT    : integer := 2;     -- tunnel recovery = diff >> 2  (lag_ss = 4×vel ≤ 19°)
    -- ─────────────────────────────────────────────────────────────────

    constant BOTTOM_ANGLE : integer := 16384;   -- 90° = ship at screen bottom
    constant DIFF_FRAMES  : integer := 300;   -- frames between difficulty increments (~4 s)

    type state_t is (S_IDLE, S_PLAYING, S_GAME_OVER);
    signal state : state_t := S_IDLE;

    signal ship_rel_ang_r : unsigned(15 downto 0) := (others => '0'); -- fast/true
    signal tunnel_disp_r  : unsigned(15 downto 0) := (others => '0'); -- lagged/visual
    signal ship_rel_vel   : signed(15 downto 0)   := (others => '0');
    signal difficulty_r   : unsigned(4 downto 0)  := (others => '0');
    signal diff_cnt       : integer range 0 to DIFF_FRAMES - 1 := 0;
    signal buttons_prev   : std_logic_vector(15 downto 0) := (others => '0');
    signal reset_r        : std_logic := '0';

begin

    game_state <= "00" when state = S_IDLE    else
                  "01" when state = S_PLAYING  else
                  "10";

    -- Ship sprite drifts away from bottom by exactly the lag offset.
    ship_scr_angle <= std_logic_vector(
                          to_unsigned(BOTTOM_ANGLE, 16)
                        + (ship_rel_ang_r - tunnel_disp_r));

    -- Tunnel counter-rotates using the lagged (visual) position.
    tunnel_angle   <= std_logic_vector(
                          to_unsigned(BOTTOM_ANGLE, 16) - tunnel_disp_r);

    -- Export the TRUE (fast) position for collision and obstacle placement.
    -- Proof: ship sprite tunnel-space position = ship_scr_angle − tunnel_angle
    --   = (BOTTOM_ANGLE + lag) − (BOTTOM_ANGLE − tunnel_disp_r)
    --   = lag + tunnel_disp_r = ship_rel_ang_r  ← matches collision exactly.
    -- tunnel_disp_r is a rendering lag only; it does not change where the
    -- sprite visually sits relative to the tunnel walls.
    ship_rel_angle <= std_logic_vector(ship_rel_ang_r);

    difficulty  <= std_logic_vector(difficulty_r);
    reset_game  <= reset_r;

    process(clk_50)
        variable start_edge : std_logic;
        variable left_held  : std_logic;
        variable right_held : std_logic;
        variable new_vel    : signed(15 downto 0);
        variable drag       : signed(15 downto 0);
        variable lag_diff   : signed(15 downto 0);
        variable lag_step   : signed(15 downto 0);
        variable max_vel_v  : integer range 200 to 700;
        variable accel_v    : integer range 16 to 56;
    begin
        if rising_edge(clk_50) then
            reset_r <= '0';

            -- ── Collision: polled every clock (not just frame_tick) ───────
            -- obstacle_manager asserts collision one cycle after frame_tick;
            -- checking here catches that single-cycle pulse.
            if state = S_PLAYING and collision = '1' then
                state        <= S_GAME_OVER;
                ship_rel_vel <= (others => '0');
            end if;

            if frame_tick = '1' then
                start_edge   := buttons(BTN_START) and not buttons_prev(BTN_START);
                left_held    := buttons(BTN_LEFT);
                right_held   := buttons(BTN_RIGHT);
                buttons_prev <= buttons;

                case state is

                    -- ── IDLE ──────────────────────────────────────────────
                    when S_IDLE =>
                        ship_rel_vel <= (others => '0');
                        difficulty_r <= (others => '0');
                        diff_cnt     <= 0;
                        -- No lag while idle — keep tunnel display in sync.
                        tunnel_disp_r <= ship_rel_ang_r;

                        if start_edge = '1' then
                            state          <= S_PLAYING;
                            reset_r        <= '1';
                            ship_rel_ang_r <= (others => '0');
                            tunnel_disp_r  <= (others => '0');
                            -- Start at INIT_DIFF instead of 0 for faster pacing.
                            -- Change the INIT_DIFF constant at the top of this file.
                            difficulty_r   <= to_unsigned(INIT_DIFF, 5);
                            diff_cnt       <= 0;
                        end if;

                    -- ── PLAYING ───────────────────────────────────────────
                    when S_PLAYING =>
                        -- Per-difficulty speed lookup (tied to adv_div thresholds).
                        -- Starts at MAX_VEL=400 (d=9-11, INIT_DIFF=10), tops at 700.
                        -- Steady-state visual lag = 4×MAX_VEL (LAG_SHIFT=2):
                        --   d<3:  200→3200 units≈5°   d=3-5: 250→4°
                        --   d=6-8:300→8°   d=9-11:400→11°
                        --   d=12-14:550→15°  d≥15:700→19°
                        if    to_integer(difficulty_r) >= 15 then max_vel_v := 700; accel_v := 56;
                        elsif to_integer(difficulty_r) >= 12 then max_vel_v := 550; accel_v := 44;
                        elsif to_integer(difficulty_r) >= 9  then max_vel_v := 400; accel_v := 32;
                        elsif to_integer(difficulty_r) >= 6  then max_vel_v := 300; accel_v := 24;
                        elsif to_integer(difficulty_r) >= 3  then max_vel_v := 250; accel_v := 20;
                        else                                        max_vel_v := 200; accel_v := 16;
                        end if;

                        -- Ship physics
                        new_vel := ship_rel_vel;
                        if right_held = '1' then
                            new_vel := ship_rel_vel + accel_v;
                            if new_vel > max_vel_v then
                                new_vel := to_signed(max_vel_v, 16);
                            end if;
                        elsif left_held = '1' then
                            new_vel := ship_rel_vel - accel_v;
                            if new_vel < -max_vel_v then
                                new_vel := to_signed(-max_vel_v, 16);
                            end if;
                        else
                            drag := shift_right(ship_rel_vel, DRAG_SHIFT);
                            if to_integer(drag) = 0 then
                                new_vel := (others => '0');
                            else
                                new_vel := ship_rel_vel - drag;
                            end if;
                        end if;

                        ship_rel_vel   <= new_vel;
                        ship_rel_ang_r <= ship_rel_ang_r
                                        + unsigned(resize(new_vel, 16));

                        -- Tunnel lag: move tunnel_disp_r 1/4 of the way
                        -- toward ship_rel_ang_r each frame (LAG_SHIFT=2).
                        -- Minimum ±1 guarantees convergence when diff < 4.
                        lag_diff := signed(ship_rel_ang_r - tunnel_disp_r);
                        lag_step := shift_right(lag_diff, LAG_SHIFT);
                        if lag_step = 0 and lag_diff /= 0 then
                            if lag_diff > 0 then lag_step := to_signed( 1, 16);
                            else                 lag_step := to_signed(-1, 16);
                            end if;
                        end if;
                        tunnel_disp_r <= unsigned(signed(tunnel_disp_r) + lag_step);

                        -- Difficulty ramp
                        if diff_cnt = DIFF_FRAMES - 1 then
                            diff_cnt <= 0;
                            if difficulty_r < to_unsigned(MAX_DIFF, 5) then
                                difficulty_r <= difficulty_r + 1;
                            end if;
                        else
                            diff_cnt <= diff_cnt + 1;
                        end if;

                    -- ── GAME OVER ─────────────────────────────────────────
                    -- Tunnel continues to catch up so the ship drifts smoothly
                    -- back to the bottom spoke while the game-over screen shows.
                    when S_GAME_OVER =>
                        lag_diff := signed(ship_rel_ang_r - tunnel_disp_r);
                        lag_step := shift_right(lag_diff, LAG_SHIFT);
                        if lag_step = 0 and lag_diff /= 0 then
                            if lag_diff > 0 then lag_step := to_signed( 1, 16);
                            else                 lag_step := to_signed(-1, 16);
                            end if;
                        end if;
                        tunnel_disp_r <= unsigned(signed(tunnel_disp_r) + lag_step);

                        if start_edge = '1' then
                            state        <= S_IDLE;
                            difficulty_r <= (others => '0');
                        end if;

                end case;
            end if;  -- frame_tick
        end if;  -- rising_edge
    end process;

end rtl;
