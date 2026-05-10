library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--
-- spaceship.vhd
--
-- Input   : Clock and frame tick
--           SNES directional and start button state
--           Pixel collision flag from tunnel.vhd
--           Wave spawn tick from obstacle_manager.vhd
-- Output  : Game state (IDLE / PLAYING / GAME OVER)
--           Rotation angles for trig_rom.vhd :
--
--					tunnel_angle   — tunnel counter-rotation (visual lag)
--					ship_scr_angle — ship sprite screen position
--					ship_rel_angle — true angular position (collision)
--
--           Difficulty level and new-game reset pulse
-- Utility : Main game FSM.  Implements angular physics with inertia,
--           tunnel visual lag, and wave-based difficulty ramp.
--           Connects to trig_rom, obstacle_manager, tunnel, screen_overlay
--

entity spaceship is
    port (
        clk_50         : in  std_logic;
        frame_tick     : in  std_logic;
        buttons        : in  std_logic_vector(15 downto 0);
        collision      : in  std_logic;
        wave_tick      : in  std_logic;
        game_state     : out std_logic_vector(1 downto 0);
        tunnel_angle   : out std_logic_vector(15 downto 0);
        ship_scr_angle : out std_logic_vector(15 downto 0);
        ship_rel_angle : out std_logic_vector(15 downto 0);
        reset_game     : out std_logic;
        difficulty     : out std_logic_vector(4 downto 0)
    );
end spaceship;

architecture rtl of spaceship is

    -- Button indices
    constant BTN_START : integer := 3;
    constant BTN_LEFT  : integer := 7;
    constant BTN_RIGHT : integer := 6;

    -- Difficulty ramp parameters
    constant INIT_DIFF      : integer := 10;  -- starting difficulty (0..20)
    constant MAX_DIFF       : integer := 20;
    constant WAVES_PER_DIFF : integer := 5;   -- obstacle waves to survive per difficulty step

    -- Physics parameters
    constant DRAG_SHIFT : integer := 2;  -- drag = vel >> 2  (~3-frame half-life)
    constant LAG_SHIFT  : integer := 2;  -- tunnel lag recovery = diff >> 2 per frame

    -- Angle encoding: 0–65535 = 0°–360°; ship rests at 90° (screen bottom)
    constant BOTTOM_ANGLE : integer := 16384;

    type state_t is (S_IDLE, S_PLAYING, S_GAME_OVER);
    signal state : state_t := S_IDLE;

    signal ship_rel_ang_r : unsigned(15 downto 0) := (others => '0'); -- true (fast) position
    signal tunnel_disp_r  : unsigned(15 downto 0) := (others => '0'); -- lagged display position
    signal ship_rel_vel   : signed(15 downto 0)   := (others => '0');
    signal difficulty_r   : unsigned(4 downto 0)  := (others => '0');
    signal wave_cnt       : integer range 0 to WAVES_PER_DIFF - 1 := 0;
    signal buttons_prev   : std_logic_vector(15 downto 0) := (others => '0');
    signal reset_r        : std_logic := '0';

begin

    -- FSM state encoding
    game_state <= "00" when state = S_IDLE    else
                  "01" when state = S_PLAYING  else
                  "10";

    -- Ship sprite drifts off bottom spoke by the current inertia lag amount
    ship_scr_angle <= std_logic_vector(
                          to_unsigned(BOTTOM_ANGLE, 16)
                        + (ship_rel_ang_r - tunnel_disp_r));

    -- Tunnel counter-rotates using the lagged (visual) position
    tunnel_angle   <= std_logic_vector(
                          to_unsigned(BOTTOM_ANGLE, 16) - tunnel_disp_r);

    -- True position exported for collision detection
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
        variable max_vel_v  : integer range 200 to 850;
        variable accel_v    : integer range 16 to 68;
    begin
        if rising_edge(clk_50) then
            reset_r <= '0';

            -- Collision check: polled every clock to catch the single-cycle pulse
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

                    -- IDLE: wait for Start button, keep tunnel in sync with ship
                    when S_IDLE =>
                        ship_rel_vel  <= (others => '0');
                        difficulty_r  <= (others => '0');
                        wave_cnt      <= 0;
                        tunnel_disp_r <= ship_rel_ang_r;

                        if start_edge = '1' then
                            state          <= S_PLAYING;
                            reset_r        <= '1';
                            ship_rel_ang_r <= (others => '0');
                            tunnel_disp_r  <= (others => '0');
                            difficulty_r   <= to_unsigned(INIT_DIFF, 5);
                            wave_cnt       <= 0;
                        end if;

                    -- PLAYING: angular physics + tunnel lag + difficulty ramp
                    when S_PLAYING =>
                        -- Per-difficulty speed lookup (MAX_VEL and ACCEL scale with difficulty)
                        if    to_integer(difficulty_r) >= 18 then max_vel_v := 850; accel_v := 68;
                        elsif to_integer(difficulty_r) >= 15 then max_vel_v := 700; accel_v := 56;
                        elsif to_integer(difficulty_r) >= 12 then max_vel_v := 550; accel_v := 44;
                        elsif to_integer(difficulty_r) >= 9  then max_vel_v := 400; accel_v := 32;
                        elsif to_integer(difficulty_r) >= 6  then max_vel_v := 300; accel_v := 24;
                        elsif to_integer(difficulty_r) >= 3  then max_vel_v := 250; accel_v := 20;
                        else                                        max_vel_v := 200; accel_v := 16;
                        end if;

                        -- Angular velocity: accelerate, decelerate, or apply drag
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

                        -- Tunnel lag: tunnel_disp_r recovers 1/4 toward ship_rel_ang_r per frame
                        lag_diff := signed(ship_rel_ang_r - tunnel_disp_r);
                        lag_step := shift_right(lag_diff, LAG_SHIFT);
                        if lag_step = 0 and lag_diff /= 0 then
                            if lag_diff > 0 then lag_step := to_signed( 1, 16);
                            else                 lag_step := to_signed(-1, 16);
                            end if;
                        end if;
                        tunnel_disp_r <= unsigned(signed(tunnel_disp_r) + lag_step);

                        -- Difficulty ramp: increment every WAVES_PER_DIFF waves survived
                        if wave_tick = '1' then
                            if wave_cnt = WAVES_PER_DIFF - 1 then
                                wave_cnt <= 0;
                                if difficulty_r < to_unsigned(MAX_DIFF, 5) then
                                    difficulty_r <= difficulty_r + 1;
                                end if;
                            else
                                wave_cnt <= wave_cnt + 1;
                            end if;
                        end if;

                    -- GAME OVER: tunnel continues to catch up; wait for Start to return to IDLE
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
            end if;
        end if;
    end process;

end rtl;
