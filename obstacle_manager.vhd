library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--
-- obstacle_manager.vhd
--
-- Input   : Clock, frame tick, game active flag and reset pulse
--           Ship angular position from spaceship.vhd
--           Difficulty level from spaceship.vhd
-- Output  : State for 8 obstacle slots and 1 scoring coin :
--
--					depth       — log-depth position in the tunnel
--					face_sector — which octagon face is blocked
--					active      — slot occupied flag
--
--           Wave spawn tick for spaceship.vhd
-- Utility : Spawns waves of 2–7 obstacles with one coin per wave.
--           Speed and spawn rate scale with difficulty :
--
--					adv_step / adv_div  — obstacle scroll speed
--					spawn_interval      — frames between new waves
--
--           Feeds tunnel.vhd for rendering and collision detection
--

entity obstacle_manager is
    port (
        clk_50          : in  std_logic;
        frame_tick      : in  std_logic;
        game_active     : in  std_logic;
        reset_game      : in  std_logic;
        ship_rel_angle  : in  std_logic_vector(15 downto 0);
        difficulty      : in  std_logic_vector(4 downto 0);
        num_faces       : in  std_logic_vector(3 downto 0);
        obs0_depth       : out std_logic_vector(7 downto 0);
        obs0_face_sector : out std_logic_vector(2 downto 0);
        obs0_active      : out std_logic;
        obs1_depth       : out std_logic_vector(7 downto 0);
        obs1_face_sector : out std_logic_vector(2 downto 0);
        obs1_active      : out std_logic;
        obs2_depth       : out std_logic_vector(7 downto 0);
        obs2_face_sector : out std_logic_vector(2 downto 0);
        obs2_active      : out std_logic;
        obs3_depth       : out std_logic_vector(7 downto 0);
        obs3_face_sector : out std_logic_vector(2 downto 0);
        obs3_active      : out std_logic;
        obs4_depth       : out std_logic_vector(7 downto 0);
        obs4_face_sector : out std_logic_vector(2 downto 0);
        obs4_active      : out std_logic;
        obs5_depth       : out std_logic_vector(7 downto 0);
        obs5_face_sector : out std_logic_vector(2 downto 0);
        obs5_active      : out std_logic;
        obs6_depth       : out std_logic_vector(7 downto 0);
        obs6_face_sector : out std_logic_vector(2 downto 0);
        obs6_active      : out std_logic;
        obs7_depth       : out std_logic_vector(7 downto 0);
        obs7_face_sector : out std_logic_vector(2 downto 0);
        obs7_active      : out std_logic;
        collision        : out std_logic;
        ring0_depth       : out std_logic_vector(7 downto 0);
        ring0_active      : out std_logic;
        ring0_face_sector : out std_logic_vector(2 downto 0);
        ring1_depth       : out std_logic_vector(7 downto 0);
        ring1_active      : out std_logic;
        ring_collected    : out std_logic;
        wave_tick         : out std_logic   -- 1-cycle pulse each time a new wave spawns
    );
end obstacle_manager;

architecture rtl of obstacle_manager is

    constant OBS_SPAWN_DEPTH   : integer := 80;   -- log-depth at which new obstacles appear
    constant OBS_COLLIDE_DEPTH : integer := 140;  -- log-depth at which slots are retired
    constant BASE_SPAWN        : integer := 360;  -- spawn interval at difficulty 0 (frames)
    constant MIN_SPAWN         : integer := 36;   -- minimum spawn interval (0.5 s at 72 Hz)

    type depth_arr_t is array (0 to 7) of unsigned(7 downto 0);
    type fsec_arr_t  is array (0 to 7) of std_logic_vector(2 downto 0);
    type act_arr_t   is array (0 to 7) of std_logic;

    signal obs_depth_r  : depth_arr_t := (others => to_unsigned(OBS_SPAWN_DEPTH, 8));
    signal obs_fsec_r   : fsec_arr_t  := (others => "000");
    signal obs_active_r : act_arr_t   := (others => '0');

    -- Scoring coin state
    signal coin_depth_r  : unsigned(7 downto 0)        := to_unsigned(OBS_SPAWN_DEPTH, 8);
    signal coin_fsec_r   : std_logic_vector(2 downto 0) := "000";
    signal coin_active_r : std_logic                    := '0';
    signal ring_coll_r   : std_logic                    := '0';

    signal spawn_cnt    : integer range 0 to BASE_SPAWN - 1 := 0;
    signal adv_cnt      : integer range 0 to 5              := 0;
    signal lfsr         : std_logic_vector(7 downto 0)      := "10110101";
    signal collision_r  : std_logic := '0';
    signal wave_tick_r  : std_logic := '0';

    signal spawn_interval : integer range MIN_SPAWN to BASE_SPAWN;
    signal adv_div        : integer range 1 to 6;
    signal adv_step       : unsigned(7 downto 0) := to_unsigned(1, 8);  -- depth units per advance tick

begin

    -- Output wiring for obstacle slots
    obs0_depth <= std_logic_vector(obs_depth_r(0));
    obs0_face_sector <= obs_fsec_r(0);  obs0_active <= obs_active_r(0);
    obs1_depth <= std_logic_vector(obs_depth_r(1));
    obs1_face_sector <= obs_fsec_r(1);  obs1_active <= obs_active_r(1);
    obs2_depth <= std_logic_vector(obs_depth_r(2));
    obs2_face_sector <= obs_fsec_r(2);  obs2_active <= obs_active_r(2);
    obs3_depth <= std_logic_vector(obs_depth_r(3));
    obs3_face_sector <= obs_fsec_r(3);  obs3_active <= obs_active_r(3);
    obs4_depth <= std_logic_vector(obs_depth_r(4));
    obs4_face_sector <= obs_fsec_r(4);  obs4_active <= obs_active_r(4);
    obs5_depth <= std_logic_vector(obs_depth_r(5));
    obs5_face_sector <= obs_fsec_r(5);  obs5_active <= obs_active_r(5);
    obs6_depth <= std_logic_vector(obs_depth_r(6));
    obs6_face_sector <= obs_fsec_r(6);  obs6_active <= obs_active_r(6);
    obs7_depth <= std_logic_vector(obs_depth_r(7));
    obs7_face_sector <= obs_fsec_r(7);  obs7_active <= obs_active_r(7);
    collision  <= collision_r;

    -- Coin outputs
    ring0_depth       <= std_logic_vector(coin_depth_r);
    ring0_active      <= coin_active_r;
    ring0_face_sector <= coin_fsec_r;
    ring_collected    <= ring_coll_r;

    -- ring1 disabled
    ring1_depth <= (others => '0');  ring1_active <= '0';
    wave_tick   <= wave_tick_r;

    -- Spawn interval: BASE_SPAWN − difficulty×20, clamped to MIN_SPAWN
    process(difficulty)
        variable d, raw : integer;
    begin
        d   := to_integer(unsigned(difficulty));
        raw := BASE_SPAWN - d * 20;
        if raw < MIN_SPAWN then spawn_interval <= MIN_SPAWN;
        else                     spawn_interval <= raw;
        end if;
    end process;

    -- Obstacle advance divisor and step: frames per advance tick, units per tick.
    -- Schedule is derived from the constraint: travel_time < spawn_interval.
    -- With OBS_SPAWN_DEPTH=80 and ship depth=129, depth_to_travel=49 units.
    --   adv_step=2, adv_div=1 : travel=25 fr  (needed for d>=15 where spawn<=60)
    --   adv_step=1, adv_div=1 : travel=49 fr  (safe for d=9..14 where spawn=80..180)
    --   adv_step=1, adv_div=2 : travel=98 fr  (safe for d=6..8  where spawn=200..240)
    --   adv_step=1, adv_div=3 : travel=147 fr (safe for d=3..5  where spawn=260..300)
    --   adv_step=1, adv_div=4 : travel=196 fr (safe for d=0..2  where spawn=320..360)
    process(difficulty)
        variable d : integer;
    begin
        d := to_integer(unsigned(difficulty));
        if    d >= 15 then adv_div <= 1; adv_step <= to_unsigned(2, 8);
        elsif d >= 9  then adv_div <= 1; adv_step <= to_unsigned(1, 8);
        elsif d >= 6  then adv_div <= 2; adv_step <= to_unsigned(1, 8);
        elsif d >= 3  then adv_div <= 3; adv_step <= to_unsigned(1, 8);
        else               adv_div <= 4; adv_step <= to_unsigned(1, 8);
        end if;
    end process;

    process(clk_50)
        variable next_lfsr     : std_logic_vector(7 downto 0);
        variable hole_face_v   : unsigned(2 downto 0);
        variable player_face_v : unsigned(2 downto 0);
        variable n_obs_v       : integer range 0 to 7;
        variable do_advance    : boolean;
        variable do_spawn      : boolean;
        variable filled_v      : integer range 0 to 8;
        variable face_i        : integer range 0 to 15;
        variable coin_face_v   : integer range 0 to 7;
        variable coin_placed_v : boolean;
    begin
        if rising_edge(clk_50) then
            collision_r <= '0';
            ring_coll_r <= '0';
            wave_tick_r <= '0';

            -- LFSR advances every clock for best randomness
            next_lfsr := lfsr(6 downto 0) &
                         (lfsr(7) xor lfsr(5) xor lfsr(4) xor lfsr(3));
            lfsr <= next_lfsr;

            if reset_game = '1' then
                obs_active_r  <= (others => '0');
                obs_fsec_r    <= (others => "000");
                coin_active_r <= '0';
                spawn_cnt     <= 0;
                adv_cnt       <= 0;

            elsif frame_tick = '1' and game_active = '1' then

                -- Advance timer: fires once every adv_div frames
                do_advance := false;
                if adv_cnt = adv_div - 1 then
                    adv_cnt    <= 0;
                    do_advance := true;
                else
                    adv_cnt <= adv_cnt + 1;
                end if;

                -- Spawn timer: fires once every spawn_interval frames
                do_spawn := false;
                if spawn_cnt = spawn_interval - 1 then
                    spawn_cnt <= 0;
                    do_spawn  := true;
                else
                    spawn_cnt <= spawn_cnt + 1;
                end if;

                -- Advance active obstacles; retire past OBS_COLLIDE_DEPTH
                for i in 0 to 7 loop
                    if obs_active_r(i) = '1' then
                        if obs_depth_r(i) >= OBS_COLLIDE_DEPTH then
                            obs_active_r(i) <= '0';
                        elsif do_advance then
                            obs_depth_r(i) <= obs_depth_r(i) + adv_step;
                        end if;
                    end if;
                end loop;

                -- Advance coin; retire past OBS_COLLIDE_DEPTH
                if coin_active_r = '1' then
                    if coin_depth_r >= OBS_COLLIDE_DEPTH then
                        coin_active_r <= '0';
                    elsif do_advance then
                        coin_depth_r <= coin_depth_r + adv_step;
                    end if;
                end if;

                -- Spawn a new wave of obstacles + one coin
                if do_spawn then
                    wave_tick_r   <= '1';
                    player_face_v := unsigned(ship_rel_angle(15 downto 13));

                    -- Choose a random hole face; avoid the player's current face
                    hole_face_v := unsigned(lfsr(2 downto 0));
                    if hole_face_v = player_face_v then
                        hole_face_v := player_face_v + 1;
                    end if;

                    -- Random obstacle count 2..7
                    case to_integer(unsigned(next_lfsr(2 downto 0))) is
                        when 0      => n_obs_v := 2;
                        when 1      => n_obs_v := 3;
                        when 2      => n_obs_v := 4;
                        when 3      => n_obs_v := 5;
                        when 4      => n_obs_v := 6;
                        when 5      => n_obs_v := 7;
                        when 6      => n_obs_v := 2;
                        when others => n_obs_v := 3;
                    end case;

                    -- Clear all obstacle slots before filling
                    for s in 0 to 7 loop
                        obs_active_r(s) <= '0';
                    end loop;

                    -- Fill n_obs_v slots; first free non-hole face gets the coin
                    filled_v      := 0;
                    coin_placed_v := false;
                    coin_face_v   := 0;

                    for i in 0 to 7 loop
                        face_i := to_integer(player_face_v) + i;
                        if face_i >= 8 then face_i := face_i - 8; end if;

                        if face_i /= to_integer(hole_face_v) then
                            if filled_v < n_obs_v then
                                obs_active_r(filled_v) <= '1';
                                obs_depth_r(filled_v)  <= to_unsigned(OBS_SPAWN_DEPTH, 8);
                                obs_fsec_r(filled_v)   <= std_logic_vector(to_unsigned(face_i, 3));
                                filled_v := filled_v + 1;
                            elsif not coin_placed_v then
                                coin_face_v   := face_i;
                                coin_placed_v := true;
                            end if;
                        end if;
                    end loop;

                    -- Place coin; if n_obs=7 (no free face), put it in the hole
                    coin_active_r <= '1';
                    coin_depth_r  <= to_unsigned(OBS_SPAWN_DEPTH, 8);
                    if coin_placed_v then
                        coin_fsec_r <= std_logic_vector(to_unsigned(coin_face_v, 3));
                    else
                        coin_fsec_r <= std_logic_vector(hole_face_v);
                    end if;

                end if;

            end if;
        end if;
    end process;

end rtl;
