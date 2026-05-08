library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- obstacle_manager.vhd
-- ================================================================
-- Manages 8 obstacle slots + 1 scoring coin per wave.
--
-- ── WAVE SPAWN SYSTEM ────────────────────────────────────────────
-- Each spawn event creates one "wave" of 2..7 simultaneous obstacles
-- all at the same depth (OBS_SPAWN_DEPTH).  A "hole" face is chosen
-- at random — that face is always left open as the safe passage.
-- One coin is placed on the FIRST free (non-obstacle, non-hole) face
-- in the iteration order starting from player_face+1.
-- Special case n_obs=7: no free non-hole face exists; coin is placed
-- on the hole face — the player must collect it to survive anyway.
--
--   hole_face : random 0..7  (LFSR bits 2:0, tick 1)
--   n_obs     : random 2..7  (LFSR bits 2:0, tick 2 → mapped 0-5 → 2-7)
--
-- ── COIN COLLECTION ──────────────────────────────────────────────
-- Same depth/timing as the obstacles.  At OBS_COLLIDE_DEPTH the coin
-- checks whether |ship_rel_angle − coin_face_angle| ≤ HALF_GAP.
-- If yes, ring_collected pulses for one clock.
--
-- ── COLLISION CHECK ──────────────────────────────────────────────
-- At OBS_COLLIDE_DEPTH (per frame_tick):
--   diff = signed(ship_rel_angle - obs_face_angle)   [16-bit wrapping]
--   hit  when |diff| ≤ HALF_GAP (ship IS on the blocked face)
--   HALF_GAP = 4096  (22.5° — one octagon face half-width)
--
-- ── SPEED PROGRESSION ────────────────────────────────────────────
--   difficulty 0  : adv_div=6, spawn every 360 frames  (5.0 s @ 72 Hz)
--   difficulty 3  : adv_div=5, spawn every 300 frames  (4.2 s)
--   difficulty 6  : adv_div=4, spawn every 240 frames  (3.3 s)
--   difficulty 9  : adv_div=3, spawn every 180 frames  (2.5 s)
--   difficulty 12 : adv_div=2, spawn every 120 frames  (1.7 s)
--   difficulty 15+: adv_div=1, spawn every  72 frames  (1.0 s, MIN_SPAWN cap)
--   travel_time at each level: 318/265/212/159/106/65 frames
--   (difficulty port is 5-bit, range 0-20; ship speed is fixed in spaceship.vhd)
-- ================================================================

entity obstacle_manager is
    port (
        clk_50          : in  std_logic;
        frame_tick      : in  std_logic;
        game_active     : in  std_logic;
        reset_game      : in  std_logic;
        ship_rel_angle  : in  std_logic_vector(15 downto 0);
        difficulty      : in  std_logic_vector(4 downto 0);
        num_faces       : in  std_logic_vector(3 downto 0);
        -- Obstacle outputs (8 slots)
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
        -- Scoring coin (ring0 repurposed; ring1 disabled)
        ring0_depth       : out std_logic_vector(7 downto 0);
        ring0_active      : out std_logic;
        ring0_face_sector : out std_logic_vector(2 downto 0);
        ring1_depth       : out std_logic_vector(7 downto 0);
        ring1_active      : out std_logic;
        ring_collected    : out std_logic
    );
end obstacle_manager;

architecture rtl of obstacle_manager is

    constant OBS_SPAWN_DEPTH   : integer := 75;
    -- Raised to 140 so obstacles travel past the ship orbit (depth_band ~129-132)
    -- before deactivating.  Collision is now pixel-based (from tunnel.vhd).
    constant OBS_COLLIDE_DEPTH : integer := 140;
    constant BASE_SPAWN        : integer := 360;
    constant MIN_SPAWN         : integer := 72;   -- 1 second at 72 Hz = max speed

    -- ---- 8-slot obstacle state ----
    type depth_arr_t is array (0 to 7) of unsigned(7 downto 0);
    type fsec_arr_t  is array (0 to 7) of std_logic_vector(2 downto 0);
    type act_arr_t   is array (0 to 7) of std_logic;

    signal obs_depth_r  : depth_arr_t := (others => to_unsigned(OBS_SPAWN_DEPTH, 8));
    signal obs_fsec_r   : fsec_arr_t  := (others => "000");
    signal obs_active_r : act_arr_t   := (others => '0');

    -- ---- Scoring coin state ----
    signal coin_depth_r  : unsigned(7 downto 0)        := to_unsigned(OBS_SPAWN_DEPTH, 8);
    signal coin_fsec_r   : std_logic_vector(2 downto 0) := "000";
    signal coin_active_r : std_logic                    := '0';
    signal ring_coll_r   : std_logic                    := '0';

    -- ---- Shared timing / LFSR ----
    signal spawn_cnt   : integer range 0 to BASE_SPAWN - 1 := 0;
    signal adv_cnt     : integer range 0 to 5              := 0;
    signal lfsr        : std_logic_vector(7 downto 0)      := "10110101";
    signal collision_r : std_logic := '0';

    signal spawn_interval : integer range MIN_SPAWN to BASE_SPAWN;
    signal adv_div        : integer range 1 to 6;

begin

    -- ---- Output assignments ------------------------------------------------
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

    -- Scoring coin outputs
    ring0_depth       <= std_logic_vector(coin_depth_r);
    ring0_active      <= coin_active_r;
    ring0_face_sector <= coin_fsec_r;
    ring_collected    <= ring_coll_r;
    -- ring1 disabled
    ring1_depth <= (others => '0');  ring1_active <= '0';

    -- ---- Spawn interval ----
    process(difficulty)
        variable d, raw : integer;
    begin
        d   := to_integer(unsigned(difficulty));
        raw := BASE_SPAWN - d * 20;
        if raw < MIN_SPAWN then spawn_interval <= MIN_SPAWN;
        else                     spawn_interval <= raw;
        end if;
    end process;

    -- ---- Advance divisor ----
    process(difficulty)
        variable d : integer;
    begin
        d := to_integer(unsigned(difficulty));
        if    d >= 15 then adv_div <= 1;
        elsif d >= 12 then adv_div <= 2;
        elsif d >= 9  then adv_div <= 3;
        elsif d >= 6  then adv_div <= 4;
        elsif d >= 3  then adv_div <= 5;
        else               adv_div <= 6;
        end if;
    end process;

    -- ---- Main update process ------------------------------------------------
    process(clk_50)
        variable next_lfsr     : std_logic_vector(7 downto 0);
        variable hole_face_v   : unsigned(2 downto 0);
        variable player_face_v : unsigned(2 downto 0);
        variable n_obs_v       : integer range 0 to 7;
        variable do_advance    : boolean;
        variable do_spawn      : boolean;
        variable filled_v      : integer range 0 to 7;
        variable face_i        : integer range 0 to 15;
        variable coin_face_v   : integer range 0 to 7;
        variable coin_placed_v : boolean;
    begin
        if rising_edge(clk_50) then
            collision_r <= '0';
            ring_coll_r <= '0';

            -- LFSR advances every clock for best randomness
            next_lfsr := lfsr(6 downto 0) &
                         (lfsr(7) xor lfsr(5) xor lfsr(4) xor lfsr(3));
            lfsr <= next_lfsr;

            if reset_game = '1' then
                obs_active_r <= (others => '0');
                obs_fsec_r   <= (others => "000");
                coin_active_r <= '0';
                spawn_cnt    <= 0;
                adv_cnt      <= 0;

            elsif frame_tick = '1' and game_active = '1' then

                -- ── Advance timer ──────────────────────────────────────
                do_advance := false;
                if adv_cnt = adv_div - 1 then
                    adv_cnt    <= 0;
                    do_advance := true;
                else
                    adv_cnt <= adv_cnt + 1;
                end if;

                -- ── Spawn timer ────────────────────────────────────────
                do_spawn := false;
                if spawn_cnt = spawn_interval - 1 then
                    spawn_cnt <= 0;
                    do_spawn  := true;
                else
                    spawn_cnt <= spawn_cnt + 1;
                end if;

                -- ── Advance obstacles; deactivate when past ship orbit ─
                -- Collision is now pixel-based in tunnel.vhd.
                for i in 0 to 7 loop
                    if obs_active_r(i) = '1' then
                        if obs_depth_r(i) >= OBS_COLLIDE_DEPTH then
                            obs_active_r(i) <= '0';
                        elsif do_advance then
                            obs_depth_r(i) <= obs_depth_r(i) + 1;
                        end if;
                    end if;
                end loop;

                -- ── Advance coin; deactivate when past ship orbit ──────
                if coin_active_r = '1' then
                    if coin_depth_r >= OBS_COLLIDE_DEPTH then
                        coin_active_r <= '0';
                    elsif do_advance then
                        coin_depth_r <= coin_depth_r + 1;
                    end if;
                end if;

                -- ── Spawn a new wave ───────────────────────────────────
                if do_spawn then
                    player_face_v := unsigned(ship_rel_angle(15 downto 13));

                    hole_face_v := unsigned(lfsr(2 downto 0));
                    if hole_face_v = player_face_v then
                        hole_face_v := player_face_v + 1;
                    end if;

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

                    -- Clear all 8 obstacle slots
                    for s in 0 to 7 loop
                        obs_active_r(s) <= '0';
                    end loop;

                    -- Fill obstacle slots and find coin face in one pass.
                    -- Iterate from player_face_v (always blocked first),
                    -- then each subsequent face in order.
                    -- The first non-hole face AFTER n_obs_v obstacles have been
                    -- placed becomes the coin face.
                    filled_v      := 0;
                    coin_placed_v := false;
                    coin_face_v   := 0;

                    for i in 0 to 7 loop
                        face_i := to_integer(player_face_v) + i;
                        if face_i >= 8 then face_i := face_i - 8; end if;

                        if face_i /= to_integer(hole_face_v) then
                            if filled_v < n_obs_v then
                                -- Place obstacle
                                obs_active_r(filled_v) <= '1';
                                obs_depth_r(filled_v)  <= to_unsigned(OBS_SPAWN_DEPTH, 8);
                                obs_fsec_r(filled_v)   <= std_logic_vector(to_unsigned(face_i, 3));
                                filled_v := filled_v + 1;
                            elsif not coin_placed_v then
                                -- First free non-hole face → coin
                                coin_face_v   := face_i;
                                coin_placed_v := true;
                            end if;
                        end if;
                    end loop;

                    -- Spawn the coin.
                    -- If coin_placed_v is false (n_obs=7, only hole is free),
                    -- put the coin in the hole — player must collect to survive.
                    coin_active_r <= '1';
                    coin_depth_r  <= to_unsigned(OBS_SPAWN_DEPTH, 8);
                    if coin_placed_v then
                        coin_fsec_r <= std_logic_vector(to_unsigned(coin_face_v, 3));
                    else
                        -- n_obs = 7: coin lives in the hole face
                        coin_fsec_r <= std_logic_vector(hole_face_v);
                    end if;

                end if;  -- do_spawn

            end if;  -- frame_tick
        end if;  -- rising_edge
    end process;

end rtl;
