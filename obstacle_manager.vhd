library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- obstacle_manager.vhd
-- ================================================================
-- Manages up to 4 simultaneous obstacles AND up to 2 scoring rings.
--
-- ── OBSTACLES ────────────────────────────────────────────────────
-- GAP ANGLE SYSTEM:
--   gap_angle is a 16-bit value (0-65535 = 0-360°) in the TUNNEL
--   frame.  Derived from the top 3 bits of the LFSR so centres fall
--   exactly on one of the 8 octagonal face centres (every 45°).
--   gap_angle = lfsr[2:0] * 8192  (lfsr top-3 << 13)
--
-- COLLISION CHECK (at OBS_COLLIDE_DEPTH, per frame_tick):
--   diff = signed(ship_rel_angle - gap_angle)   [16-bit wrapping]
--   safe  when |diff| <= HALF_GAP  (ship is in the open sector)
--   hit   when |diff| >  HALF_GAP  → collision='1' for one cycle
--
-- HALF_GAP = 4096  (22.5° → one octagonal face-width of safety)
--
-- VISUAL:
--   obs*_gap_sector = obs*_gap_angle[15:13]  (top 3 bits → 0..7)
--   Passed to tunnel_renderer to suppress the ring in that wedge.
--
-- ── SCORING RINGS ────────────────────────────────────────────────
-- Full rings (no gap) that the ship always collects on contact.
-- They spawn at OBS_SPAWN_DEPTH and advance at the same rate as
-- obstacles (do_advance flag).  Separate spawn timer fires every
-- RING_SPAWN_INTERVAL frames.
--
-- COLLECTION CHECK (at OBS_COLLIDE_DEPTH, per frame_tick):
--   Ring deactivates and ring_collected pulses '1' for one cycle.
--   game_fsm counts these pulses toward the zone-unlock threshold.
--
-- VISUAL:
--   ring*_depth and ring*_active are passed to tunnel_renderer,
--   which renders them with a full gold band (no gap suppression).
-- ================================================================

entity obstacle_manager is
    port (
        clk_50          : in  std_logic;
        frame_tick      : in  std_logic;
        game_active     : in  std_logic;
        reset_game      : in  std_logic;
        ship_rel_angle  : in  std_logic_vector(15 downto 0);
        difficulty      : in  std_logic_vector(3 downto 0);
        num_faces       : in  std_logic_vector(3 downto 0);   -- 3..8, for gap scaling
        -- Obstacle outputs (4 slots)
        -- face_sector = which tunnel face the block sits ON (0-7 octant)
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
        collision        : out std_logic;   -- DISABLED (always '0') for visual testing
        -- Scoring ring outputs (2 slots, no gap)
        ring0_depth     : out std_logic_vector(7 downto 0);
        ring0_active    : out std_logic;
        ring1_depth     : out std_logic_vector(7 downto 0);
        ring1_active    : out std_logic;
        ring_collected  : out std_logic   -- 1-cycle pulse per ring collected
    );
end obstacle_manager;

architecture rtl of obstacle_manager is

    constant OBS_SPAWN_DEPTH    : integer := 75;
    constant OBS_COLLIDE_DEPTH  : integer := 128;
    constant BASE_SPAWN         : integer := 180;
    constant MIN_SPAWN          : integer := 20;
    constant RING_SPAWN_INTERVAL: integer := 180;   -- frames between scoring ring spawns
    -- HALF_GAP is now per-N (see process below); no longer a single constant

    -- ---- Obstacle state ----
    type depth_arr_t is array (0 to 3) of unsigned(7 downto 0);
    type gap_arr_t   is array (0 to 3) of unsigned(15 downto 0);
    type act_arr_t   is array (0 to 3) of std_logic;

    signal obs_depth_r  : depth_arr_t := (others => to_unsigned(OBS_SPAWN_DEPTH, 8));
    signal obs_face_r   : gap_arr_t   := (others => (others => '0'));
    signal obs_active_r : act_arr_t   := (others => '0');

    -- ---- Scoring ring state ----
    type ring_depth_arr_t is array (0 to 1) of unsigned(7 downto 0);
    type ring_act_arr_t   is array (0 to 1) of std_logic;

    signal ring_depth_r  : ring_depth_arr_t := (others => to_unsigned(OBS_SPAWN_DEPTH, 8));
    signal ring_active_r : ring_act_arr_t   := (others => '0');
    signal ring_spawn_cnt : integer range 0 to RING_SPAWN_INTERVAL - 1 := 0;
    signal ring_coll_r   : std_logic := '0';

    -- ---- Shared timing / LFSR ----
    signal spawn_cnt : integer range 0 to BASE_SPAWN - 1 := 0;
    signal adv_cnt   : integer range 0 to 3 := 0;
    signal lfsr      : std_logic_vector(7 downto 0) := "10110101";
    signal collision_r : std_logic := '0';

    -- Combinational helpers
    signal spawn_interval : integer range MIN_SPAWN to BASE_SPAWN;
    signal adv_div        : integer range 1 to 4;
    -- Per-polygon half-gap: 65536 / (2*N)  = half a face width in 16-bit angle units
    --   N=3 → 10923 (60°)   N=4 → 8192 (45°)   N=5 → 6554 (36°)
    --   N=6 → 5461 (30°)    N=7 → 4681 (≈25.7°) N=8 → 4096 (22.5°)
    signal half_gap_s     : integer range 4096 to 10923;

begin

    -- ----------------------------------------------------------------
    -- Output assignments
    -- ----------------------------------------------------------------
    obs0_depth       <= std_logic_vector(obs_depth_r(0));
    obs0_face_sector <= std_logic_vector(obs_face_r(0)(15 downto 13));
    obs0_active      <= obs_active_r(0);
    obs1_depth       <= std_logic_vector(obs_depth_r(1));
    obs1_face_sector <= std_logic_vector(obs_face_r(1)(15 downto 13));
    obs1_active      <= obs_active_r(1);
    obs2_depth       <= std_logic_vector(obs_depth_r(2));
    obs2_face_sector <= std_logic_vector(obs_face_r(2)(15 downto 13));
    obs2_active      <= obs_active_r(2);
    obs3_depth       <= std_logic_vector(obs_depth_r(3));
    obs3_face_sector <= std_logic_vector(obs_face_r(3)(15 downto 13));
    obs3_active      <= obs_active_r(3);
    collision        <= '0';   -- disabled for visual testing; re-enable later

    ring0_depth  <= std_logic_vector(ring_depth_r(0));
    ring0_active <= ring_active_r(0);
    ring1_depth  <= std_logic_vector(ring_depth_r(1));
    ring1_active <= ring_active_r(1);
    ring_collected <= ring_coll_r;

    -- ----------------------------------------------------------------
    -- Spawn interval: BASE_SPAWN - difficulty*8, floored at MIN_SPAWN
    -- ----------------------------------------------------------------
    process(difficulty)
        variable d, raw : integer;
    begin
        d   := to_integer(unsigned(difficulty));
        raw := BASE_SPAWN - d * 8;
        if raw < MIN_SPAWN then spawn_interval <= MIN_SPAWN;
        else                     spawn_interval <= raw;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Advance divisor: 4 at difficulty 0, down to 1 at difficulty 12+
    -- ----------------------------------------------------------------
    process(difficulty)
        variable d, raw : integer;
    begin
        d   := to_integer(unsigned(difficulty));
        raw := 4 - d / 4;
        if raw < 1 then adv_div <= 1;
        else             adv_div <= raw;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Half-gap size: scales with polygon face width (65536 / 2N)
    -- ----------------------------------------------------------------
    process(num_faces)
    begin
        case to_integer(unsigned(num_faces)) is
            when 3      => half_gap_s <= 10923;  -- 60°  : half of 120° triangle face
            when 4      => half_gap_s <= 8192;   -- 45°  : half of  90° square face
            when 5      => half_gap_s <= 6554;   -- 36°  : half of  72° pentagon face
            when 6      => half_gap_s <= 5461;   -- 30°  : half of  60° hexagon face
            when 7      => half_gap_s <= 4681;   -- ~25.7°: half of ~51° heptagon face
            when others => half_gap_s <= 4096;   -- 22.5°: half of  45° octagon face (N=8)
        end case;
    end process;

    -- ----------------------------------------------------------------
    -- Main update process
    -- ----------------------------------------------------------------
    process(clk_50)
        variable next_lfsr   : std_logic_vector(7 downto 0);
        variable gap_sec     : unsigned(2 downto 0);
        variable gap_angle   : unsigned(15 downto 0);
        variable diff        : signed(15 downto 0);
        variable found_obs   : boolean;
        variable found_ring  : boolean;
        variable do_advance  : boolean;
        variable do_spawn    : boolean;
        variable do_rspawn   : boolean;
    begin
        if rising_edge(clk_50) then
            collision_r  <= '0';
            ring_coll_r  <= '0';

            -- LFSR ticks every clock (obstacle gap randomisation)
            next_lfsr := lfsr(6 downto 0) & (lfsr(7) xor lfsr(5) xor lfsr(4) xor lfsr(3));

            if reset_game = '1' then
                obs_active_r  <= (others => '0');
                ring_active_r <= (others => '0');
                spawn_cnt     <= 0;
                adv_cnt       <= 0;
                ring_spawn_cnt <= 0;

            elsif frame_tick = '1' then

                -- ── Advance timer ─────────────────────────────────
                do_advance := false;
                if adv_cnt = adv_div - 1 then
                    adv_cnt    <= 0;
                    do_advance := true;
                else
                    adv_cnt <= adv_cnt + 1;
                end if;

                -- ── Obstacle spawn timer ───────────────────────────
                do_spawn := false;
                if spawn_cnt = spawn_interval - 1 then
                    spawn_cnt <= 0;
                    do_spawn  := true;
                else
                    spawn_cnt <= spawn_cnt + 1;
                end if;

                -- ── Scoring ring spawn timer ───────────────────────
                do_rspawn := false;
                if ring_spawn_cnt = RING_SPAWN_INTERVAL - 1 then
                    ring_spawn_cnt <= 0;
                    do_rspawn      := true;
                else
                    ring_spawn_cnt <= ring_spawn_cnt + 1;
                end if;

                -- ── Advance obstacles; collision check at depth ────
                for i in 0 to 3 loop
                    if obs_active_r(i) = '1' then
                        if obs_depth_r(i) >= OBS_COLLIDE_DEPTH then
                            diff := signed(unsigned(ship_rel_angle) - obs_face_r(i));
                            if diff > half_gap_s or diff < -half_gap_s then
                                collision_r <= '1';
                            end if;
                            obs_active_r(i) <= '0';
                        elsif do_advance then
                            obs_depth_r(i) <= obs_depth_r(i) + 1;
                        end if;
                    end if;
                end loop;

                -- ── Advance scoring rings; collect at depth ────────
                for i in 0 to 1 loop
                    if ring_active_r(i) = '1' then
                        if ring_depth_r(i) >= OBS_COLLIDE_DEPTH then
                            -- Always collected (full ring, no gap to miss)
                            ring_coll_r      <= '1';
                            ring_active_r(i) <= '0';
                        elsif do_advance then
                            ring_depth_r(i) <= ring_depth_r(i) + 1;
                        end if;
                    end if;
                end loop;

                -- ── Spawn obstacle into first free slot ───────────
                if do_spawn then
                    lfsr <= next_lfsr;
                    gap_sec   := unsigned(next_lfsr(2 downto 0));
                    -- face_angle: one of 8 octant centres (0°,45°,…,315°)
                    -- top 3 bits select sector; the block sits ON this face
                    gap_angle := gap_sec & "0000000000000";   -- face_sec × 8192
                    found_obs := false;
                    for i in 0 to 3 loop
                        if obs_active_r(i) = '0' and not found_obs then
                            obs_active_r(i) <= '1';
                            obs_depth_r(i)  <= to_unsigned(OBS_SPAWN_DEPTH, 8);
                            obs_face_r(i)   <= gap_angle;
                            found_obs       := true;
                        end if;
                    end loop;
                end if;

                -- ── Spawn scoring ring into first free slot ────────
                if do_rspawn then
                    found_ring := false;
                    for i in 0 to 1 loop
                        if ring_active_r(i) = '0' and not found_ring then
                            ring_active_r(i) <= '1';
                            ring_depth_r(i)  <= to_unsigned(OBS_SPAWN_DEPTH, 8);
                            found_ring       := true;
                        end if;
                    end loop;
                end if;

            end if;
        end if;
    end process;

end rtl;
