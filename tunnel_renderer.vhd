library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- tunnel_renderer.vhd  —  Thin orchestrator
-- ================================================================
-- Wires the three focused sub-modules and handles the obstacle /
-- scoring-ring depth comparisons that require both log_band and
-- pix_sec to be available simultaneously.
--
-- Sub-modules instantiated:
--   u_xfm  : coord_transform    — Stage-0: rotation, ship hit detect
--   u_core : tunnel_core        — Stage-1 combinational: N-gon metric,
--                                  ring/spoke/dead-zone flags
--   u_bg   : tunnel_background  — Screen-space background pattern
--   u_pal  : colour_palette     — Stage-1 register: flags → RGB
--
-- PIPELINE:
--   Stage 0 register  — coord_transform  (dx_rot, dy_rot, on_ship, vid)
--   (combinational)   — tunnel_core, tunnel_background, obstacle checks
--   Stage 1 register  — colour_palette   (red, green, blue)
--
-- OBSTACLE / RING DEPTH CHECKS:
--   A pixel is "on obstacle i" when:
--     • obs_active = '1'
--     • |log_band − obs_depth| < OBS_THICK    (correct depth band)
--     • pix_sec = obs_face_sector              (correct tunnel face)
--
--   A pixel is "on score ring" when:
--     • ring_active = '1'
--     • |log_band − ring_depth| < OBS_THICK   (full ring, any face)
--
-- TUNING:
--   OBS_THICK [6] : half-width of the obstacle depth band in log_band units.
--                   INCREASE for thicker obstacle blocks.
-- ================================================================

entity tunnel_renderer is
    port (
        clk_50            : in  std_logic;
        video_on          : in  std_logic;
        hpos              : in  std_logic_vector(9 downto 0);
        vpos              : in  std_logic_vector(9 downto 0);
        scroll            : in  std_logic_vector(7 downto 0);
        hue_cnt           : in  unsigned(9 downto 0);
        num_faces         : in  std_logic_vector(3 downto 0);
        transition_active : in  std_logic;
        -- Tunnel rotation (from trig_rom, tunnel angle)
        cos_t             : in  signed(8 downto 0);
        sin_t             : in  signed(8 downto 0);
        -- Ship sprite position (registered, updated each clock)
        ship_x            : in  unsigned(9 downto 0);
        ship_y            : in  unsigned(9 downto 0);
        -- Game state
        game_state        : in  std_logic_vector(1 downto 0);
        -- Obstacles (4 slots)
        obs0_depth        : in  std_logic_vector(7 downto 0);
        obs0_face_sector  : in  std_logic_vector(2 downto 0);
        obs0_active       : in  std_logic;
        obs1_depth        : in  std_logic_vector(7 downto 0);
        obs1_face_sector  : in  std_logic_vector(2 downto 0);
        obs1_active       : in  std_logic;
        obs2_depth        : in  std_logic_vector(7 downto 0);
        obs2_face_sector  : in  std_logic_vector(2 downto 0);
        obs2_active       : in  std_logic;
        obs3_depth        : in  std_logic_vector(7 downto 0);
        obs3_face_sector  : in  std_logic_vector(2 downto 0);
        obs3_active       : in  std_logic;
        -- Scoring rings (2 slots, full ring / no face sector)
        ring0_depth       : in  std_logic_vector(7 downto 0);
        ring0_active      : in  std_logic;
        ring1_depth       : in  std_logic_vector(7 downto 0);
        ring1_active      : in  std_logic;
        -- Output
        red               : out std_logic_vector(3 downto 0);
        green             : out std_logic_vector(3 downto 0);
        blue              : out std_logic_vector(3 downto 0)
    );
end tunnel_renderer;

architecture rtl of tunnel_renderer is

    -- Obstacle depth-band half-width (in log_band units, 8-bit scale)
    constant OBS_THICK : integer := 6;

    -- ── Stage-0 outputs (registered inside coord_transform) ──────
    signal dx_rot   : signed(11 downto 0);
    signal dy_rot   : signed(11 downto 0);
    signal on_ship  : std_logic;
    signal vid_s0   : std_logic;   -- video_on delayed by one stage

    -- ── tunnel_core outputs (combinational) ──────────────────────
    signal is_ring_s     : std_logic;
    signal is_spoke_s    : std_logic;
    signal in_dead_zone_s: std_logic;
    signal log_band_s    : unsigned(7 downto 0);
    signal depth_shade_s : unsigned(3 downto 0);
    signal pix_sec_s     : std_logic_vector(2 downto 0);

    -- ── Background ───────────────────────────────────────────────
    signal bg_active_s   : std_logic;
    -- bg_scroll: slow screen-space horizontal counter.
    -- CHANGE: wire to a top-level counter if you want the background
    -- to scroll at a different rate.  Currently static (all zeros).
    signal bg_scroll_s   : std_logic_vector(9 downto 0) := (others => '0');

    -- ── Obstacle depth comparisons (combinational) ────────────────
    signal obs0_diff, obs1_diff, obs2_diff, obs3_diff : signed(8 downto 0);
    signal on_obs0, on_obs1, on_obs2, on_obs3         : std_logic;
    signal on_obstacle_s                               : std_logic;

    -- ── Scoring ring depth comparisons ───────────────────────────
    signal ring0_diff, ring1_diff : signed(8 downto 0);
    signal on_ring0, on_ring1     : std_logic;
    signal on_score_ring_s        : std_logic;

begin

    -- ================================================================
    -- u_xfm : coord_transform — Stage-0 register
    -- Rotates pixel (hpos, vpos) by (cos_t, sin_t) into tunnel frame.
    -- Also detects whether the pixel is on the ship diamond sprite.
    -- ================================================================
    u_xfm : entity work.coord_transform
        port map (
            clk_50   => clk_50,
            hpos     => hpos,
            vpos     => vpos,
            cos_t    => cos_t,
            sin_t    => sin_t,
            ship_x   => ship_x,
            ship_y   => ship_y,
            video_on => video_on,
            dx_rot   => dx_rot,
            dy_rot   => dy_rot,
            on_ship  => on_ship,
            vid_out  => vid_s0
        );

    -- ================================================================
    -- u_core : tunnel_core — N-gon metric (combinational)
    -- ================================================================
    u_core : entity work.tunnel_core
        port map (
            dx_rot       => dx_rot,
            dy_rot       => dy_rot,
            num_faces    => num_faces,
            scroll       => scroll,
            is_ring      => is_ring_s,
            is_spoke     => is_spoke_s,
            in_dead_zone => in_dead_zone_s,
            log_band     => log_band_s,
            depth_shade  => depth_shade_s,
            pix_sec      => pix_sec_s
        );

    -- ================================================================
    -- u_bg : tunnel_background — screen-space background pattern
    -- bg_scroll wired to static '0' for now; hook up a top-level
    -- counter to animate it.
    -- ================================================================
    u_bg : entity work.tunnel_background
        port map (
            hpos      => hpos,
            vpos      => vpos,
            bg_scroll => bg_scroll_s,
            bg_active => bg_active_s
        );

    -- ================================================================
    -- Obstacle depth comparisons
    -- ------------------------------------------------------------------
    -- Each obstacle lives at a specific depth (log_band value) and on
    -- a specific face (pix_sec = face_sector).
    -- The signed difference tells us how far (in log units) this pixel
    -- is from the obstacle's depth plane.
    -- ================================================================
    obs0_diff <= signed('0' & log_band_s) - signed('0' & unsigned(obs0_depth));
    obs1_diff <= signed('0' & log_band_s) - signed('0' & unsigned(obs1_depth));
    obs2_diff <= signed('0' & log_band_s) - signed('0' & unsigned(obs2_depth));
    obs3_diff <= signed('0' & log_band_s) - signed('0' & unsigned(obs3_depth));

    on_obs0 <= '1' when obs0_active = '1'
                        and obs0_diff > -OBS_THICK and obs0_diff < OBS_THICK
                        and pix_sec_s = obs0_face_sector
               else '0';
    on_obs1 <= '1' when obs1_active = '1'
                        and obs1_diff > -OBS_THICK and obs1_diff < OBS_THICK
                        and pix_sec_s = obs1_face_sector
               else '0';
    on_obs2 <= '1' when obs2_active = '1'
                        and obs2_diff > -OBS_THICK and obs2_diff < OBS_THICK
                        and pix_sec_s = obs2_face_sector
               else '0';
    on_obs3 <= '1' when obs3_active = '1'
                        and obs3_diff > -OBS_THICK and obs3_diff < OBS_THICK
                        and pix_sec_s = obs3_face_sector
               else '0';

    on_obstacle_s <= on_obs0 or on_obs1 or on_obs2 or on_obs3;

    -- ================================================================
    -- Scoring ring depth comparisons (full ring, no face sector check)
    -- ================================================================
    ring0_diff <= signed('0' & log_band_s) - signed('0' & unsigned(ring0_depth));
    ring1_diff <= signed('0' & log_band_s) - signed('0' & unsigned(ring1_depth));

    on_ring0 <= '1' when ring0_active = '1'
                         and ring0_diff > -OBS_THICK and ring0_diff < OBS_THICK
               else '0';
    on_ring1 <= '1' when ring1_active = '1'
                         and ring1_diff > -OBS_THICK and ring1_diff < OBS_THICK
               else '0';

    on_score_ring_s <= on_ring0 or on_ring1;

    -- ================================================================
    -- u_pal : colour_palette — Stage-1 register
    -- ================================================================
    u_pal : entity work.colour_palette
        port map (
            clk_50            => clk_50,
            is_ring           => is_ring_s,
            is_spoke          => is_spoke_s,
            in_dead_zone      => in_dead_zone_s,
            depth_shade       => depth_shade_s,
            on_bg             => bg_active_s,
            on_ship           => on_ship,
            on_obstacle       => on_obstacle_s,
            on_score_ring     => on_score_ring_s,
            vid_in            => vid_s0,
            game_state        => game_state,
            transition_active => transition_active,
            hue_cnt           => hue_cnt,
            red               => red,
            green             => green,
            blue              => blue
        );

end rtl;
