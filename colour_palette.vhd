library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- colour_palette.vhd
-- ================================================================
-- Single responsibility: map pixel classification flags → RGB output.
-- All colour decisions live here.  Nothing else assigns red/green/blue.
--
-- ──────────────────────────────────────────────────────────────
-- COLOUR SETTINGS — edit these to change the entire game's look
-- ──────────────────────────────────────────────────────────────
--
-- Tunnel line colour cycles through 4 keyframe colours.
-- Each colour is (Red, Green, Blue) in range 0..15 (4-bit VGA).
--
--   PALETTE TABLE — change keyframe values to change the colour scheme:
--
--   KF_R / KF_G / KF_B  (4 entries, index 0..3)
--
--   Default "Operius vaporwave":
--     0: neon pink   R=15  G= 2  B=12
--     1: magenta     R=15  G= 0  B=15
--     2: violet-blue R= 4  G= 0  B=15
--     3: cyan        R= 0  G=12  B=15
--
--   For monochrome green (like image 2):
--     set all KF_R=0, KF_G=15, KF_B=0
--
--   For Operius white/grey (like image 3):
--     set KF_R=15, KF_G=15, KF_B=15 (all same value, no cycling)
--
-- Fixed-colour overrides (ship, obstacles, score rings):
--   SHIP_R/G/B, OBS_R/G/B, RING_R/G/B   — also adjustable below.
-- ──────────────────────────────────────────────────────────────
--
-- PIPELINE:
--   Inputs  : combinational (from tunnel_core + renderer)
--   Output  : registered 1 clock later (Stage-1 register)
--
-- PRIORITY (highest wins):
--   1. vid_in='0' or transition_active='1' → black
--   2. game_state="10" (GAME OVER)         → solid red tint
--   3. on_ship  (and PLAYING)              → SHIP colour
--   4. on_obstacle                         → OBS colour
--   5. on_score_ring                       → RING colour
--   6. is_ring or is_spoke                 → palette colour × depth_shade
--   7. in_dead_zone                        → black
--   8. background                          → black (background module handles bg)
-- ================================================================

entity colour_palette is
    port (
        clk_50            : in  std_logic;
        -- Pixel flags (combinational, from tunnel_core + renderer)
        is_ring           : in  std_logic;
        is_spoke          : in  std_logic;
        in_dead_zone      : in  std_logic;
        depth_shade       : in  unsigned(3 downto 0);  -- 0=dark(void)..15=bright(edge)
        on_bg             : in  std_logic;              -- background pattern pixel (dims tunnel lines)
        on_ship           : in  std_logic;
        on_obstacle       : in  std_logic;
        on_score_ring     : in  std_logic;
        vid_in            : in  std_logic;             -- video_on delayed 1 stage
        -- Game control
        game_state        : in  std_logic_vector(1 downto 0);  -- "00"IDLE "01"PLAY "10"OVER
        transition_active : in  std_logic;
        -- Colour cycle counter (incremented by 1 each frame at top level)
        hue_cnt           : in  unsigned(9 downto 0);
        -- Registered RGB outputs
        red               : out std_logic_vector(3 downto 0);
        green             : out std_logic_vector(3 downto 0);
        blue              : out std_logic_vector(3 downto 0)
    );
end colour_palette;

architecture rtl of colour_palette is

    -- ──────────────────────────────────────────────────────────────
    -- ★ TUNNEL PALETTE  —  edit the 4 keyframe colours here ★
    -- ──────────────────────────────────────────────────────────────
    -- 5th entry = copy of entry 0 for seamless wrap interpolation.
    type pal_t is array(0 to 4) of integer range 0 to 15;

    --                     KF0   KF1   KF2   KF3   KF4(=KF0)
    constant KF_R : pal_t := (15,   15,    4,    0,   15);  -- red channel
    constant KF_G : pal_t := ( 2,    0,    0,   12,    2);  -- green channel
    constant KF_B : pal_t := (12,   15,   15,   15,   12);  -- blue channel

    -- ──────────────────────────────────────────────────────────────
    -- ★ FIXED OBJECT COLOURS  —  edit here ★
    -- ──────────────────────────────────────────────────────────────
    --                     R     G     B
    constant SHIP_R  : integer := 15;  -- ship: bright yellow
    constant SHIP_G  : integer := 14;
    constant SHIP_B  : integer :=  0;

    constant OBS_R   : integer := 15;  -- obstacle block: orange-red
    constant OBS_G   : integer :=  4;
    constant OBS_B   : integer :=  0;

    constant RING_R  : integer := 15;  -- scoring ring: gold
    constant RING_G  : integer := 12;
    constant RING_B  : integer :=  0;

    -- ──────────────────────────────────────────────────────────────
    -- Internal: interpolated palette colour
    -- ──────────────────────────────────────────────────────────────
    signal pal_r_s, pal_g_s, pal_b_s : unsigned(3 downto 0);

    -- Depth-shaded palette: pal × depth_shade / 16 (top nibble of 8-bit product)
    signal prod_r, prod_g, prod_b : unsigned(7 downto 0);
    signal sh_r,   sh_g,   sh_b   : unsigned(3 downto 0);

begin

    -- ================================================================
    -- Palette interpolation (combinational)
    -- ------------------------------------------------------------------
    -- hue_cnt[9:8] = phase index 0..3  (which keyframe pair)
    -- hue_cnt[7:4] = interpolation t   (0..15 steps within pair)
    -- Total cycle: 64 steps = ~0.9 s at 72 Hz
    -- CHANGE hue_cnt slices to speed/slow the colour cycling.
    -- ================================================================
    process(hue_cnt)
        variable ph : integer range 0 to 3;
        variable t  : integer range 0 to 15;
        variable ar, ag, ab, br, bg, bb : integer range 0 to 15;
    begin
        ph := to_integer(hue_cnt(9 downto 8));
        t  := to_integer(hue_cnt(7 downto 4));
        ar := KF_R(ph);      ag := KF_G(ph);      ab := KF_B(ph);
        br := KF_R(ph + 1);  bg := KF_G(ph + 1);  bb := KF_B(ph + 1);
        pal_r_s <= to_unsigned(ar + ((br - ar) * t) / 16, 4);
        pal_g_s <= to_unsigned(ag + ((bg - ag) * t) / 16, 4);
        pal_b_s <= to_unsigned(ab + ((bb - ab) * t) / 16, 4);
    end process;

    -- Depth shading: multiply palette colour by depth_shade then take top nibble.
    -- depth_shade=0 → black (void);  depth_shade=15 → 94% brightness (edge).
    -- CHANGE to "pal_r_s * to_unsigned(15, 4)" to disable depth shading (flat colour).
    prod_r <= pal_r_s * depth_shade;
    prod_g <= pal_g_s * depth_shade;
    prod_b <= pal_b_s * depth_shade;
    sh_r   <= prod_r(7 downto 4);
    sh_g   <= prod_g(7 downto 4);
    sh_b   <= prod_b(7 downto 4);

    -- ================================================================
    -- Stage-1 register: priority colour mux
    -- ================================================================
    process(clk_50)
    begin
        if rising_edge(clk_50) then

            -- ── Default: black (void / off-tunnel background) ────
            red   <= "0000";
            green <= "0000";
            blue  <= "0000";

            -- ── Tunnel lines: depth-shaded palette colour ────────
            -- is_ring and is_spoke both draw with the same colour.
            -- To give spokes a different colour, split into two ifs.
            -- When on_bg='1' the brightness is halved (background dim effect).
            if is_ring = '1' or is_spoke = '1' then
                if on_bg = '1' then
                    -- Halve channel values: "dim by background" effect
                    red   <= std_logic_vector('0' & sh_r(3 downto 1));
                    green <= std_logic_vector('0' & sh_g(3 downto 1));
                    blue  <= std_logic_vector('0' & sh_b(3 downto 1));
                else
                    red   <= std_logic_vector(sh_r);
                    green <= std_logic_vector(sh_g);
                    blue  <= std_logic_vector(sh_b);
                end if;
            end if;

            -- ── Object overrides (highest priority at bottom) ────

            if on_score_ring = '1' then
                red <= std_logic_vector(to_unsigned(RING_R, 4));
                green <= std_logic_vector(to_unsigned(RING_G, 4));
                blue  <= std_logic_vector(to_unsigned(RING_B, 4));
            end if;

            if on_obstacle = '1' then
                red   <= std_logic_vector(to_unsigned(OBS_R, 4));
                green <= std_logic_vector(to_unsigned(OBS_G, 4));
                blue  <= std_logic_vector(to_unsigned(OBS_B, 4));
            end if;

            if on_ship = '1' and game_state = "01" then
                red   <= std_logic_vector(to_unsigned(SHIP_R, 4));
                green <= std_logic_vector(to_unsigned(SHIP_G, 4));
                blue  <= std_logic_vector(to_unsigned(SHIP_B, 4));
            end if;

            -- ── Game-over: red flash ─────────────────────────────
            if game_state = "10" and vid_in = '1' then
                red   <= "1111";
                green <= "0000";
                blue  <= "0000";
            end if;

            -- ── Blanking: black out anything off-screen or in transition
            if vid_in = '0' or transition_active = '1' then
                red   <= "0000";
                green <= "0000";
                blue  <= "0000";
            end if;

        end if;
    end process;

end rtl;
