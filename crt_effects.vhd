library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- crt_effects.vhd  —  Operius-style sliding diagonal criss-cross
-- ================================================================
-- Draws a diamond criss-cross grid on the bottom layer, just like
-- Operius.  All old CRT effects (chromatic aberration, vignette,
-- film grain, scanline dimming) have been removed.
--
-- GEOMETRY
--   Two families of diagonal lines at ±45°:
--
--   "/" family  — slides toward lower-right as scroll increases:
--       (hpos + vpos + 1024 − scroll) mod 128 < LINE_WIDTH
--       1024 is a multiple of 128 so it is neutral mod 128.
--       As scroll grows each frame the constant shifts → "/" lines
--       drift toward the lower-right corner.  ✓
--
--   "\" family  — FIXED (no scroll term):
--       (hpos − vpos + 1024) mod 128 < LINE_WIDTH
--       The scroll direction (1,1) is geometrically parallel to "\"
--       lines, so adding a (1,1) offset to the grid leaves them at
--       exactly the same screen positions.  Only the intersection
--       points drift (which is the whole criss-cross motion).
--
--   mod 128  =  bits[6:0]  →  no divider, just a bit-slice.
--
-- COLOUR
--   Additive dim tint cycling slowly through 8 hues via hue_cnt.
--   Max line component = 3/15 ≈ 20 %.  On the tunnel's near-black
--   background (~1,0,1) lines read as (~4,0,4) — subtle but clear.
--   On bright tunnel content the small additive tint is invisible.
--
-- PIPELINE
--   One register stage.  hpos/vpos are delayed 2 cycles to stay
--   aligned with the tunnel's 2-stage pipeline (unchanged from the
--   old crt_effects).  scroll changes only once per frame so it
--   needs no delay.
-- ================================================================

entity crt_effects is
    port (
        clk_50    : in  std_logic;
        -- Pixel colour from tunnel renderer (2-clock pipeline behind VGA)
        red_in    : in  std_logic_vector(3 downto 0);
        green_in  : in  std_logic_vector(3 downto 0);
        blue_in   : in  std_logic_vector(3 downto 0);
        -- VGA scan position (undelayed)
        hpos      : in  std_logic_vector(9 downto 0);
        vpos      : in  std_logic_vector(9 downto 0);
        -- Hue cycle counter (increments once per frame) — drives line colour
        hue_cnt   : in  std_logic_vector(9 downto 0);
        -- Frame scroll counter (increments once per frame) — drives grid motion
        scroll    : in  std_logic_vector(7 downto 0);
        -- Post-processed output (1 clock after inputs)
        red_out   : out std_logic_vector(3 downto 0);
        green_out : out std_logic_vector(3 downto 0);
        blue_out  : out std_logic_vector(3 downto 0)
    );
end crt_effects;

architecture rtl of crt_effects is

    -- ── Grid geometry ────────────────────────────────────────────────────
    -- GRID_SPACING = 128 (power-of-2 → mod = bit slice, no multiplier)
    -- LINE_WIDTH   = 6   → 6-pixel-wide diagonal lines (~8 px visually)
    constant LINE_WIDTH : integer := 6;

    -- ── hpos/vpos delay registers (2 stages to match tunnel pipeline) ────
    signal hpos_d1, hpos_d2 : unsigned(9 downto 0) := (others => '0');
    signal vpos_d1, vpos_d2 : unsigned(9 downto 0) := (others => '0');

begin

    process(clk_50)
        -- 12-bit intermediates (large enough: max hpos+vpos+1024 = 2422 < 4096)
        variable h_slash : unsigned(11 downto 0);
        variable h_back  : unsigned(11 downto 0);
        variable on_grid : boolean;

        -- 5-bit colour accumulators
        variable r5, g5, b5 : unsigned(4 downto 0);
        variable lr, lg, lb  : unsigned(3 downto 0);
        -- Sum of input channels — used to detect background pixels (sum ≤ 2)
        variable in_sum      : unsigned(5 downto 0);

        variable hue_idx : integer range 0 to 7;
    begin
        if rising_edge(clk_50) then

            -- ── 1. Advance hpos/vpos delay chain ───────────────────────
            hpos_d1 <= unsigned(hpos);  hpos_d2 <= hpos_d1;
            vpos_d1 <= unsigned(vpos);  vpos_d2 <= vpos_d1;

            -- ── 2. Hue-cycling dim line colour ──────────────────────────
            -- hue_cnt[9:7] → 8 states, each 128 frames ≈ 1.8 s @ 72 Hz.
            -- Colours mirror the tunnel palette (neon pink → cyan arc)
            -- but dimmed to max component = 3 (20 % brightness).
            hue_idx := to_integer(unsigned(hue_cnt(9 downto 7)));
            case hue_idx is
                when 0      => lr := "0011"; lg := "0000"; lb := "0010"; -- dim pink-violet
                when 1      => lr := "0011"; lg := "0000"; lb := "0011"; -- dim magenta
                when 2      => lr := "0010"; lg := "0000"; lb := "0011"; -- dim violet
                when 3      => lr := "0001"; lg := "0000"; lb := "0011"; -- dim blue-violet
                when 4      => lr := "0000"; lg := "0001"; lb := "0011"; -- dim blue
                when 5      => lr := "0000"; lg := "0010"; lb := "0011"; -- dim cyan-blue
                when 6      => lr := "0000"; lg := "0011"; lb := "0011"; -- dim cyan
                when others => lr := "0000"; lg := "0011"; lb := "0010"; -- dim teal
            end case;

            -- ── 3. Criss-cross grid detection ───────────────────────────
            --
            -- "/" family — mod 128 (= bits[6:0]) of (hpos+vpos+1024-scroll).
            -- scroll is 8-bit (0-255); +1024 keeps the result ≥ 769 so
            -- unsigned subtraction never wraps.
            h_slash := resize(hpos_d2, 12)
                     + resize(vpos_d2, 12)
                     + to_unsigned(1024, 12)
                     - resize(unsigned(scroll), 12);

            -- "\" family — mod 128 of (hpos-vpos+1024).  FIXED (no scroll).
            -- Minimum: 0+1024-599 = 425 > 0, no underflow.
            h_back  := resize(hpos_d2, 12)
                     + to_unsigned(1024, 12)
                     - resize(vpos_d2, 12);

            -- On a line when the low 7 bits (= value mod 128) < LINE_WIDTH
            on_grid := (to_integer(h_slash(6 downto 0)) < LINE_WIDTH) or
                       (to_integer(h_back (6 downto 0)) < LINE_WIDTH);

            -- ── 4. Grid-below-everything blend ──────────────────────────
            -- The grid only replaces pixels that are still near-black
            -- background (tunnel output R+G+B ≤ 2 — the tunnel background
            -- is always (1,0,1) = sum 2; any ring/spoke/obstacle/ship is
            -- always brighter, sum ≥ 3).
            -- On non-background pixels the tunnel output passes through
            -- completely unchanged, so the grid is truly the bottom layer.
            in_sum := resize(unsigned(red_in),   6)
                    + resize(unsigned(green_in), 6)
                    + resize(unsigned(blue_in),  6);

            if on_grid and in_sum <= 2 then
                -- Background pixel on a grid line → draw the line colour
                r5 := resize(lr, 5);
                g5 := resize(lg, 5);
                b5 := resize(lb, 5);
            else
                -- Tunnel content (or background not on a line) → pass through
                r5 := resize(unsigned(red_in),   5);
                g5 := resize(unsigned(green_in), 5);
                b5 := resize(unsigned(blue_in),  5);
            end if;

            -- ── 5. Register output ───────────────────────────────────────
            red_out   <= std_logic_vector(r5(3 downto 0));
            green_out <= std_logic_vector(g5(3 downto 0));
            blue_out  <= std_logic_vector(b5(3 downto 0));

        end if;
    end process;

end rtl;
