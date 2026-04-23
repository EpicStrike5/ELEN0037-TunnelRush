library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- tunnel_background.vhd
-- ================================================================
-- Generates the screen-space background pattern.
--
-- DESIGN NOTE:
--   The background is drawn in SCREEN space (not tunnel/rotated space),
--   so it never rotates with the ship or tunnel.  It scrolls purely
--   in the horizontal direction at a fixed slow speed.
--
-- ──────────────────────────────────────────────────────────────
-- CURRENT IMPLEMENTATION: horizontal dashed lines
-- ──────────────────────────────────────────────────────────────
--   Thick horizontal bands spaced BG_PERIOD pixels apart, scrolling
--   slowly leftward at one pixel per BG_SCROLL_DIV frames.
--
--   TUNING:
--     BG_PERIOD     [64]  — vertical spacing between background bands
--                           DECREASE for more lines.  INCREASE for fewer.
--     BG_THICKNESS  [2]   — line width in pixels
--                           INCREASE for thicker lines.
--     DASH_PERIOD   [32]  — horizontal dash repeat length
--                           DECREASE for shorter dashes.
--     DASH_ON       [16]  — visible portion of each dash
--
-- OUTPUTS:
--   bg_active : '1' when this pixel is on a background feature.
--               Used by colour_palette to dim overlapping pixels
--               (optional — wire to '0' to disable entirely).
--
-- TO SWITCH PATTERN:
--   Replace the concurrent assignment of bg_active below.
--   The inputs hpos/vpos are raw screen coordinates (0..799 / 0..599).
--   bg_scroll is a slow horizontal counter you can wire from top level.
--
-- FUTURE PATTERNS (examples):
--   • Diagonal cross-hatch:  steep diagonal lines in two directions
--   • Dot grid:              (hpos mod P) < 1 and (vpos mod P) < 1
--   • Solid background color: bg_active always '0', set bg_colour in palette
-- ================================================================

entity tunnel_background is
    port (
        hpos      : in  std_logic_vector(9 downto 0);  -- screen X (0..799)
        vpos      : in  std_logic_vector(9 downto 0);  -- screen Y (0..599)
        bg_scroll : in  std_logic_vector(9 downto 0);  -- slow scroll counter
        bg_active : out std_logic                       -- '1' = on background feature
    );
end tunnel_background;

architecture rtl of tunnel_background is

    -- ──────────────────────────────────────────────────────────────
    -- TUNING PARAMETERS
    -- ──────────────────────────────────────────────────────────────
    constant BG_PERIOD    : integer := 64;   -- vertical spacing between lines
    constant BG_THICKNESS : integer := 2;    -- line height in pixels
    constant DASH_PERIOD  : integer := 32;   -- horizontal dash repeat length
    constant DASH_ON      : integer := 16;   -- visible pixels per dash

    signal hpos_u, vpos_u : unsigned(9 downto 0);
    signal scroll_u       : unsigned(9 downto 0);
    signal h_scrolled     : unsigned(9 downto 0);    -- hpos + scroll (intermediate)

    -- Modulo helpers (power-of-2 friendly: BG_PERIOD=64, DASH_PERIOD=32)
    signal v_pos_in_period : unsigned(5 downto 0);   -- vpos mod BG_PERIOD (0..63)
    signal h_pos_in_dash   : unsigned(4 downto 0);   -- (hpos+scroll) mod DASH_PERIOD (0..31)

    signal on_h_line : std_logic;
    signal on_dash   : std_logic;

begin

    hpos_u   <= unsigned(hpos);
    vpos_u   <= unsigned(vpos);
    scroll_u <= unsigned(bg_scroll);

    -- Vertical position within each band (mod 64 = bottom 6 bits)
    v_pos_in_period <= vpos_u(5 downto 0);

    -- Horizontal position within each dash, accounting for slow scroll.
    -- (mod 32 = bottom 5 bits of scrolled position)
    -- Intermediate signal required: VHDL-93 forbids slicing an expression directly.
    h_scrolled    <= hpos_u + scroll_u;
    h_pos_in_dash <= h_scrolled(4 downto 0);

    -- Pixel is on a horizontal line band
    on_h_line <= '1' when to_integer(v_pos_in_period) < BG_THICKNESS else '0';

    -- Pixel is in the visible part of a dash
    on_dash   <= '1' when to_integer(h_pos_in_dash)   < DASH_ON      else '0';

    -- Background feature: horizontal dashed line
    -- REPLACE this line to change the background pattern:
    bg_active <= on_h_line and on_dash;

end rtl;
