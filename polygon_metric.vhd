library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- polygon_metric.vhd
-- ================================================================
-- Purely combinational Stage-1 metric for the tunnel renderer.
-- Extracted so polygon shape can be swapped without touching the
-- renderer or colour logic.
--
-- METRIC SELECTION (num_faces):
--   N=3,4  →  square   :  max(adx, ady_eff)
--   N=5,6  →  hexagon  :  max(adx, (adx + ady_eff) >> 1)
--   N=7,8  →  octagon  :  max(max(adx, ady_eff), 0.75*(adx+ady_eff))
--   Odd N rounds up to the next even N for the distance formula;
--   pix_sec still uses the raw 3-bit octant so gap matching works
--   consistently with obstacle_manager.
--
-- ady_eff = 1.5 × ady  (wider-than-tall: same poly_dist reached at
--   a smaller actual dy, so the tunnel looks wider than it is tall).
--
-- LOG-BAND ENCODING:
--   log_band[7:4] = floor(log2(poly_dist))
--   log_band[3:0] = next 4 mantissa bits  (linear sub-band interp)
--   ring_log_ctr  = log_band − scroll  (animated tunnel fly-through)
--   Ring period = 8 log-band units → ~6-7 rings visible, cascading
--   to infinity near the vanishing point.
--
-- FACE DIFF / SPOKES:
--   face_diff = |sum_shrunk − oct_max_xy|
--   on_spoke when face_diff < SPOKE_WIDTH and poly_dist >= MIN_DIST
--   The formula is based on the octagon basis but the spoke-boundary
--   sits at the 45° diagonal for all three metric shapes, giving a
--   consistent 8-spoke visual for any N (or 4-spoke for N=3/4).
--
-- DEPTH SHADE:
--   depth_shade = poly_dist[9:5] + 1, saturated at 15
--   Colour channels multiply by this so rings fade black at void.
--
-- PIXEL SECTOR:
--   pix_sec[2] = adx ≥ ady_eff  (x-dominant vs y-dominant face)
--   pix_sec[1] = dx_rot ≥ 0     (right half)
--   pix_sec[0] = dy_rot ≥ 0     (bottom half)
--   → 8 unique octant codes matching obstacle_manager gap_sector.
-- ================================================================

entity polygon_metric is
    port (
        dx_rot      : in  signed(11 downto 0);   -- rotated X (Stage 0 register output)
        dy_rot      : in  signed(11 downto 0);   -- rotated Y (Stage 0 register output)
        num_faces   : in  std_logic_vector(3 downto 0);  -- 4-bit: 3..8
        scroll      : in  std_logic_vector(7 downto 0);
        -- Distance
        poly_dist   : out unsigned(10 downto 0);
        -- Ring detection
        log_band    : out unsigned(7 downto 0);
        is_ring     : out std_logic;
        -- Spoke detection
        face_diff   : out unsigned(10 downto 0);
        on_spoke    : out std_logic;
        -- Depth
        depth_shade : out unsigned(3 downto 0);
        -- Sector
        pix_sec     : out std_logic_vector(2 downto 0)
    );
end polygon_metric;

architecture rtl of polygon_metric is

    constant MIN_DIST    : integer := 48;
    constant SPOKE_WIDTH : integer := 4;

    -- ----------------------------------------------------------------
    -- Truncated 11-bit signed coords (dx_rot/dy_rot are ±538 max,
    -- well inside signed(10:0) range of ±1023).
    -- ----------------------------------------------------------------
    signal dx_s1, dy_s1   : signed(10 downto 0);
    signal dx_abs, dy_abs : signed(10 downto 0);
    signal adx, ady       : unsigned(9 downto 0);
    signal ady_eff        : unsigned(10 downto 0);   -- 1.5 × ady

    -- Shared intermediates used across metrics
    signal sum_ad         : unsigned(10 downto 0);   -- adx + ady_eff
    signal sum_shrunk     : unsigned(10 downto 0);   -- sum_ad × 0.75
    signal oct_max_xy     : unsigned(10 downto 0);   -- max(adx, ady_eff)
    signal hex_half       : unsigned(10 downto 0);   -- sum_ad >> 1

    -- Metric result and log encoder
    signal poly_dist_s    : unsigned(10 downto 0);
    signal log_int_s      : unsigned(3 downto 0);
    signal log_frac_s     : unsigned(3 downto 0);
    signal log_band_s     : unsigned(7 downto 0);
    signal ring_log_ctr   : unsigned(7 downto 0);
    signal ring_width_v   : unsigned(3 downto 0);

    -- Spoke
    signal face_diff_s    : unsigned(10 downto 0);

    -- Depth
    signal depth_idx_s    : unsigned(5 downto 0);

begin

    -- ================================================================
    -- Absolute rotated coordinates (lower 11 bits; MSB never needed)
    -- ================================================================
    dx_s1  <= dx_rot(10 downto 0);
    dy_s1  <= dy_rot(10 downto 0);
    dx_abs <= abs(dx_s1);
    dy_abs <= abs(dy_s1);
    adx    <= unsigned(dx_abs(9 downto 0));
    ady    <= unsigned(dy_abs(9 downto 0));

    -- ady_eff = ady + (ady >> 1)  =  ady × 1.5   (11-bit result)
    ady_eff <= ('0' & ady) + ('0' & '0' & ady(9 downto 1));

    -- ================================================================
    -- Shared intermediate signals (all metrics re-use these)
    -- ================================================================
    sum_ad     <= ('0' & adx) + ady_eff;
    sum_shrunk <= sum_ad - ('0' & '0' & sum_ad(10 downto 2));  -- × 0.75
    oct_max_xy <= ('0' & adx) when (('0' & adx) >= ady_eff) else ady_eff;
    hex_half   <= '0' & sum_ad(10 downto 1);                   -- sum_ad / 2

    -- ================================================================
    -- N-gon distance metric MUX
    -- ================================================================
    -- NOTE: N=3 uses signed dx_s1/dy_s1 (not abs) because the
    -- triangle is NOT symmetric under |·|.  The other cases use abs
    -- values (adx, ady_eff) which is safe for their even symmetry.
    -- ================================================================
    process(num_faces, adx, ady_eff, oct_max_xy, sum_shrunk, hex_half,
            dx_s1, dy_s1)
        variable n           : integer range 0 to 15;
        variable dx12, dy12  : signed(11 downto 0);   -- 12-bit sign-ext
        variable c1, c2, c3  : signed(11 downto 0);   -- per-face components
        variable tri_raw     : signed(11 downto 0);
    begin
        n := to_integer(unsigned(num_faces));
        case n is

            -- ── Triangle (N=3) ──────────────────────────────────────
            -- Equilateral, flat bottom face pointing toward screen base.
            -- Three inward face normals (screen coords, y grows down):
            --   bottom      :  (0, 1)          → c1 = dy * 1.5
            --   upper-right :  ( sin60, −cos60) → c2 = dx*7/8 − dy/2
            --   upper-left  :  (−sin60, −cos60) → c3 = −dx*7/8 − dy/2
            --   (7/8 ≈ sin 60° = 0.866, dy scaled ×1.5 for wider-than-tall)
            -- poly_dist = max(c1,c2,c3), clipped to 0 inside the void.
            when 3 =>
                dx12 := resize(dx_s1, 12);
                dy12 := resize(dy_s1, 12);
                c1   := dy12 + shift_right(dy12, 1);                           -- dy×1.5
                c2   := dx12 - shift_right(dx12, 3) - shift_right(dy12, 1);    -- dx×7/8 − dy/2
                c3   := -(dx12 - shift_right(dx12, 3)) - shift_right(dy12, 1); -- −dx×7/8 − dy/2
                if c1 >= c2 then tri_raw := c1; else tri_raw := c2; end if;
                if c3 > tri_raw then tri_raw := c3; end if;
                if tri_raw < 0 then
                    poly_dist_s <= (others => '0');       -- inside void
                else
                    poly_dist_s <= unsigned(tri_raw(10 downto 0));
                end if;

            -- ── Square (N=4) ─────────────────────────────────────────
            -- max(|dx|, ady_eff)
            when 4 =>
                poly_dist_s <= oct_max_xy;

            -- ── Hexagon (N=5 or N=6) ─────────────────────────────────
            -- max(|dx|, (|dx|+ady_eff)/2)
            when 5 | 6 =>
                if ('0' & adx) >= hex_half then
                    poly_dist_s <= '0' & adx;
                else
                    poly_dist_s <= hex_half;
                end if;

            -- ── Octagon (N=7, N=8, default) ──────────────────────────
            -- max(max(|dx|,ady_eff), 0.75*(|dx|+ady_eff))
            when others =>
                if sum_shrunk > oct_max_xy then
                    poly_dist_s <= sum_shrunk;
                else
                    poly_dist_s <= oct_max_xy;
                end if;

        end case;
    end process;

    poly_dist <= poly_dist_s;

    -- ================================================================
    -- Log2 priority encoder
    -- Covers poly_dist range 4..1009  (bit 10 is never set on our screen).
    -- log_band = log_int[7:4] & log_frac[3:0]
    -- ================================================================
    process(poly_dist_s)
    begin
        if    poly_dist_s(9) = '1' then
            log_int_s <= "1001"; log_frac_s <= poly_dist_s(8 downto 5);
        elsif poly_dist_s(8) = '1' then
            log_int_s <= "1000"; log_frac_s <= poly_dist_s(7 downto 4);
        elsif poly_dist_s(7) = '1' then
            log_int_s <= "0111"; log_frac_s <= poly_dist_s(6 downto 3);
        elsif poly_dist_s(6) = '1' then
            log_int_s <= "0110"; log_frac_s <= poly_dist_s(5 downto 2);
        elsif poly_dist_s(5) = '1' then
            log_int_s <= "0101"; log_frac_s <= poly_dist_s(4 downto 1);
        elsif poly_dist_s(4) = '1' then
            log_int_s <= "0100"; log_frac_s <= poly_dist_s(3 downto 0);
        elsif poly_dist_s(3) = '1' then
            log_int_s <= "0011"; log_frac_s <= poly_dist_s(2 downto 0) & '0';
        elsif poly_dist_s(2) = '1' then
            log_int_s <= "0010"; log_frac_s <= poly_dist_s(1 downto 0) & "00";
        else
            log_int_s <= "0000"; log_frac_s <= poly_dist_s(3 downto 0);
        end if;
    end process;

    log_band_s   <= log_int_s & log_frac_s;
    log_band     <= log_band_s;

    -- ring_log_ctr wraps naturally (8-bit unsigned subtraction)
    ring_log_ctr <= log_band_s - unsigned(scroll);

    -- Inverse width taper: at large poly_dist each log-unit covers
    -- more pixels → use 1 band; near void use 2 bands.  Both give
    -- ~1-2 px apparent ring thickness.
    ring_width_v <= "0001" when log_int_s >= "1000" else "0010";

    -- Ring period = 8 log-band units (doubled density vs period 16)
    is_ring <= '1' when (ring_log_ctr(2 downto 0) < ring_width_v(2 downto 0)
                         and poly_dist_s >= MIN_DIST)
               else '0';

    -- ================================================================
    -- Spoke detection
    -- face_diff = distance from nearest 45-deg spoke boundary.
    -- The boundary is where sum_shrunk = oct_max_xy (true for
    -- octagon; approximation for hex/square — spokes appear at 45°).
    -- ================================================================
    face_diff_s <= (sum_shrunk - oct_max_xy) when sum_shrunk >= oct_max_xy
                   else (oct_max_xy - sum_shrunk);
    face_diff <= face_diff_s;
    on_spoke  <= '1' when (face_diff_s < SPOKE_WIDTH and poly_dist_s >= MIN_DIST)
                 else '0';

    -- ================================================================
    -- Depth shading
    -- poly_dist[9:5] gives 0..31 across the screen;  +1 so the
    -- innermost visible ring starts at shade=2 (near-black, not black).
    -- Saturate at 15 for 4-bit output.
    -- ================================================================
    depth_idx_s <= ('0' & poly_dist_s(9 downto 5)) + 1;
    depth_shade <= depth_idx_s(3 downto 0) when depth_idx_s < 16 else "1111";

    -- ================================================================
    -- Pixel sector (3-bit octant, tunnel frame)
    -- Compatible with obs*_gap_sector in obstacle_manager.
    -- ================================================================
    pix_sec(2) <= '1' when ('0' & adx) >= ady_eff else '0';
    pix_sec(1) <= not dx_s1(10);   -- dx ≥ 0  → right half
    pix_sec(0) <= not dy_s1(10);   -- dy ≥ 0  → bottom half

end rtl;
