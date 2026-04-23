library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- tunnel_core.vhd
-- ================================================================
-- Pure combinational N-gon geometry.  Single responsibility:
-- given a rotated pixel position (dx_rot, dy_rot) relative to
-- screen centre, decide what that pixel IS inside the tunnel.
--
-- ──────────────────────────────────────────────────────────────
-- VISUAL TUNING PARAMETERS (only things to touch for look changes)
-- ──────────────────────────────────────────────────────────────
--   RING_PERIOD_LOG2   [7]
--       log2 of the log-space interval between ring lines.
--       2^7 = 128 sub-octave steps ≈ 2 rings per octave.
--       DECREASE (e.g. 6) → denser rings.   INCREASE → sparser.
--
--   RING_THICKNESS     [1]
--       How many log sub-steps the ring line occupies.
--       At the screen edge 1 step ≈ 1–2 px.  Increase for fatter rings.
--
--   SPOKE_THRESH       [4]
--       Absolute tolerance (in the N-gon projection metric) to count
--       a pixel as "on a spoke".  ~1 px perpendicular width per unit.
--       INCREASE for fatter spokes.
--
--   DEAD_ZONE_R        [10]
--       Radius (screen pixels) of the black void at the vanishing point.
--       Rings and spokes are suppressed inside this circle.
--       Increase for a bigger soft-black centre hole.
--
--   ADY_SCALE_NUM/DEN  [3/2]
--       Vertical stretch factor ADY_EFF = ady * NUM/DEN.
--       3/2 = 1.5× makes tunnel wider-than-tall on a 4:3 screen.
--       Change to 4/3 for a more square-looking polygon on 800×600.
-- ──────────────────────────────────────────────────────────────
-- OUTPUTS:
--   is_ring     : pixel is on a ring line
--   is_spoke    : pixel is on a spoke line (extends to screen edge)
--   in_dead_zone: pixel inside the centre void (suppress ring+spoke)
--   log_band    : 8-bit depth index (top 8 bits of 12-bit log2 value)
--                 used by tunnel_renderer for obstacle depth matching
--   depth_shade : 0=void/dark .. 15=outer/bright (for colour scaling)
--   pix_sec     : 3-bit sector identifying which N-gon face this
--                 pixel belongs to (matches obstacle_manager sectors)
-- ================================================================

entity tunnel_core is
    port (
        dx_rot      : in  signed(11 downto 0);  -- rotated X from coord_transform
        dy_rot      : in  signed(11 downto 0);  -- rotated Y from coord_transform
        num_faces   : in  std_logic_vector(3 downto 0);  -- 3..8
        scroll      : in  std_logic_vector(7 downto 0);  -- animation scroll counter
        -- Classification outputs
        is_ring     : out std_logic;
        is_spoke    : out std_logic;
        in_dead_zone: out std_logic;
        -- Depth / sector (used by renderer for obstacles)
        log_band    : out unsigned(7 downto 0);
        depth_shade : out unsigned(3 downto 0);
        pix_sec     : out std_logic_vector(2 downto 0)
    );
end tunnel_core;

architecture rtl of tunnel_core is

    -- ──────────────────────────────────────────────────────────────
    -- TUNING PARAMETERS
    -- ──────────────────────────────────────────────────────────────
    constant RING_PERIOD_LOG2 : integer := 7;   -- ring every 2^7 = 128 log-steps
    constant RING_THICKNESS   : integer := 1;   -- 1 log-step ≈ 1 px at screen edge
    constant SPOKE_THRESH     : integer := 4;   -- projection tolerance for spokes
    constant DEAD_ZONE_R      : integer := 4;   -- void radius in pixels (keep tiny: bright core effect)
    -- ady vertical stretch: ady_eff = ady * ADY_NUM / ADY_DEN
    constant ADY_NUM : integer := 3;  -- numerator   (3/2 = 1.5×)
    constant ADY_DEN : integer := 2;  -- denominator

    constant DEAD_ZONE_SQ : integer := DEAD_ZONE_R * DEAD_ZONE_R;

    -- ──────────────────────────────────────────────────────────────
    -- Intermediate signals
    -- ──────────────────────────────────────────────────────────────
    signal dx11, dy11      : signed(10 downto 0);
    signal dx11_abs, dy11_abs : signed(10 downto 0);  -- abs intermediates
    signal adx, ady        : unsigned(9 downto 0);

    -- ady_eff = ady × 1.5  (vertical aspect correction, 11-bit)
    signal ady_eff     : unsigned(10 downto 0);

    -- N-gon metric result (pixel scale)
    signal poly_dist_s : unsigned(10 downto 0);

    -- 12-bit log2 of poly_dist: [11:8] = integer, [7:0] = fractional (1/256 octave)
    signal log12       : unsigned(11 downto 0);

    -- Ring animation counter (log12 minus scroll offset, wraps)
    signal ring_ctr    : unsigned(11 downto 0);

    -- Spoke: minimum distance to nearest face-boundary line
    signal spoke_diff  : unsigned(10 downto 0);

    -- Dead zone flag
    signal in_dz       : std_logic;

    -- Depth helpers
    signal depth_raw   : unsigned(5 downto 0);

begin

    -- ================================================================
    -- Coordinate preparation
    -- ================================================================
    dx11      <= dx_rot(10 downto 0);
    dy11      <= dy_rot(10 downto 0);
    dx11_abs  <= abs(dx11);   -- intermediate: abs() result must be in a signal before slicing
    dy11_abs  <= abs(dy11);
    adx       <= unsigned(dx11_abs(9 downto 0));
    ady       <= unsigned(dy11_abs(9 downto 0));

    -- ady_eff = ady + ady/2  (= ady × 3/2 = 1.5×)
    -- CHANGE ADY_NUM/ADY_DEN above to adjust vertical stretch.
    ady_eff <= ('0' & ady) + ("00" & ady(9 downto 1));

    -- ================================================================
    -- Dead zone: r² = adx² + ady²  compared against DEAD_ZONE_R²
    -- Uses true Euclidean distance, no sqrt needed.
    -- ================================================================
    process(adx, ady)
        variable rsq : unsigned(19 downto 0);
    begin
        rsq := (adx * adx) + (ady * ady);
        if rsq < DEAD_ZONE_SQ then
            in_dz <= '1';
        else
            in_dz <= '0';
        end if;
    end process;
    in_dead_zone <= in_dz;

    -- ================================================================
    -- N-gon distance metric
    -- ------------------------------------------------------------------
    -- poly_dist is the generalised "radius" of the N-gon that just
    -- contains the current pixel.  It equals the max face-projection:
    --     poly_dist = max_k (dx·cos α_k + dy·sin α_k)
    -- where α_k = k·2π/N are the outward face-normal angles.
    --
    -- Implemented as closed-form per N-type using integer arithmetic:
    --   N=4  square  : max(|dx|, ady_eff)
    --   N=6  hexagon : max(|dx|, (|dx|+ady_eff)/2)
    --   N=8  octagon : max(max(|dx|,ady_eff), (|dx|+ady_eff)×¾)
    --   N=3  triangle: three signed projections, max of three
    --   N=5,7        : mapped to nearest even N above
    -- ================================================================
    process(num_faces, adx, ady_eff, dx11, dy11)
        variable n               : integer range 0 to 15;
        variable p_axial         : unsigned(10 downto 0);  -- max(|dx|, ady_eff)
        variable p_sum           : unsigned(10 downto 0);  -- |dx| + ady_eff
        variable p_oct           : unsigned(10 downto 0);  -- p_sum × 3/4  (octagon diag)
        variable p_hex           : unsigned(10 downto 0);  -- p_sum / 2    (hexagon mid)
        -- Triangle variables (need signed coords)
        variable dx12, dy12      : signed(11 downto 0);
        variable c1, c2, c3      : signed(11 downto 0);
        variable tri_raw         : signed(11 downto 0);
    begin
        n       := to_integer(unsigned(num_faces));
        if ('0' & adx) >= ady_eff then p_axial := ('0' & adx);
        else                           p_axial := ady_eff;
        end if;
        p_sum   := ('0' & adx) + ady_eff;
        -- p_sum × 3/4 = p_sum - p_sum/4
        p_oct   := p_sum - ("00" & p_sum(10 downto 2));
        -- p_sum / 2
        p_hex   := '0' & p_sum(10 downto 1);

        case n is

            -- ── Triangle (N=3) ───────────────────────────────────
            -- Flat-bottom equilateral: bottom face normal = (0,+1),
            -- upper-right ≈ (sin60°, -cos60°) ≈ (7/8, -1/2),
            -- upper-left  ≈ (-7/8, -1/2).
            -- Must use SIGNED coords (triangle is not origin-symmetric).
            when 3 =>
                dx12 := resize(dx11, 12);
                dy12 := resize(dy11, 12);
                c1   := dy12 + shift_right(dy12, 1);                             -- dy × 1.5
                c2   := dx12 - shift_right(dx12, 3) - shift_right(dy12, 1);      -- dx×7/8 − dy/2
                c3   := -(dx12 - shift_right(dx12, 3)) - shift_right(dy12, 1);   -- -dx×7/8 − dy/2
                tri_raw := c1;
                if c2 > tri_raw then tri_raw := c2; end if;
                if c3 > tri_raw then tri_raw := c3; end if;
                -- Halve poly_dist for triangle: its apothem-to-corner ratio is 2:1
                -- (vs ~1.1:1 for octagon), so without correction the tunnel looks 2×
                -- larger than all other shapes.  Right-shift by 1 normalises the size.
                if tri_raw < 0 then poly_dist_s <= (others => '0');
                else                 poly_dist_s <= '0' & unsigned(tri_raw(10 downto 1));
                end if;

            -- ── Square (N=4) ─────────────────────────────────────
            when 4 =>
                poly_dist_s <= p_axial;

            -- ── Hexagon (N=5 mapped to 6, N=6) ───────────────────
            when 5 | 6 =>
                if ('0' & adx) >= p_hex then poly_dist_s <= '0' & adx;
                else                          poly_dist_s <= p_hex;
                end if;

            -- ── Octagon (N=7 mapped to 8, N=8, default) ──────────
            when others =>
                if p_oct >= p_axial then poly_dist_s <= p_oct;
                else                     poly_dist_s <= p_axial;
                end if;

        end case;
    end process;

    -- ================================================================
    -- Log2 encoder — 12-bit precision
    -- ------------------------------------------------------------------
    -- log12[11:8] = floor(log2(poly_dist))  — which octave (0..9)
    -- log12[7:0]  = fractional mantissa     — position within octave
    --               = the 8 bits immediately after the leading '1' bit
    --               (zero-padded on the right if poly_dist < 256)
    --
    -- This gives ~1-pixel ring width over the full depth range:
    --   1 log-step at r=256px ≈ 256 × (2^(1/256)−1) ≈ 0.7 px  ✓
    --   1 log-step at r=512px ≈ 1.4 px                          ✓
    -- ================================================================
    process(poly_dist_s)
    begin
        if    poly_dist_s(9) = '1' then
            log12 <= "1001" & poly_dist_s(8 downto 1);
        elsif poly_dist_s(8) = '1' then
            log12 <= "1000" & poly_dist_s(7 downto 0);
        elsif poly_dist_s(7) = '1' then
            log12 <= "0111" & poly_dist_s(6 downto 0) & '0';
        elsif poly_dist_s(6) = '1' then
            log12 <= "0110" & poly_dist_s(5 downto 0) & "00";
        elsif poly_dist_s(5) = '1' then
            log12 <= "0101" & poly_dist_s(4 downto 0) & "000";
        elsif poly_dist_s(4) = '1' then
            log12 <= "0100" & poly_dist_s(3 downto 0) & "0000";
        elsif poly_dist_s(3) = '1' then
            log12 <= "0011" & poly_dist_s(2 downto 0) & "00000";
        elsif poly_dist_s(2) = '1' then
            log12 <= "0010" & poly_dist_s(1 downto 0) & "000000";
        elsif poly_dist_s(1) = '1' then
            log12 <= "0001" & poly_dist_s(0)          & "0000000";
        else
            log12 <= (others => '0');
        end if;
    end process;

    -- log_band output: top 8 bits of log12 (backwards-compatible with
    -- obstacle_manager's 8-bit depth comparison).
    log_band <= log12(11 downto 4);

    -- ================================================================
    -- Ring animation counter
    -- ------------------------------------------------------------------
    -- ring_ctr = log12 - scroll×8
    -- Each frame scroll increments by 1 → log12 effectively decreases
    -- by 8 sub-steps per frame.  With RING_PERIOD = 2^7 = 128 steps,
    -- one ring travels across the screen in 128/8 = 16 frames (≈0.22s).
    -- CHANGE the appended zeros to adjust tunnel speed:
    --   "0000" = ×16 → 8 frames/ring   (fast, arcade-like)
    --   "000"  = ×8  → 16 frames/ring  (moderate)  ← current
    --   "00"   = ×4  → 32 frames/ring  (slow / hypnotic)
    -- ================================================================
    ring_ctr <= log12 - (unsigned(scroll) & "000");

    -- Ring is visible for RING_THICKNESS consecutive steps out of 2^RING_PERIOD_LOG2.
    is_ring <= '1' when (to_integer(ring_ctr(RING_PERIOD_LOG2 - 1 downto 0)) < RING_THICKNESS
                         and in_dz = '0')
              else '0';

    -- ================================================================
    -- Spoke detection
    -- ------------------------------------------------------------------
    -- N spokes for an N-gon: one spoke per CORNER (vertex) direction.
    -- A corner is the boundary between two adjacent N-gon faces.
    --
    -- Method: N-gon metric = max over face projections.
    -- A pixel is on a spoke when two adjacent face projections are
    -- nearly equal → spoke_diff = (poly_dist − second_max) < THRESH.
    --
    -- Using abs(dx)/abs(dy) gives 4-fold symmetry, so we work within
    -- one quadrant and count how many distinct spoke angles exist there.
    --
    -- Spoke count per shape:
    --   N=3 (triangle) : 3 spokes — uses signed coords, not abs()
    --   N=4 (square)   : 4 spokes — 1 diagonal spoke per quadrant
    --                    Fires at: |adx − ady_eff| < THRESH
    --   N=5 (→ hex)    : 6 spokes — same as N=6 (hex approximation)
    --   N=6 (hexagon)  : 6 spokes — 1 diagonal + 1 vertical per quadrant
    --                    Diagonal: |adx − ady_eff| < THRESH  (4 total)
    --                    Vertical: adx < THRESH              (2 total)
    --   N=7 (→ oct)    : 8 spokes — same as N=8 (oct approximation)
    --   N=8 (octagon)  : 8 spokes — 2 spokes per quadrant
    --                    |adx − p_oct| < THRESH  OR
    --                    |ady_eff − p_oct| < THRESH
    -- ================================================================
    process(num_faces, adx, ady_eff, dx11, dy11)
        variable n                : integer range 0 to 15;
        variable p_sum_v          : unsigned(10 downto 0);
        variable p_oct_v          : unsigned(10 downto 0);
        variable diff_a, diff_b   : unsigned(10 downto 0);
        -- Triangle projection variables (signed coords required)
        variable dx12v, dy12v     : signed(11 downto 0);
        variable c1v, c2v, c3v   : signed(11 downto 0);
        variable diff_s           : signed(11 downto 0);   -- intermediate for abs of signed diff
        variable d12, d23, d13   : unsigned(10 downto 0);
    begin
        n       := to_integer(unsigned(num_faces));
        p_sum_v := ('0' & adx) + ady_eff;
        p_oct_v := p_sum_v - ("00" & p_sum_v(10 downto 2));  -- p_sum × 3/4

        case n is

            -- ── Triangle (3 spokes) ──────────────────────────────
            -- Three face projections; spoke at each boundary between
            -- two projections.  Must use signed coords (not abs) because
            -- the triangle is NOT origin-symmetric.
            when 3 =>
                dx12v := resize(dx11, 12);
                dy12v := resize(dy11, 12);
                c1v   := dy12v + shift_right(dy12v, 1);
                c2v   := dx12v - shift_right(dx12v, 3) - shift_right(dy12v, 1);
                c3v   := -(dx12v - shift_right(dx12v, 3)) - shift_right(dy12v, 1);
                -- Compute |cX-cY| for each pair of face projections.
                -- Store diff in a variable before slicing (VHDL-93 requirement).
                if c1v >= c2v then diff_s := c1v - c2v; else diff_s := c2v - c1v; end if;
                d12 := unsigned(diff_s(10 downto 0));
                if c2v >= c3v then diff_s := c2v - c3v; else diff_s := c3v - c2v; end if;
                d23 := unsigned(diff_s(10 downto 0));
                if c1v >= c3v then diff_s := c1v - c3v; else diff_s := c3v - c1v; end if;
                d13 := unsigned(diff_s(10 downto 0));

                -- ── Directionality guard ────────────────────────────
                -- Each face-boundary is an infinite line through the origin,
                -- giving 2 rays per boundary → 6 apparent spokes instead of 3.
                -- Fix: invalidate boundary dXY when the THIRD face is dominant
                -- (i.e. on the "wrong" side of the boundary line).
                --
                --  d12 (c1≈c2): valid at bottom-right vertex → c3 must be lowest
                --  d23 (c2≈c3): valid at top vertex           → c1 must be lowest
                --  d13 (c1≈c3): valid at bottom-left vertex   → c2 must be lowest
                if c3v >= c1v or c3v >= c2v then d12 := (others => '1'); end if;
                if c1v >= c2v or c1v >= c3v then d23 := (others => '1'); end if;
                if c2v >= c1v or c2v >= c3v then d13 := (others => '1'); end if;

                if d12 <= d23 and d12 <= d13 then spoke_diff <= d12;
                elsif d23 <= d13             then spoke_diff <= d23;
                else                              spoke_diff <= d13;
                end if;

            -- ── Square (4 spokes) ────────────────────────────────
            -- One corner per quadrant, at the 45° diagonal.
            -- With ady_eff stretch: fires at adx ≈ ady_eff (≈34° screen angle).
            when 4 =>
                if ('0' & adx) >= ady_eff then diff_a := ('0' & adx) - ady_eff;
                else                           diff_a := ady_eff - ('0' & adx);
                end if;
                spoke_diff <= diff_a;

            -- ── Hexagon (6 spokes): N=5 treated as hexagon ───────
            -- Two families of corners, giving 6 total across the screen:
            --   DIAGONAL (4): boundary between adx-face and diag-face.
            --                 Fires when adx ≈ ady_eff (|adx-ady_eff| small).
            --   VERTICAL (2): boundary between the two diagonal-faces.
            --                 Fires when adx ≈ 0 (pixel near y-axis).
            -- spoke_diff = min(|adx - ady_eff|, adx) catches both.
            when 5 | 6 =>
                if ('0' & adx) >= ady_eff then diff_a := ('0' & adx) - ady_eff;
                else                           diff_a := ady_eff - ('0' & adx);
                end if;
                diff_b := '0' & adx;  -- distance to vertical axis
                if diff_a <= diff_b then spoke_diff <= diff_a;
                else                     spoke_diff <= diff_b;
                end if;

            -- ── Octagon (8 spokes): N=7 treated as octagon ───────
            -- Two corner families per quadrant, giving 8 total:
            --   |adx − p_oct| < THRESH   (corner at ~12° screen angle)
            --   |ady_eff − p_oct| < THRESH (corner at ~63° screen angle)
            -- spoke_diff = min of the two.
            when others =>
                if ('0' & adx) >= p_oct_v then diff_a := ('0' & adx) - p_oct_v;
                else                           diff_a := p_oct_v - ('0' & adx);
                end if;
                if ady_eff >= p_oct_v then diff_b := ady_eff - p_oct_v;
                else                       diff_b := p_oct_v - ady_eff;
                end if;
                if diff_a <= diff_b then spoke_diff <= diff_a;
                else                     spoke_diff <= diff_b;
                end if;

        end case;
    end process;

    is_spoke <= '1' when spoke_diff < SPOKE_THRESH and in_dz = '0' else '0';

    -- ================================================================
    -- Depth shade: tunnel face brightness
    -- ------------------------------------------------------------------
    -- Brighter near screen edge (large poly_dist), darker near centre.
    -- poly_dist[9:5] spans 0..31.  The +SHADE_FLOOR offset sets the
    -- minimum brightness of the innermost rings.
    --
    -- SHADE_FLOOR = 6  →  inner rings at 6/15 ≈ 40% brightness
    --               1  →  inner rings almost black (old default)
    --              10  →  nearly flat brightness across the whole tunnel
    --
    -- CHANGE the slice poly_dist_s(N downto N-4) to shift where full
    -- brightness (15) is reached:  lower bits → full bright reached sooner.
    -- ================================================================
    process(poly_dist_s)
        variable raw : unsigned(5 downto 0);
    begin
        raw := ('0' & poly_dist_s(9 downto 5)) + 6;  -- floor = 6 → bright core
        if raw >= 16 then depth_shade <= "1111";
        else              depth_shade <= raw(3 downto 0);
        end if;
    end process;

    -- ================================================================
    -- Pixel sector (obstacle face matching)
    -- ------------------------------------------------------------------
    -- 3-bit code identifying which of the 8 octants the pixel is in.
    -- Matches the sector encoding in obstacle_manager.vhd.
    --   bit 2: X-dominant face (|dx| > ady_eff) vs Y-dominant
    --   bit 1: dx ≥ 0  (right half of screen)
    --   bit 0: dy ≥ 0  (bottom half of screen)
    -- ================================================================
    pix_sec(2) <= '1' when ('0' & adx) >= ady_eff else '0';
    pix_sec(1) <= not dx11(10);
    pix_sec(0) <= not dy11(10);

end rtl;
