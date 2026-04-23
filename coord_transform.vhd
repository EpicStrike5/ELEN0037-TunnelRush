library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- coord_transform.vhd
-- ================================================================
-- Pipeline Stage 0: rotation + ship-sprite hit detection.
-- All outputs are registered on the rising edge of clk_50.
--
-- ROTATION:
--   Centre pixel on (400, 240) then apply rotation matrix:
--     x' =  x*cos_t + y*sin_t   (>> 8 normalises cos/sin ≈ 1.0)
--     y' = -x*sin_t + y*cos_t
--   cos_t, sin_t are signed(8:0), range -255..+255  (255 ≈ 1.0).
--   Result is signed(11:0) — max magnitude ≈ 538, comfortably fits.
--
-- SHIP HIT (diamond metric, screen space):
--   |pixel_x − ship_x| + |pixel_y − ship_y| < SHIP_SIZE
--   Checked before rotation so the ship sprite sits at its true
--   screen position regardless of tunnel angle.
--
-- video_on is pipelined one stage to stay synchronous with the
-- rotated coordinates.
-- ================================================================

entity coord_transform is
    port (
        clk_50   : in  std_logic;
        -- VGA scan position
        hpos     : in  std_logic_vector(9 downto 0);
        vpos     : in  std_logic_vector(9 downto 0);
        -- Tunnel rotation trig (from trig_rom)
        cos_t    : in  signed(8 downto 0);
        sin_t    : in  signed(8 downto 0);
        -- Ship screen position
        ship_x   : in  unsigned(9 downto 0);
        ship_y   : in  unsigned(9 downto 0);
        -- Video gate
        video_on : in  std_logic;
        -- Registered Stage-0 outputs
        dx_rot   : out signed(11 downto 0);
        dy_rot   : out signed(11 downto 0);
        on_ship  : out std_logic;
        vid_out  : out std_logic
    );
end coord_transform;

architecture rtl of coord_transform is

    constant SHIP_SIZE : integer := 10;   -- diamond half-radius in pixels

    -- Centre-offset pixel coords (11-bit signed, range -400..+400)
    signal dx_s0, dy_s0     : signed(10 downto 0);

    -- Four products for rotation matrix
    -- signed(10:0) × signed(8:0) = signed(19:0)
    signal prod_xc, prod_xs : signed(19 downto 0);
    signal prod_yc, prod_ys : signed(19 downto 0);

    -- Rotated coordinates (combinational)
    signal dx_rot_c, dy_rot_c : signed(11 downto 0);

    -- Ship distance (screen space)
    signal ship_dx, ship_dy         : signed(10 downto 0);
    signal ship_dx_abs, ship_dy_abs : signed(10 downto 0);
    signal ship_adx, ship_ady       : unsigned(9 downto 0);
    signal on_ship_c                : std_logic;

begin

    -- ================================================================
    -- Centre-offset coordinates
    -- ================================================================
    dx_s0 <= signed('0' & hpos) - to_signed(400, 11);
    dy_s0 <= signed('0' & vpos) - to_signed(240, 11);

    -- ================================================================
    -- Rotation matrix products
    -- ================================================================
    prod_xc <= dx_s0 * cos_t;
    prod_xs <= dx_s0 * sin_t;
    prod_yc <= dy_s0 * cos_t;
    prod_ys <= dy_s0 * sin_t;

    -- x' =  x*cos + y*sin   |   y' = -x*sin + y*cos   (/ 256)
    dx_rot_c <= resize(shift_right(prod_xc + prod_ys,  8), 12);
    dy_rot_c <= resize(shift_right(-prod_xs + prod_yc, 8), 12);

    -- ================================================================
    -- Ship diamond hit (screen space, before rotation)
    -- ================================================================
    ship_dx     <= signed('0' & hpos)                    - signed('0' & std_logic_vector(ship_x));
    ship_dy     <= signed('0' & vpos)                    - signed('0' & std_logic_vector(ship_y));
    ship_dx_abs <= abs(ship_dx);
    ship_dy_abs <= abs(ship_dy);
    ship_adx    <= unsigned(ship_dx_abs(9 downto 0));
    ship_ady    <= unsigned(ship_dy_abs(9 downto 0));
    on_ship_c   <= '1' when (('0' & ship_adx) + ('0' & ship_ady) < SHIP_SIZE) else '0';

    -- ================================================================
    -- Stage 0 register
    -- ================================================================
    process(clk_50)
    begin
        if rising_edge(clk_50) then
            dx_rot  <= dx_rot_c;
            dy_rot  <= dy_rot_c;
            on_ship <= on_ship_c;
            vid_out <= video_on;
        end if;
    end process;

end rtl;
