library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library altera_mf;
use altera_mf.altera_mf_components.all;

--
-- tunnel.vhd
--
-- Input   : Clock and VGA scan position from vga_controller.vhd
--           Rotation transform from trig_rom.vhd
--           Animation counters from RushTunnelFPGA.vhd :
--
--					scroll  — advances tunnel rings each frame
--					hue_cnt — cycles the colour palette
--
--           Ship screen position and game state
--           Obstacle and coin state for 8 slots
-- Output  : RGB pixel with 2-clock pipeline delay
--           Pixel-level collision flags :
--
--					ship_in_obs  — ship centre inside an obstacle
--					ship_on_coin — ship centre overlapping the coin
--
-- Utility : Octagon tunnel renderer.  Computes log-depth perspective,
--           face sectors, obstacle bands, coin ring and ship sprite.
--           Feeds crt_effects.vhd; collision flags feed spaceship.vhd
--

entity tunnel is
    port (
        clk_50            : in  std_logic;
        video_on          : in  std_logic;
        hpos              : in  std_logic_vector(9 downto 0);
        vpos              : in  std_logic_vector(9 downto 0);
        cos_t             : in  signed(8 downto 0);
        sin_t             : in  signed(8 downto 0);
        scroll            : in  std_logic_vector(7 downto 0);
        hue_cnt           : in  unsigned(9 downto 0);
        ship_x            : in  unsigned(9 downto 0);
        ship_y            : in  unsigned(9 downto 0);
        game_state        : in  std_logic_vector(1 downto 0);
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
        obs4_depth        : in  std_logic_vector(7 downto 0);
        obs4_face_sector  : in  std_logic_vector(2 downto 0);
        obs4_active       : in  std_logic;
        obs5_depth        : in  std_logic_vector(7 downto 0);
        obs5_face_sector  : in  std_logic_vector(2 downto 0);
        obs5_active       : in  std_logic;
        obs6_depth        : in  std_logic_vector(7 downto 0);
        obs6_face_sector  : in  std_logic_vector(2 downto 0);
        obs6_active       : in  std_logic;
        obs7_depth        : in  std_logic_vector(7 downto 0);
        obs7_face_sector  : in  std_logic_vector(2 downto 0);
        obs7_active       : in  std_logic;
        ring0_depth       : in  std_logic_vector(7 downto 0);
        ring0_active      : in  std_logic;
        ring0_face_sector : in  std_logic_vector(2 downto 0);
        ring1_depth       : in  std_logic_vector(7 downto 0);
        ring1_active      : in  std_logic;
        red               : out std_logic_vector(3 downto 0);
        green             : out std_logic_vector(3 downto 0);
        blue              : out std_logic_vector(3 downto 0);
        ship_in_obs       : out std_logic;
        ship_on_coin      : out std_logic
    );
end tunnel;

architecture rtl of tunnel is

    -- ── Geometry tuning constants ──────────────────────────────────────────
    constant DEAD_ZONE_RADIUS_SQ   : integer := 144;  -- black hole radius² (px²)
    constant OCT_DIAG_WEIGHT_Q8    : integer := 181;  -- 181/256 ≈ 1/√2 in Q8
    constant OCT_DIAG_WEIGHT_SHIFT : integer := 8;
    constant RING_SPACING_LOG2     : integer := 7;    -- ring period = 2^7 log-depth steps
    constant RING_LINE_WIDTH       : integer := 1;    -- ring thickness (log-depth steps)
    constant SPOKE_MAX_WIDTH_PX    : integer := 3;    -- spoke half-width cap (pixels)
    constant SPOKE_TAPER_FACTOR    : integer := 32;   -- spoke angular narrowing rate
    constant BRIGHTNESS_FLOOR      : integer := 4;    -- minimum brightness for inner rings
    constant OBSTACLE_BAND_WIDTH   : integer := 6;    -- obstacle/coin leading-edge half-width
    constant SHIP_DIAMOND_SIZE     : integer := 10;   -- ship diamond Manhattan radius (px)

    -- ── Colour palette ─────────────────────────────────────────────────────
    -- 4 keyframe colours cycling through hue_cnt; entry [4] copies [0] for wrap.
    type palette_array is array(0 to 4) of integer range 0 to 15;
    constant KF_RED   : palette_array := (15,  15,   4,   0,  15);
    constant KF_GREEN : palette_array := ( 2,   0,   0,  12,   2);
    constant KF_BLUE  : palette_array := (12,  15,  15,  15,  12);

    -- Fixed colours for game objects (4-bit R, G, B)
    constant SHIP_RED_VAL        : integer := 15;
    constant SHIP_GREEN_VAL      : integer := 14;
    constant SHIP_BLUE_VAL       : integer :=  0;
    constant OBS_RED_VAL         : integer := 15;
    constant OBS_GREEN_VAL       : integer :=  4;
    constant OBS_BLUE_VAL        : integer :=  0;
    constant SCORE_RING_RED_VAL  : integer := 15;
    constant SCORE_RING_GREEN_VAL: integer := 12;
    constant SCORE_RING_BLUE_VAL : integer :=  0;

    -- ── Stage-0 pipeline registers ─────────────────────────────────────────
    signal dx_rotated   : signed(11 downto 0) := (others => '0');
    signal dy_rotated   : signed(11 downto 0) := (others => '0');
    signal video_active : std_logic := '0';

    -- Ship sprite ROM signals
    signal address_ImEx : std_logic_vector(11 downto 0) := (others => '0');
    signal color_ImEx   : std_logic_vector(11 downto 0) := (others => '0');
    signal ship_spr_hit : std_logic := '0';

    -- Obstacle and ring signals delayed 1 cycle to align with rotated pixel coords
    signal obs0_depth_d, obs1_depth_d,
           obs2_depth_d, obs3_depth_d,
           obs4_depth_d, obs5_depth_d,
           obs6_depth_d, obs7_depth_d  : std_logic_vector(7 downto 0) := (others => '0');
    signal obs0_face_d,  obs1_face_d,
           obs2_face_d,  obs3_face_d,
           obs4_face_d,  obs5_face_d,
           obs6_face_d,  obs7_face_d   : std_logic_vector(2 downto 0) := (others => '0');
    signal obs0_active_d, obs1_active_d,
           obs2_active_d, obs3_active_d,
           obs4_active_d, obs5_active_d,
           obs6_active_d, obs7_active_d : std_logic := '0';
    signal ring0_depth_d  : std_logic_vector(7 downto 0) := (others => '0');
    signal ring0_active_d : std_logic := '0';
    signal ring0_face_d                   : std_logic_vector(2 downto 0) := "000";

    -- ── Abs() intermediates (VHDL-93: no direct slicing of function results) ──
    signal dx_11bit      : signed(10 downto 0);
    signal dy_11bit      : signed(10 downto 0);
    signal dx_abs_signed : signed(10 downto 0);
    signal dy_abs_signed : signed(10 downto 0);
    signal pix_adx       : unsigned(9 downto 0);
    signal pix_ady       : unsigned(9 downto 0);
    signal pix_ady_eff   : unsigned(10 downto 0);  -- ady × 18/16 (aspect correction)

    -- ── Combinational classify signals ─────────────────────────────────────
    signal oct_dist      : unsigned(9 downto 0);   -- octagonal radius in tunnel space
    signal face_sector   : std_logic_vector(2 downto 0); -- which of the 8 faces (0=right … 7=upper-right)
    signal spoke_dist    : unsigned(11 downto 0);  -- distance from nearest spoke boundary
    signal log_depth     : unsigned(11 downto 0);  -- 12-bit log2(oct_dist)
    signal depth_band    : unsigned(7 downto 0);   -- top 8 bits of log_depth
    signal ring_phase    : unsigned(11 downto 0);  -- scroll-shifted log_depth for ring animation
    signal brightness    : unsigned(3 downto 0);   -- 0..15 brightness from depth
    signal on_ring       : std_logic;
    signal on_spoke      : std_logic;
    signal in_dead_zone  : std_logic;
    signal on_obstacle   : std_logic;
    signal on_obs_edge   : std_logic;
    signal on_score_ring : std_logic;

begin

    -- Ship sprite ROM (50×50 px, 12-bit RGB444, M9K block)
    ROM_ship : altsyncram
        generic map (
            operation_mode                => "ROM",
            width_a                       => 12,
            widthad_a                     => 12,
            numwords_a                    => 2500,
            outdata_reg_a                 => "UNREGISTERED",
            init_file                     => "../spaceship.mif",
            lpm_hint                      => "ENABLE_RUNTIME_MOD=NO",
            lpm_type                      => "altsyncram",
            ram_block_type                => "M9K",
            read_during_write_mode_port_a => "DONT_CARE",
            intended_device_family        => "Cyclone IV E"
        )
        port map (
            clock0    => clk_50,
            address_a => address_ImEx,
            q_a       => color_ImEx
        );

    -- Abs() chain (VHDL-93 step-by-step to allow safe bit-slicing)
    dx_11bit      <= dx_rotated(10 downto 0);
    dy_11bit      <= dy_rotated(10 downto 0);
    dx_abs_signed <= abs(dx_11bit);
    dy_abs_signed <= abs(dy_11bit);
    pix_adx       <= unsigned(dx_abs_signed(9 downto 0));
    pix_ady       <= unsigned(dy_abs_signed(9 downto 0));

    -- Aspect correction: ady_eff = ady + ady/8 = ady × 9/8
    pix_ady_eff   <= resize(pix_ady, 11)
                   + resize(shift_right(pix_ady, 3), 11);

    -- Dead zone: black hole within ~12 px of the vanishing point (Euclidean distance²)
    process(pix_adx, pix_ady)
        variable dist_sq : unsigned(19 downto 0);
    begin
        dist_sq := (pix_adx * pix_adx) + (pix_ady * pix_ady);
        if dist_sq < DEAD_ZONE_RADIUS_SQ then
            in_dead_zone <= '1';
        else
            in_dead_zone <= '0';
        end if;
    end process;

    -- Octagon metric: computes oct_dist and face_sector from aspect-corrected |dx|/|dy|
    process(pix_adx, pix_ady_eff, dx_11bit, dy_11bit)
        variable adx_11          : unsigned(10 downto 0);
        variable oct_sum         : unsigned(10 downto 0);
        variable oct_diag_scaled : unsigned(18 downto 0);
        variable oct_diag_shifted: unsigned(18 downto 0);
        variable oct_diag_dist   : unsigned(10 downto 0);
        variable oct_axial       : unsigned(10 downto 0);
    begin
        adx_11 := resize(pix_adx, 11);
        oct_sum := adx_11 + pix_ady_eff;

        -- Diagonal projection: oct_sum × 181/256 (181 = 128+32+16+4+1)
        oct_diag_scaled :=
              shift_left(resize(oct_sum, 19), 7)
            + shift_left(resize(oct_sum, 19), 5)
            + shift_left(resize(oct_sum, 19), 4)
            + shift_left(resize(oct_sum, 19), 2)
            + resize(oct_sum, 19);
        oct_diag_shifted := shift_right(oct_diag_scaled, OCT_DIAG_WEIGHT_SHIFT);
        oct_diag_dist    := oct_diag_shifted(10 downto 0);

        if adx_11 >= pix_ady_eff then
            oct_axial := adx_11;
        else
            oct_axial := pix_ady_eff;
        end if;

        if oct_diag_dist >= oct_axial then
            -- Diagonal face (45° faces)
            oct_dist <= oct_diag_dist(9 downto 0);
            if    dx_11bit(10) = '0' and dy_11bit(10) = '0' then
                face_sector <= "001";   -- lower-right  (45°)
            elsif dx_11bit(10) = '1' and dy_11bit(10) = '0' then
                face_sector <= "011";   -- lower-left  (135°)
            elsif dx_11bit(10) = '1' and dy_11bit(10) = '1' then
                face_sector <= "101";   -- upper-left  (225°)
            else
                face_sector <= "111";   -- upper-right (315°)
            end if;
        else
            -- Axial face (0° / 90° / 180° / 270°)
            oct_dist <= oct_axial(9 downto 0);
            if adx_11 >= pix_ady_eff then
                if dx_11bit(10) = '0' then
                    face_sector <= "000";   -- right   (0°)
                else
                    face_sector <= "100";   -- left  (180°)
                end if;
            else
                if dy_11bit(10) = '0' then
                    face_sector <= "010";   -- bottom  (90°)
                else
                    face_sector <= "110";   -- top    (270°)
                end if;
            end if;
        end if;
    end process;

    -- Spoke detection: distance from nearest spoke boundary (axial ↔ diagonal transition)
    process(pix_adx, pix_ady_eff)
        variable adx_12bit        : unsigned(11 downto 0);
        variable ady_eff_12bit    : unsigned(11 downto 0);
        variable oct_sum_12bit    : unsigned(11 downto 0);
        variable oct_diag_scaled  : unsigned(18 downto 0);
        variable oct_diag_shifted : unsigned(18 downto 0);
        variable diag_proj_12bit  : unsigned(11 downto 0);
        variable dist_horiz_spoke : unsigned(11 downto 0);
        variable dist_vert_spoke  : unsigned(11 downto 0);
    begin
        adx_12bit     := resize(pix_adx, 12);
        ady_eff_12bit := resize(pix_ady_eff, 12);
        oct_sum_12bit := adx_12bit + ady_eff_12bit;

        oct_diag_scaled :=
              shift_left(resize(oct_sum_12bit, 19), 7)
            + shift_left(resize(oct_sum_12bit, 19), 5)
            + shift_left(resize(oct_sum_12bit, 19), 4)
            + shift_left(resize(oct_sum_12bit, 19), 2)
            + resize(oct_sum_12bit, 19);
        oct_diag_shifted := shift_right(oct_diag_scaled, OCT_DIAG_WEIGHT_SHIFT);
        diag_proj_12bit  := oct_diag_shifted(11 downto 0);

        if adx_12bit >= diag_proj_12bit then
            dist_horiz_spoke := adx_12bit - diag_proj_12bit;
        else
            dist_horiz_spoke := diag_proj_12bit - adx_12bit;
        end if;

        if ady_eff_12bit >= diag_proj_12bit then
            dist_vert_spoke := ady_eff_12bit - diag_proj_12bit;
        else
            dist_vert_spoke := diag_proj_12bit - ady_eff_12bit;
        end if;

        if dist_horiz_spoke <= dist_vert_spoke then
            spoke_dist <= dist_horiz_spoke;
        else
            spoke_dist <= dist_vert_spoke;
        end if;
    end process;

    -- Spoke visibility: angular taper + pixel-width cap + dead-zone exclusion
    on_spoke <= '1'
        when to_integer(spoke_dist) * SPOKE_TAPER_FACTOR < to_integer(oct_dist)
         and to_integer(spoke_dist) < SPOKE_MAX_WIDTH_PX
         and in_dead_zone = '0'
        else '0';

    -- Log2 encoder: converts oct_dist to 12-bit log_depth for equal-spacing rings
    process(oct_dist)
    begin
        if    oct_dist(9) = '1' then
            log_depth <= "1001" & oct_dist(8 downto 1);
        elsif oct_dist(8) = '1' then
            log_depth <= "1000" & oct_dist(7 downto 0);
        elsif oct_dist(7) = '1' then
            log_depth <= "0111" & oct_dist(6 downto 0) & '0';
        elsif oct_dist(6) = '1' then
            log_depth <= "0110" & oct_dist(5 downto 0) & "00";
        elsif oct_dist(5) = '1' then
            log_depth <= "0101" & oct_dist(4 downto 0) & "000";
        elsif oct_dist(4) = '1' then
            log_depth <= "0100" & oct_dist(3 downto 0) & "0000";
        elsif oct_dist(3) = '1' then
            log_depth <= "0011" & oct_dist(2 downto 0) & "00000";
        elsif oct_dist(2) = '1' then
            log_depth <= "0010" & oct_dist(1 downto 0) & "000000";
        elsif oct_dist(1) = '1' then
            log_depth <= "0001" & oct_dist(0) & "0000000";
        else
            log_depth <= (others => '0');
        end if;
    end process;

    -- Top 8 bits of log_depth for obstacle depth comparison
    depth_band <= log_depth(11 downto 4);

    -- Scroll-shifted ring phase: subtracting scroll×8 animates rings toward the player
    ring_phase <= log_depth
                - resize(shift_left(resize(unsigned(scroll), 12), 3), 12);

    -- Ring visibility: low-order bits of ring_phase roll over at the ring period
    on_ring <= '1'
        when to_integer(ring_phase(RING_SPACING_LOG2 - 1 downto 0)) < RING_LINE_WIDTH
         and in_dead_zone = '0'
        else '0';

    -- Depth brightness: maps oct_dist coarsely to 0..15, floored at BRIGHTNESS_FLOOR
    process(oct_dist)
        variable raw_brightness : unsigned(5 downto 0);
    begin
        raw_brightness := ('0' & oct_dist(9 downto 5)) + BRIGHTNESS_FLOOR;
        if raw_brightness >= 16 then
            brightness <= "1111";
        else
            brightness <= raw_brightness(3 downto 0);
        end if;
    end process;

    -- Obstacle depth comparisons: pixel is in obstacle fill (depth_band ≤ obs_depth, same face)
    process(depth_band, face_sector, in_dead_zone,
            obs0_depth_d, obs0_face_d, obs0_active_d,
            obs1_depth_d, obs1_face_d, obs1_active_d,
            obs2_depth_d, obs2_face_d, obs2_active_d,
            obs3_depth_d, obs3_face_d, obs3_active_d,
            obs4_depth_d, obs4_face_d, obs4_active_d,
            obs5_depth_d, obs5_face_d, obs5_active_d,
            obs6_depth_d, obs6_face_d, obs6_active_d,
            obs7_depth_d, obs7_face_d, obs7_active_d)
        variable depth_diff0, depth_diff1,
                 depth_diff2, depth_diff3,
                 depth_diff4, depth_diff5,
                 depth_diff6, depth_diff7 : signed(8 downto 0);
    begin
        depth_diff0 := signed('0' & depth_band) - signed('0' & unsigned(obs0_depth_d));
        depth_diff1 := signed('0' & depth_band) - signed('0' & unsigned(obs1_depth_d));
        depth_diff2 := signed('0' & depth_band) - signed('0' & unsigned(obs2_depth_d));
        depth_diff3 := signed('0' & depth_band) - signed('0' & unsigned(obs3_depth_d));
        depth_diff4 := signed('0' & depth_band) - signed('0' & unsigned(obs4_depth_d));
        depth_diff5 := signed('0' & depth_band) - signed('0' & unsigned(obs5_depth_d));
        depth_diff6 := signed('0' & depth_band) - signed('0' & unsigned(obs6_depth_d));
        depth_diff7 := signed('0' & depth_band) - signed('0' & unsigned(obs7_depth_d));

        on_obstacle <= '0';
        on_obs_edge <= '0';

        -- Obstacle fill zone (depth_band ≤ obs_depth, same face, outside dead zone)
        if obs0_active_d = '1' and depth_diff0 <= 0
           and face_sector = obs0_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs1_active_d = '1' and depth_diff1 <= 0
           and face_sector = obs1_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs2_active_d = '1' and depth_diff2 <= 0
           and face_sector = obs2_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs3_active_d = '1' and depth_diff3 <= 0
           and face_sector = obs3_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs4_active_d = '1' and depth_diff4 <= 0
           and face_sector = obs4_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs5_active_d = '1' and depth_diff5 <= 0
           and face_sector = obs5_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs6_active_d = '1' and depth_diff6 <= 0
           and face_sector = obs6_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        if obs7_active_d = '1' and depth_diff7 <= 0
           and face_sector = obs7_face_d and in_dead_zone = '0'
        then on_obstacle <= '1'; end if;

        -- Obstacle leading-edge band (bright ring segment at front face)
        if obs0_active_d = '1'
           and depth_diff0 >= -OBSTACLE_BAND_WIDTH and depth_diff0 <= 0
           and face_sector = obs0_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs1_active_d = '1'
           and depth_diff1 >= -OBSTACLE_BAND_WIDTH and depth_diff1 <= 0
           and face_sector = obs1_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs2_active_d = '1'
           and depth_diff2 >= -OBSTACLE_BAND_WIDTH and depth_diff2 <= 0
           and face_sector = obs2_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs3_active_d = '1'
           and depth_diff3 >= -OBSTACLE_BAND_WIDTH and depth_diff3 <= 0
           and face_sector = obs3_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs4_active_d = '1'
           and depth_diff4 >= -OBSTACLE_BAND_WIDTH and depth_diff4 <= 0
           and face_sector = obs4_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs5_active_d = '1'
           and depth_diff5 >= -OBSTACLE_BAND_WIDTH and depth_diff5 <= 0
           and face_sector = obs5_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs6_active_d = '1'
           and depth_diff6 >= -OBSTACLE_BAND_WIDTH and depth_diff6 <= 0
           and face_sector = obs6_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;

        if obs7_active_d = '1'
           and depth_diff7 >= -OBSTACLE_BAND_WIDTH and depth_diff7 <= 0
           and face_sector = obs7_face_d and in_dead_zone = '0'
        then on_obs_edge <= '1'; end if;
    end process;

    -- Coin depth + face comparison (thin band check, same face as ring0)
    process(depth_band, face_sector, in_dead_zone,
            ring0_depth_d, ring0_active_d, ring0_face_d)
        variable ring_diff0 : signed(8 downto 0);
    begin
        ring_diff0 := signed('0' & depth_band) - signed('0' & unsigned(ring0_depth_d));

        on_score_ring <= '0';
        if ring0_active_d = '1'
           and ring_diff0 > -OBSTACLE_BAND_WIDTH and ring_diff0 < OBSTACLE_BAND_WIDTH
           and face_sector = ring0_face_d
           and in_dead_zone = '0'
        then on_score_ring <= '1'; end if;
    end process;

    -- Stage 0: coordinate rotation + ship hit + pipeline obstacle/ring signals
    process(clk_50)
        variable dx_centre, dy_centre     : signed(10 downto 0);
        variable dx_cos, dx_sin           : signed(19 downto 0);
        variable dy_cos, dy_sin           : signed(19 downto 0);
        variable rotated_x, rotated_y     : signed(20 downto 0);
        variable ship_dx, ship_dy         : signed(10 downto 0);
        variable ship_dx_abs, ship_dy_abs : signed(10 downto 0);
        variable ship_dist                : unsigned(10 downto 0);
        variable sl_v, sr_v, st_v, sb_v   : unsigned(9 downto 0);
        variable lx_v, ly_v               : unsigned(9 downto 0);
        variable pxn                      : unsigned(9 downto 0);
    begin
        if rising_edge(clk_50) then

            -- Centre-offset pixel coordinates (vanishing point = 400, 300)
            dx_centre := signed('0' & hpos) - to_signed(400, 11);
            dy_centre := signed('0' & vpos) - to_signed(300, 11);

            -- Rotation matrix: dx' = dx·cos + dy·sin,  dy' = −dx·sin + dy·cos
            dx_cos := dx_centre * cos_t;
            dx_sin := dx_centre * sin_t;
            dy_cos := dy_centre * cos_t;
            dy_sin := dy_centre * sin_t;

            rotated_x :=  resize(dx_cos, 21) + resize(dy_sin, 21);
            rotated_y := -resize(dx_sin, 21) + resize(dy_cos, 21);

            -- Divide by 256 to normalise cos/sin (range ±255 ≈ ±1.0)
            dx_rotated <= resize(shift_right(rotated_x, 8), 12);
            dy_rotated <= resize(shift_right(rotated_y, 8), 12);

            video_active <= video_on;

            -- Ship diamond hit check (screen space, Manhattan distance)
            ship_dx     := signed('0' & hpos) - signed('0' & std_logic_vector(ship_x));
            ship_dy     := signed('0' & vpos) - signed('0' & std_logic_vector(ship_y));
            ship_dx_abs := abs(ship_dx);
            ship_dy_abs := abs(ship_dy);
            ship_dist   := ('0' & unsigned(ship_dx_abs(9 downto 0)))
                         + ('0' & unsigned(ship_dy_abs(9 downto 0)));
            -- Ship sprite ROM: look-ahead addressing (pxn = hpos+1) so color_ImEx
            -- is ready at Stage 1 for the current pixel
            sl_v := ship_x - to_unsigned(25, 10);
            sr_v := ship_x + to_unsigned(24, 10);
            st_v := ship_y - to_unsigned(25, 10);
            sb_v := ship_y + to_unsigned(24, 10);
            pxn  := unsigned(hpos) + 1;

            if pxn >= sl_v and pxn <= sr_v
               and unsigned(vpos) >= st_v and unsigned(vpos) <= sb_v
            then
                lx_v := pxn - sl_v;
                ly_v := unsigned(vpos) - st_v;
                address_ImEx <= std_logic_vector(
                    to_unsigned(to_integer(ly_v) * 50 + to_integer(lx_v), 12));
            else
                address_ImEx <= (others => '0');
            end if;

            if unsigned(hpos) >= sl_v and unsigned(hpos) <= sr_v
               and unsigned(vpos) >= st_v and unsigned(vpos) <= sb_v
            then
                ship_spr_hit <= '1';
            else
                ship_spr_hit <= '0';
            end if;

            -- Pipeline obstacle and ring signals (1 cycle delay to match rotated coords)
            obs0_depth_d  <= obs0_depth;   obs0_face_d   <= obs0_face_sector;   obs0_active_d <= obs0_active;
            obs1_depth_d  <= obs1_depth;   obs1_face_d   <= obs1_face_sector;   obs1_active_d <= obs1_active;
            obs2_depth_d  <= obs2_depth;   obs2_face_d   <= obs2_face_sector;   obs2_active_d <= obs2_active;
            obs3_depth_d  <= obs3_depth;   obs3_face_d   <= obs3_face_sector;   obs3_active_d <= obs3_active;
            obs4_depth_d  <= obs4_depth;   obs4_face_d   <= obs4_face_sector;   obs4_active_d <= obs4_active;
            obs5_depth_d  <= obs5_depth;   obs5_face_d   <= obs5_face_sector;   obs5_active_d <= obs5_active;
            obs6_depth_d  <= obs6_depth;   obs6_face_d   <= obs6_face_sector;   obs6_active_d <= obs6_active;
            obs7_depth_d  <= obs7_depth;   obs7_face_d   <= obs7_face_sector;   obs7_active_d <= obs7_active;
            ring0_depth_d  <= ring0_depth;   ring0_active_d <= ring0_active;   ring0_face_d <= ring0_face_sector;

        end if;
    end process;

    -- Stage 1: colour priority mux (registered output)
    -- Priority order, lowest to highest (later assignment wins):
    --   background → obstacle fill → ring/spoke → dead zone →
    --   coin → obstacle edge → ship sprite → blanking
    process(clk_50)
        variable hue_pair            : integer range 0 to 3;
        variable hue_blend           : integer range 0 to 15;
        variable col_a_r, col_a_g, col_a_b : integer range 0 to 15;
        variable col_b_r, col_b_g, col_b_b : integer range 0 to 15;
        variable pal_red, pal_grn, pal_blu  : unsigned(3 downto 0);
        variable shaded_r, shaded_g, shaded_b : unsigned(7 downto 0);
        variable out_r, out_g, out_b : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk_50) then

            -- Interpolate palette colour from hue_cnt (pair × 16 blend steps)
            hue_pair  := to_integer(hue_cnt(9 downto 8));
            hue_blend := to_integer(hue_cnt(7 downto 4));
            col_a_r := KF_RED(hue_pair);    col_b_r := KF_RED(hue_pair + 1);
            col_a_g := KF_GREEN(hue_pair);  col_b_g := KF_GREEN(hue_pair + 1);
            col_a_b := KF_BLUE(hue_pair);   col_b_b := KF_BLUE(hue_pair + 1);
            pal_red := to_unsigned(col_a_r + ((col_b_r - col_a_r) * hue_blend) / 16, 4);
            pal_grn := to_unsigned(col_a_g + ((col_b_g - col_a_g) * hue_blend) / 16, 4);
            pal_blu := to_unsigned(col_a_b + ((col_b_b - col_a_b) * hue_blend) / 16, 4);

            -- Depth-shade the palette (4-bit × 4-bit → take top nibble)
            shaded_r := pal_red * brightness;
            shaded_g := pal_grn * brightness;
            shaded_b := pal_blu * brightness;

            -- 1. Near-black background
            out_r := "0001"; out_g := "0000"; out_b := "0001";

            -- 2. Obstacle fill: very dim rust tint (spokes remain visible through it)
            if on_obstacle = '1' then
                out_r := "0011"; out_g := "0000"; out_b := "0000";
            end if;

            -- 3. Tunnel ring or spoke (depth-shaded palette colour)
            if on_ring = '1' or on_spoke = '1' then
                out_r := std_logic_vector(shaded_r(7 downto 4));
                out_g := std_logic_vector(shaded_g(7 downto 4));
                out_b := std_logic_vector(shaded_b(7 downto 4));
            end if;

            -- 4. Dead zone: solid black at vanishing point
            if in_dead_zone = '1' then
                out_r := "0000"; out_g := "0000"; out_b := "0000";
            end if;

            -- 5. Scoring coin (gold)
            if on_score_ring = '1' then
                out_r := std_logic_vector(to_unsigned(SCORE_RING_RED_VAL,   4));
                out_g := std_logic_vector(to_unsigned(SCORE_RING_GREEN_VAL, 4));
                out_b := std_logic_vector(to_unsigned(SCORE_RING_BLUE_VAL,  4));
            end if;

            -- 6. Obstacle leading edge: bright depth-shaded orange ring segment
            if on_obs_edge = '1' then
                if to_integer(brightness) >= 6 then
                    shaded_r := to_unsigned(OBS_RED_VAL,   4) * brightness;
                    shaded_g := to_unsigned(OBS_GREEN_VAL, 4) * brightness;
                    shaded_b := to_unsigned(OBS_BLUE_VAL,  4) * brightness;
                else
                    shaded_r := to_unsigned(OBS_RED_VAL,   4) * to_unsigned(6, 4);
                    shaded_g := to_unsigned(OBS_GREEN_VAL, 4) * to_unsigned(6, 4);
                    shaded_b := to_unsigned(OBS_BLUE_VAL,  4) * to_unsigned(6, 4);
                end if;
                out_r := std_logic_vector(shaded_r(7 downto 4));
                out_g := std_logic_vector(shaded_g(7 downto 4));
                out_b := std_logic_vector(shaded_b(7 downto 4));
            end if;

            -- 7. Ship sprite from ROM (PLAYING state only; 0x000 = transparent)
            if ship_spr_hit = '1' and game_state = "01" then
                if color_ImEx /= "000000000000" then
                    out_r := color_ImEx(11 downto 8);
                    out_g := color_ImEx(7  downto 4);
                    out_b := color_ImEx(3  downto 0);
                end if;
            end if;

            -- 8. Blanking: black outside active video area
            if video_active = '0' then
                out_r := "0000"; out_g := "0000"; out_b := "0000";
            end if;

            red   <= out_r;
            green <= out_g;
            blue  <= out_b;

        end if;
    end process;

    -- Ship pixel collision: same oct transform applied to ship centre pixel
    process(clk_50)
        variable sx_v, sy_v       : signed(10 downto 0);
        variable sx_cos_v         : signed(19 downto 0);
        variable sx_sin_v         : signed(19 downto 0);
        variable sy_cos_v         : signed(19 downto 0);
        variable sy_sin_v         : signed(19 downto 0);
        variable rot_x_v          : signed(20 downto 0);
        variable rot_y_v          : signed(20 downto 0);
        variable drx_v, dry_v     : signed(11 downto 0);
        variable drx_11, dry_11   : signed(10 downto 0);
        variable drx_abs, dry_abs : signed(10 downto 0);
        variable adx_v            : unsigned(9 downto 0);
        variable ady_v            : unsigned(9 downto 0);
        variable ady_eff_v        : unsigned(10 downto 0);
        variable adx_11_v         : unsigned(10 downto 0);
        variable oct_s_v          : unsigned(10 downto 0);
        variable oct_ds_v         : unsigned(18 downto 0);
        variable oct_dsh_v        : unsigned(18 downto 0);
        variable oct_dd_v         : unsigned(10 downto 0);
        variable oct_ax_v         : unsigned(10 downto 0);
        variable oct_d_v          : unsigned(9 downto 0);
        variable fs_v             : std_logic_vector(2 downto 0);
        variable ld_v             : unsigned(11 downto 0);
        variable db_v             : unsigned(7 downto 0);
        variable dd_v             : signed(8 downto 0);
        variable in_obs_v         : std_logic;
        variable on_coin_v        : std_logic;
    begin
        if rising_edge(clk_50) then

            -- Centre-offset ship coordinates
            sx_v := signed('0' & std_logic_vector(ship_x)) - to_signed(400, 11);
            sy_v := signed('0' & std_logic_vector(ship_y)) - to_signed(300, 11);

            -- Rotate ship pixel into tunnel frame (same matrix as Stage 0)
            sx_cos_v := sx_v * cos_t;
            sx_sin_v := sx_v * sin_t;
            sy_cos_v := sy_v * cos_t;
            sy_sin_v := sy_v * sin_t;
            rot_x_v  :=  resize(sx_cos_v, 21) + resize(sy_sin_v, 21);
            rot_y_v  := -resize(sx_sin_v, 21) + resize(sy_cos_v, 21);
            drx_v    := resize(shift_right(rot_x_v, 8), 12);
            dry_v    := resize(shift_right(rot_y_v, 8), 12);
            drx_11   := drx_v(10 downto 0);
            dry_11   := dry_v(10 downto 0);

            -- Abs (VHDL-93: no direct slicing of abs())
            drx_abs  := abs(drx_11);
            dry_abs  := abs(dry_11);
            adx_v    := unsigned(drx_abs(9 downto 0));
            ady_v    := unsigned(dry_abs(9 downto 0));

            -- Aspect correction
            ady_eff_v := resize(ady_v, 11)
                       + resize(shift_right(ady_v, 3), 11);

            -- Octagon metric (mirrors main oct_dist process)
            adx_11_v  := resize(adx_v, 11);
            oct_s_v   := adx_11_v + ady_eff_v;
            oct_ds_v  :=   shift_left(resize(oct_s_v, 19), 7)
                         + shift_left(resize(oct_s_v, 19), 5)
                         + shift_left(resize(oct_s_v, 19), 4)
                         + shift_left(resize(oct_s_v, 19), 2)
                         + resize(oct_s_v, 19);
            oct_dsh_v := shift_right(oct_ds_v, OCT_DIAG_WEIGHT_SHIFT);
            oct_dd_v  := oct_dsh_v(10 downto 0);

            if adx_11_v >= ady_eff_v then oct_ax_v := adx_11_v;
            else                          oct_ax_v := ady_eff_v;
            end if;

            if oct_dd_v >= oct_ax_v then
                oct_d_v := oct_dd_v(9 downto 0);
                if    drx_11(10) = '0' and dry_11(10) = '0' then fs_v := "001";
                elsif drx_11(10) = '1' and dry_11(10) = '0' then fs_v := "011";
                elsif drx_11(10) = '1' and dry_11(10) = '1' then fs_v := "101";
                else                                              fs_v := "111";
                end if;
            else
                oct_d_v := oct_ax_v(9 downto 0);
                if adx_11_v >= ady_eff_v then
                    if drx_11(10) = '0' then fs_v := "000";
                    else                     fs_v := "100";
                    end if;
                else
                    if dry_11(10) = '0' then fs_v := "010";
                    else                     fs_v := "110";
                    end if;
                end if;
            end if;

            -- Log2 to depth_band (mirrors main log_depth process)
            if    oct_d_v(9) = '1' then ld_v := "1001" & oct_d_v(8 downto 1);
            elsif oct_d_v(8) = '1' then ld_v := "1000" & oct_d_v(7 downto 0);
            elsif oct_d_v(7) = '1' then ld_v := "0111" & oct_d_v(6 downto 0) & '0';
            elsif oct_d_v(6) = '1' then ld_v := "0110" & oct_d_v(5 downto 0) & "00";
            elsif oct_d_v(5) = '1' then ld_v := "0101" & oct_d_v(4 downto 0) & "000";
            elsif oct_d_v(4) = '1' then ld_v := "0100" & oct_d_v(3 downto 0) & "0000";
            elsif oct_d_v(3) = '1' then ld_v := "0011" & oct_d_v(2 downto 0) & "00000";
            elsif oct_d_v(2) = '1' then ld_v := "0010" & oct_d_v(1 downto 0) & "000000";
            elsif oct_d_v(1) = '1' then ld_v := "0001" & oct_d_v(0) & "0000000";
            else                         ld_v := (others => '0');
            end if;
            db_v := ld_v(11 downto 4);

            -- Obstacle fill-zone check for ship centre (db_v ≤ obs_depth, same face)
            in_obs_v := '0';
            if obs0_active = '1' and fs_v = obs0_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs0_depth))
            then in_obs_v := '1'; end if;

            if obs1_active = '1' and fs_v = obs1_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs1_depth))
            then in_obs_v := '1'; end if;

            if obs2_active = '1' and fs_v = obs2_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs2_depth))
            then in_obs_v := '1'; end if;

            if obs3_active = '1' and fs_v = obs3_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs3_depth))
            then in_obs_v := '1'; end if;

            if obs4_active = '1' and fs_v = obs4_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs4_depth))
            then in_obs_v := '1'; end if;

            if obs5_active = '1' and fs_v = obs5_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs5_depth))
            then in_obs_v := '1'; end if;

            if obs6_active = '1' and fs_v = obs6_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs6_depth))
            then in_obs_v := '1'; end if;

            if obs7_active = '1' and fs_v = obs7_face_sector
               and signed('0' & db_v) <= signed('0' & unsigned(obs7_depth))
            then in_obs_v := '1'; end if;

            -- Coin band check for ship centre (|db_v − coin_depth| < BAND_WIDTH, same face)
            on_coin_v := '0';
            dd_v := signed('0' & db_v) - signed('0' & unsigned(ring0_depth));
            if ring0_active = '1' and fs_v = ring0_face_sector
               and to_integer(dd_v) > -OBSTACLE_BAND_WIDTH
               and to_integer(dd_v) < OBSTACLE_BAND_WIDTH
            then on_coin_v := '1'; end if;

            ship_in_obs  <= in_obs_v;
            ship_on_coin <= on_coin_v;

        end if;
    end process;

end rtl;
