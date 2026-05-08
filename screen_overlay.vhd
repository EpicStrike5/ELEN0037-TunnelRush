library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.game_over_font.all;    -- GAME_OVER constant array
use work.press_start_font.all;  -- FONT constant array
use work.title_font.all;        -- TITLE_FONT constant array ("TUNNEL RUSH")

-- ================================================================
-- screen_overlay.vhd
-- ================================================================
-- Last stage in the rendering pipeline.  Placed AFTER crt_effects
-- so the tunnel image (already CRT-processed) can be passed through
-- or replaced by the UI screens.
--
-- PIPELINE ALIGNMENT
--   tunnel.vhd   : 2-clock pipeline (stage 0 + stage 1)
--   crt_effects  : 1-clock pipeline (delays hpos/vpos internally)
--   screen_overlay: 1-clock pipeline (delays hpos/vpos 3 stages here)
--   Total        : 4-clock delay from hpos/vpos input to VGA pins
--
--   At rising edge T this module receives crt_r/g/b which correspond
--   to the pixel that was at hpos[T-3]/vpos[T-3].
--   hpos_d3 (the OLD value read inside the process) = hpos[T-3]. ✓
--
-- RENDERING MODES (selected by game_state)
--   "00" IDLE      : black background + coloured particle squares
--                    + neon-cyan "TUNNEL RUSH" title
--                    + white "PRESS START" text at bottom.
--   "01" PLAYING   : tunnel + CRT effects pass straight through
--                    + gold score (top-left).
--   "10" GAME OVER : black background + red "GAME OVER" text that
--                    bounces around the screen like a DVD logo
--                    + amber "BEST XXX" high score below it.
-- ================================================================

entity screen_overlay is
    port (
        clk_50      : in  std_logic;
        frame_tick  : in  std_logic;   -- 1 pulse per frame — drives bounce update
        -- Scan position from VGA controller (undelayed)
        hpos        : in  std_logic_vector(9 downto 0);
        vpos        : in  std_logic_vector(9 downto 0);
        video_on    : in  std_logic;
        -- Game state from spaceship FSM
        game_state  : in  std_logic_vector(1 downto 0);
        -- CRT-processed tunnel output (arrives 3 clocks after hpos/vpos)
        crt_r       : in  std_logic_vector(3 downto 0);
        crt_g       : in  std_logic_vector(3 downto 0);
        crt_b       : in  std_logic_vector(3 downto 0);
        -- Start-screen particle data from start_screen entity.
        -- Particle i occupies bits width*(i+1)-1 downto width*i.
        sq_px       : in  std_logic_vector(65 downto 0);  -- 6 × 11-bit signed centre X
        sq_py       : in  std_logic_vector(65 downto 0);  -- 6 × 11-bit signed centre Y
        sq_half     : in  std_logic_vector(47 downto 0);  -- 6 × 8-bit half-size
        sq_active   : in  std_logic_vector(5  downto 0);  -- 6 × 1-bit active flag
        sq_col      : in  std_logic_vector(17 downto 0);  -- 6 × 3-bit colour index
        -- Score (12-bit BCD: bits[11:8]=hundreds, [7:4]=tens, [3:0]=ones)
        score       : in  std_logic_vector(11 downto 0);
        -- VGA output (4-clock delay from hpos/vpos input)
        red_out     : out std_logic_vector(3 downto 0);
        green_out   : out std_logic_vector(3 downto 0);
        blue_out    : out std_logic_vector(3 downto 0)
    );
end screen_overlay;

architecture rtl of screen_overlay is

    -- ── Position delay chain ──────────────────────────────────────────
    -- Three registers align hpos/vpos with the CRT output.
    signal hpos_d1, hpos_d2, hpos_d3 : unsigned(9 downto 0) := (others => '0');
    signal vpos_d1, vpos_d2, vpos_d3 : unsigned(9 downto 0) := (others => '0');
    signal vdo_d1,  vdo_d2,  vdo_d3  : std_logic := '0';

    -- ── "TUNNEL RUSH" big title (state IDLE) ─────────────────────────
    -- 11 chars × 6 px/char × scale 8 = 528 px wide
    constant TITLE_SCALE  : integer := 8;
    constant TITLE_CHARS  : integer := 11;
    constant TITLE_CHAR_W : integer := 6;
    constant TITLE_CHAR_H : integer := 7;
    constant TITLE_W      : integer := TITLE_CHARS * TITLE_CHAR_W * TITLE_SCALE; -- 528
    constant TITLE_H      : integer := TITLE_CHAR_H * TITLE_SCALE;               --  56
    constant TITLE_X      : integer := (800 - TITLE_W) / 2;                      -- 136
    constant TITLE_Y      : integer := 180;   -- vertical position on start screen

    -- ── "PRESS START" layout (state IDLE) ────────────────────────────
    -- Total text width = NUM_CHARS × CHAR_W × SCALE = 11 × 6 × 4 = 264 px
    -- TEXT_X = (800 - 264) / 2 = 268  (horizontally centred)
    constant SCALE     : integer := 4;
    constant TEXT_X    : integer := 268;
    constant TEXT_Y    : integer := 500;  -- bottom of screen, below title
    constant CHAR_W    : integer := 6;    -- glyph width (5) + 1 px inter-char gap
    constant CHAR_H    : integer := 7;    -- glyph height in font pixels
    constant NUM_CHARS : integer := 11;

    -- ── "GAME OVER" glyph parameters ─────────────────────────────────
    -- 9 chars × 6 px/char × scale 6 = 324 px wide, 7 × 6 = 42 px tall
    constant GO_SCALE  : integer := 6;
    constant GO_CHARS  : integer := 9;
    constant GO_CHAR_W : integer := 6;
    constant GO_CHAR_H : integer := 7;

    -- ── "BEST XXX" label displayed below bouncing GAME OVER ──────────
    -- 8 chars (B,E,S,T,space,d2,d1,d0) × 6 × scale 4 = 192 px wide
    constant BEST_SCALE  : integer := 4;
    constant BEST_CHARS  : integer := 8;   -- 4 label + 1 space + 3 digits
    constant BEST_CHAR_W : integer := 6;   -- glyph width (5) + 1 px inter-char gap
    constant BEST_CHAR_H : integer := 7;   -- glyph height in font pixels
    constant BEST_GAP    : integer := 10;  -- vertical gap between GAME OVER and BEST
    constant BEST_W      : integer := BEST_CHARS * BEST_CHAR_W * BEST_SCALE;   -- 192
    constant BEST_H      : integer := BEST_CHAR_H * BEST_SCALE;                --  28

    -- ── Bouncing unit bounding box ────────────────────────────────────
    -- Total width  = GAME OVER width  = GO_CHARS × GO_CHAR_W × GO_SCALE = 324
    -- Total height = GO height + gap + BEST height                       =  80
    constant UNIT_W       : integer := GO_CHARS * GO_CHAR_W * GO_SCALE;            -- 324
    constant UNIT_H       : integer := GO_CHAR_H * GO_SCALE + BEST_GAP + BEST_H;   --  80
    constant BOUNCE_X_MAX : integer := 800 - UNIT_W;   -- 476
    constant BOUNCE_Y_MAX : integer := 600 - UNIT_H;   -- 520

    -- Offset of BEST label within the bouncing unit (centred under GAME OVER)
    constant BEST_X_OFF : integer := (UNIT_W - BEST_W) / 2;           --  66
    constant BEST_Y_OFF : integer := GO_CHAR_H * GO_SCALE + BEST_GAP; --  52

    -- ── Bounce position / direction ───────────────────────────────────
    -- go_bx_r / go_by_r: top-left corner of the bouncing GAME OVER text.
    -- Initialised near centre; speeds: 2 px/frame ≈ 144 px/s @ 72 Hz.
    signal go_bx_r : integer range 0 to 800 := BOUNCE_X_MAX / 2;  -- ~238
    signal go_by_r : integer range 0 to 600 := BOUNCE_Y_MAX / 2;  -- ~260
    signal go_dx_r : std_logic := '1';  -- '1' = moving right, '0' = moving left
    signal go_dy_r : std_logic := '1';  -- '1' = moving down,  '0' = moving up

    -- ── High score (BCD, same format as score port) ───────────────────
    signal high_score_bcd : std_logic_vector(11 downto 0) := (others => '0');

    -- ── Score display (state PLAYING) ────────────────────────────────
    -- 3 BCD digits, top-left corner.  Each digit is a 5×7 pixel glyph
    -- scaled by SCORE_SCALE and separated by 1 px gap.
    constant SCORE_SCALE   : integer := 3;   -- pixel scale factor
    constant SCORE_CHAR_W  : integer := 6;   -- glyph width (5) + 1 px gap
    constant SCORE_CHAR_H  : integer := 7;   -- glyph height in font pixels
    constant SCORE_X       : integer := 10;  -- left edge of score display
    constant SCORE_Y       : integer := 8;   -- top edge of score display

    -- ── 5×7 bitmap digit font (shared by score and high-score display) ─
    type digit_row_t  is array (0 to 4) of std_logic;
    type digit_rows_t is array (0 to 6) of digit_row_t;
    type digit_font_t is array (0 to 9) of digit_rows_t;
    constant DIGIT_FONT : digit_font_t := (
        -- 0
        (('0','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('1','0','0','0','1'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0')),
        -- 1
        (('0','0','1','0','0'), ('0','1','1','0','0'), ('0','0','1','0','0'),
         ('0','0','1','0','0'), ('0','0','1','0','0'), ('0','0','1','0','0'),
         ('0','1','1','1','0')),
        -- 2
        (('0','1','1','1','0'), ('1','0','0','0','1'), ('0','0','0','0','1'),
         ('0','0','1','1','0'), ('0','1','0','0','0'), ('1','0','0','0','0'),
         ('1','1','1','1','1')),
        -- 3
        (('0','1','1','1','0'), ('1','0','0','0','1'), ('0','0','0','0','1'),
         ('0','0','1','1','0'), ('0','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0')),
        -- 4
        (('0','0','0','1','0'), ('0','0','1','1','0'), ('0','1','0','1','0'),
         ('1','0','0','1','0'), ('1','1','1','1','1'), ('0','0','0','1','0'),
         ('0','0','0','1','0')),
        -- 5
        (('1','1','1','1','1'), ('1','0','0','0','0'), ('1','1','1','1','0'),
         ('0','0','0','0','1'), ('0','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0')),
        -- 6
        (('0','1','1','1','0'), ('1','0','0','0','0'), ('1','0','0','0','0'),
         ('1','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0')),
        -- 7
        (('1','1','1','1','1'), ('0','0','0','0','1'), ('0','0','0','1','0'),
         ('0','0','1','0','0'), ('0','0','1','0','0'), ('0','0','1','0','0'),
         ('0','0','1','0','0')),
        -- 8
        (('0','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0')),
        -- 9
        (('0','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','1'), ('0','0','0','0','1'), ('1','0','0','0','1'),
         ('0','1','1','1','0'))
    );

    -- ── 5×7 bitmap font for "BEST" label + space ─────────────────────
    -- Indices: 0=B  1=E  2=S  3=T  4=space
    -- Reuses digit_rows_t so no extra type needed.
    type best_font_t is array (0 to 4) of digit_rows_t;
    constant BEST_FONT : best_font_t := (
        -- B
        (('1','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('1','1','1','1','0'), ('1','0','0','0','1'), ('1','0','0','0','1'),
         ('1','1','1','1','0')),
        -- E
        (('1','1','1','1','1'), ('1','0','0','0','0'), ('1','0','0','0','0'),
         ('1','1','1','1','0'), ('1','0','0','0','0'), ('1','0','0','0','0'),
         ('1','1','1','1','1')),
        -- S
        (('0','1','1','1','1'), ('1','0','0','0','0'), ('1','0','0','0','0'),
         ('0','1','1','1','0'), ('0','0','0','0','1'), ('0','0','0','0','1'),
         ('1','1','1','1','0')),
        -- T
        (('1','1','1','1','1'), ('0','0','1','0','0'), ('0','0','1','0','0'),
         ('0','0','1','0','0'), ('0','0','1','0','0'), ('0','0','1','0','0'),
         ('0','0','1','0','0')),
        -- space
        (('0','0','0','0','0'), ('0','0','0','0','0'), ('0','0','0','0','0'),
         ('0','0','0','0','0'), ('0','0','0','0','0'), ('0','0','0','0','0'),
         ('0','0','0','0','0'))
    );

    -- ── Particle colour lookup table ──────────────────────────────────
    -- 12-bit packed: bits [11:8]=R, [7:4]=G, [3:0]=B
    type col_lut_t is array (0 to 5) of std_logic_vector(11 downto 0);
    constant COL_LUT : col_lut_t := (
        x"F20",   -- 0  red-orange
        x"F80",   -- 1  orange
        x"FF0",   -- 2  yellow
        x"0F4",   -- 3  green
        x"0EF",   -- 4  cyan
        x"E0F"    -- 5  magenta
    );

begin

    process(clk_50)
        -- ── Aligned pixel coordinates (old values = hpos[T-3]) ────────
        variable px, py     : integer;

        -- ── Output colour ─────────────────────────────────────────────
        variable out_r, out_g, out_b : std_logic_vector(3 downto 0);

        -- ── Particle variables (IDLE state) ───────────────────────────
        variable part_cx   : integer;
        variable part_cy   : integer;
        variable part_half : integer;
        variable cidx      : integer;
        variable sq_l, sq_r, sq_t, sq_b : integer;

        -- ── Shared text variables ─────────────────────────────────────
        variable tx_off, ty_off : integer;
        variable char_i, col_i, row_i : integer;
        variable text_pixel : std_logic;

        -- ── Score / high-score display variables ──────────────────────
        variable sc_char  : integer range 0 to 2;   -- 0=hundreds 1=tens 2=ones
        variable sc_digit : integer range 0 to 9;

        -- ── High score comparison (used once per clock in GAME OVER) ──
        variable cur_val  : integer range 0 to 9999 := 0;
        variable hs_val   : integer range 0 to 9999 := 0;

    begin
        if rising_edge(clk_50) then

            -- ── Advance position delay chain ──────────────────────────
            hpos_d1 <= unsigned(hpos);
            hpos_d2 <= hpos_d1;
            hpos_d3 <= hpos_d2;
            vpos_d1 <= unsigned(vpos);
            vpos_d2 <= vpos_d1;
            vpos_d3 <= vpos_d2;
            vdo_d1  <= video_on;
            vdo_d2  <= vdo_d1;
            vdo_d3  <= vdo_d2;

            -- ── Bounce update (once per frame, GAME OVER state only) ──
            -- Speed: ±2 px/frame ≈ 144 px/s at 72 Hz.
            -- X and Y have different traversal times (476 vs 520 units),
            -- so they never phase-lock — corner hits are rare (DVD feel).
            if frame_tick = '1' and game_state = "10" then

                -- X axis
                if go_dx_r = '1' then              -- moving right
                    if go_bx_r + 2 >= BOUNCE_X_MAX then
                        go_bx_r <= BOUNCE_X_MAX;
                        go_dx_r <= '0';
                    else
                        go_bx_r <= go_bx_r + 2;
                    end if;
                else                               -- moving left
                    if go_bx_r < 2 then
                        go_bx_r <= 0;
                        go_dx_r <= '1';
                    else
                        go_bx_r <= go_bx_r - 2;
                    end if;
                end if;

                -- Y axis
                if go_dy_r = '1' then              -- moving down
                    if go_by_r + 2 >= BOUNCE_Y_MAX then
                        go_by_r <= BOUNCE_Y_MAX;
                        go_dy_r <= '0';
                    else
                        go_by_r <= go_by_r + 2;
                    end if;
                else                               -- moving up
                    if go_by_r < 2 then
                        go_by_r <= 0;
                        go_dy_r <= '1';
                    else
                        go_by_r <= go_by_r - 2;
                    end if;
                end if;

            end if;

            -- ── High score update (every clock while in GAME OVER) ────
            -- Converts both BCD values to integers for comparison,
            -- then copies the whole score vector if it beats the record.
            -- After one update high_score_bcd = score, condition stays false.
            if game_state = "10" then
                cur_val := to_integer(unsigned(score(11 downto 8))) * 100
                         + to_integer(unsigned(score(7  downto 4))) * 10
                         + to_integer(unsigned(score(3  downto 0)));
                hs_val  := to_integer(unsigned(high_score_bcd(11 downto 8))) * 100
                         + to_integer(unsigned(high_score_bcd(7  downto 4))) * 10
                         + to_integer(unsigned(high_score_bcd(3  downto 0)));
                if cur_val > hs_val then
                    high_score_bcd <= score;   -- score IS BCD — direct copy
                end if;
            end if;

            -- OLD hpos_d3 / vpos_d3 read here align with crt_r/g/b input
            px := to_integer(hpos_d3);
            py := to_integer(vpos_d3);

            -- Default: black
            out_r := "0000"; out_g := "0000"; out_b := "0000";

            if vdo_d3 = '1' then

                -- ==============================================================
                -- IDLE:  start screen
                -- Layer order (back to front):
                --   1. Coloured particle squares  (behind everything)
                --   2. "TUNNEL RUSH" title (neon cyan, scale 8)
                --   3. White "PRESS START" text at bottom
                -- ==============================================================
                if game_state = "00" then

                    -- ── Layer 1: Particle squares ──────────────────────
                    for i in 0 to 5 loop
                        if sq_active(i) = '1' then

                            part_cx   := to_integer(signed  (sq_px (11*(i+1)-1 downto 11*i)));
                            part_cy   := to_integer(signed  (sq_py (11*(i+1)-1 downto 11*i)));
                            part_half := to_integer(unsigned(sq_half(8*(i+1)-1 downto  8*i)));
                            cidx      := to_integer(unsigned(sq_col ( 3*(i+1)-1 downto  3*i)));
                            if cidx > 5 then cidx := 5; end if;

                            sq_l := part_cx - part_half;
                            sq_r := part_cx + part_half;
                            sq_t := part_cy - part_half;
                            sq_b := part_cy + part_half;

                            if px >= sq_l and px <= sq_r and
                               py >= sq_t and py <= sq_b then
                                out_r := COL_LUT(cidx)(11 downto 8);
                                out_g := COL_LUT(cidx)(7  downto 4);
                                out_b := COL_LUT(cidx)(3  downto 0);
                            end if;

                        end if;
                    end loop;

                    -- ── Layer 2: "TUNNEL RUSH" title (neon cyan) ────────
                    tx_off := px - TITLE_X;
                    ty_off := py - TITLE_Y;
                    if tx_off >= 0 and tx_off < TITLE_W and
                       ty_off >= 0 and ty_off < TITLE_H then
                        char_i := (tx_off / TITLE_SCALE) / TITLE_CHAR_W;
                        col_i  := (tx_off / TITLE_SCALE) mod TITLE_CHAR_W;
                        row_i  :=  ty_off / TITLE_SCALE;
                        if col_i < 5 and
                           TITLE_FONT(char_i)(row_i)(col_i) = '1' then
                            out_r := "0000";
                            out_g := "1111";
                            out_b := "1111";  -- neon cyan
                        end if;
                    end if;

                    -- ── Layer 3: "PRESS START" text (bottom) ───────────
                    text_pixel := '0';
                    tx_off := px - TEXT_X;
                    ty_off := py - TEXT_Y;

                    if tx_off >= 0 and tx_off < NUM_CHARS * CHAR_W * SCALE and
                       ty_off >= 0 and ty_off < CHAR_H * SCALE then

                        char_i := (tx_off / SCALE) / CHAR_W;
                        col_i  := (tx_off / SCALE) mod CHAR_W;
                        row_i  :=  ty_off / SCALE;

                        if col_i < 5 then
                            text_pixel := FONT(char_i)(row_i, col_i);
                        end if;
                    end if;

                    if text_pixel = '1' then
                        out_r := "1111"; out_g := "1111"; out_b := "1111";
                    end if;

                -- ==============================================================
                -- PLAYING:  tunnel pass-through + score display top-left
                -- ==============================================================
                elsif game_state = "01" then
                    out_r := crt_r;
                    out_g := crt_g;
                    out_b := crt_b;

                    -- ── Score: 3 BCD digits at top-left ───────────────
                    tx_off := px - SCORE_X;
                    ty_off := py - SCORE_Y;

                    if tx_off >= 0 and tx_off < 3 * SCORE_CHAR_W * SCORE_SCALE and
                       ty_off >= 0 and ty_off < SCORE_CHAR_H * SCORE_SCALE then

                        sc_char := (tx_off / SCORE_SCALE) / SCORE_CHAR_W;
                        col_i   := (tx_off / SCORE_SCALE) mod SCORE_CHAR_W;
                        row_i   :=  ty_off / SCORE_SCALE;

                        if    sc_char = 0 then
                            sc_digit := to_integer(unsigned(score(11 downto 8)));
                        elsif sc_char = 1 then
                            sc_digit := to_integer(unsigned(score(7  downto 4)));
                        else
                            sc_digit := to_integer(unsigned(score(3  downto 0)));
                        end if;
                        if sc_digit > 9 then sc_digit := 9; end if;

                        if col_i < 5 and DIGIT_FONT(sc_digit)(row_i)(col_i) = '1' then
                            out_r := "1111"; out_g := "1111"; out_b := "0000"; -- gold
                        end if;

                    end if;

                -- ==============================================================
                -- GAME OVER:  black background
                --   Layer 1 — "GAME OVER" in red,  bouncing like a DVD logo.
                --   Layer 2 — "BEST XXX" in amber below, moves with Layer 1.
                -- ==============================================================
                else

                    -- ── Layer 1: bouncing "GAME OVER" (red) ───────────
                    text_pixel := '0';
                    tx_off := px - go_bx_r;
                    ty_off := py - go_by_r;

                    if tx_off >= 0 and tx_off < UNIT_W and
                       ty_off >= 0 and ty_off < GO_CHAR_H * GO_SCALE then

                        char_i := (tx_off / GO_SCALE) / GO_CHAR_W;
                        col_i  := (tx_off / GO_SCALE) mod GO_CHAR_W;
                        row_i  :=  ty_off / GO_SCALE;

                        if col_i < 5 then
                            text_pixel := GAME_OVER(char_i)(row_i)(col_i);
                        end if;
                    end if;

                    if text_pixel = '1' then
                        out_r := "1111"; out_g := "0000"; out_b := "0000"; -- red
                    end if;

                    -- ── Layer 2: "BEST XXX" high score (amber/gold) ────
                    -- Centred horizontally under "GAME OVER", BEST_Y_OFF px below.
                    -- "BEST " (chars 0-4 from BEST_FONT) then 3 digits from
                    -- DIGIT_FONT indexed by high_score_bcd (chars 5-7).
                    text_pixel := '0';
                    tx_off := px - (go_bx_r + BEST_X_OFF);
                    ty_off := py - (go_by_r + BEST_Y_OFF);

                    if tx_off >= 0 and tx_off < BEST_W and
                       ty_off >= 0 and ty_off < BEST_H then

                        char_i := (tx_off / BEST_SCALE) / BEST_CHAR_W;
                        col_i  := (tx_off / BEST_SCALE) mod BEST_CHAR_W;
                        row_i  :=  ty_off / BEST_SCALE;

                        if col_i < 5 then
                            if char_i <= 4 then
                                -- "BEST " from BEST_FONT
                                text_pixel := BEST_FONT(char_i)(row_i)(col_i);
                            else
                                -- 3 digits of high score (char_i 5,6,7)
                                sc_char := char_i - 5;   -- 0=hundreds 1=tens 2=ones
                                if sc_char = 0 then
                                    sc_digit :=
                                        to_integer(unsigned(high_score_bcd(11 downto 8)));
                                elsif sc_char = 1 then
                                    sc_digit :=
                                        to_integer(unsigned(high_score_bcd(7 downto 4)));
                                else
                                    sc_digit :=
                                        to_integer(unsigned(high_score_bcd(3 downto 0)));
                                end if;
                                if sc_digit > 9 then sc_digit := 9; end if;
                                text_pixel := DIGIT_FONT(sc_digit)(row_i)(col_i);
                            end if;
                        end if;
                    end if;

                    if text_pixel = '1' then
                        out_r := "1111"; out_g := "1100"; out_b := "0000"; -- amber
                    end if;

                end if; -- game_state

            end if; -- vdo_d3

            -- Register outputs (1 clock stage → 4-clock total pipeline)
            red_out   <= out_r;
            green_out <= out_g;
            blue_out  <= out_b;

        end if;
    end process;

end rtl;
