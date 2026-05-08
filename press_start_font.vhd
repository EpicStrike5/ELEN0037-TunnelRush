library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- press_start_font.vhd
-- ================================================================
-- Bitmap definitions for the "PRESS START" string (11 characters).
-- Format: 7 rows × 5 columns per character (2-D array).
-- Access pattern: FONT(char_index)(row, col)
-- This differs from game_over_font which uses three separate indices.
-- ================================================================

package press_start_font is

    -- 2-D bitmap: rows 0..6, cols 0..4
    type bitmap_ps_t is array (0 to 6, 0 to 4) of std_logic;
    -- Font array for 11 characters
    type font_ps_t   is array (0 to 10) of bitmap_ps_t;

    -- P
    constant CH_P : bitmap_ps_t := (
        ('1','1','1','1','0'),
        ('1','0','0','0','1'),
        ('1','0','0','0','1'),
        ('1','1','1','1','0'),
        ('1','0','0','0','0'),
        ('1','0','0','0','0'),
        ('1','0','0','0','0')
    );

    -- R
    constant CH_R_ps : bitmap_ps_t := (
        ('1','1','1','1','0'),
        ('1','0','0','0','1'),
        ('1','0','0','0','1'),
        ('1','1','1','1','0'),
        ('1','0','1','0','0'),
        ('1','0','0','1','0'),
        ('1','0','0','0','1')
    );

    -- E
    constant CH_E_ps : bitmap_ps_t := (
        ('1','1','1','1','1'),
        ('1','0','0','0','0'),
        ('1','0','0','0','0'),
        ('1','1','1','1','0'),
        ('1','0','0','0','0'),
        ('1','0','0','0','0'),
        ('1','1','1','1','1')
    );

    -- S
    constant CH_S_ps : bitmap_ps_t := (
        ('0','1','1','1','1'),
        ('1','0','0','0','0'),
        ('1','0','0','0','0'),
        ('0','1','1','1','0'),
        ('0','0','0','0','1'),
        ('0','0','0','0','1'),
        ('1','1','1','1','0')
    );

    -- Space
    constant CH_SP_ps : bitmap_ps_t := (
        ('0','0','0','0','0'),
        ('0','0','0','0','0'),
        ('0','0','0','0','0'),
        ('0','0','0','0','0'),
        ('0','0','0','0','0'),
        ('0','0','0','0','0'),
        ('0','0','0','0','0')
    );

    -- T
    constant CH_T_ps : bitmap_ps_t := (
        ('1','1','1','1','1'),
        ('0','0','1','0','0'),
        ('0','0','1','0','0'),
        ('0','0','1','0','0'),
        ('0','0','1','0','0'),
        ('0','0','1','0','0'),
        ('0','0','1','0','0')
    );

    -- A
    constant CH_A_ps : bitmap_ps_t := (
        ('0','1','1','1','0'),
        ('1','0','0','0','1'),
        ('1','0','0','0','1'),
        ('1','1','1','1','1'),
        ('1','0','0','0','1'),
        ('1','0','0','0','1'),
        ('1','0','0','0','1')
    );

    -- "PRESS START" = P R E S S [space] S T A R T
    constant FONT : font_ps_t := (
        CH_P, CH_R_ps, CH_E_ps, CH_S_ps, CH_S_ps,
        CH_SP_ps,
        CH_S_ps, CH_T_ps, CH_A_ps, CH_R_ps, CH_T_ps
    );

end package press_start_font;
