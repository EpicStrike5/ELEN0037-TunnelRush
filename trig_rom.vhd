library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

-- ================================================================
-- trig_rom.vhd  —  Combinational sin/cos lookup
-- ================================================================
-- Converts a 16-bit angle (0-65535 = 0°-360°) into signed 9-bit
-- sin and cos values (-255 to +255), purely combinationally.
--
-- Internally: 256-entry quarter-sine table initialised at synthesis
-- time via ieee.math_real.  Quartus Prime supports this.
--
-- Table:  QSIN[i] = round(255 * sin(i * pi / 512))   i = 0..255
-- Covers [0°, 90°).  Full circle handled by quadrant mirroring:
--
--   angle[15:14]  quadrant
--   "00"    0°- 90°   sin+  cos+
--   "01"   90°-180°   sin+  cos-
--   "10"  180°-270°   sin-  cos-
--   "11"  270°-360°   sin-  cos+
--
-- Index into QSIN:
--   Q0, Q2 :  sin_idx = angle[13:6],       cos_idx = 255 - angle[13:6]
--   Q1, Q3 :  sin_idx = 255 - angle[13:6], cos_idx = angle[13:6]
-- (cos(x) = sin(90°-x), which reverses the table.)
--
-- Sign:
--   sin negative when angle[15] = '1'  (Q2 or Q3)
--   cos negative when angle[15] XOR angle[14] = '1'  (Q1 or Q2)
-- ================================================================

entity trig_rom is
    port (
        angle   : in  std_logic_vector(15 downto 0);
        sin_val : out signed(8 downto 0);   -- -255 .. +255
        cos_val : out signed(8 downto 0)
    );
end trig_rom;

architecture rtl of trig_rom is

    -- ---- Quarter-sine ROM (synthesis-time initialisation) ----
    type qsin_t is array (0 to 255) of integer range 0 to 255;

    function init_qsin return qsin_t is
        variable t : qsin_t;
    begin
        for i in 0 to 255 loop
            t(i) := integer(round(255.0 * sin(real(i) * MATH_PI / 512.0)));
        end loop;
        return t;
    end function;

    constant QSIN : qsin_t := init_qsin;

    -- ---- Internal signals ----
    signal quad    : std_logic_vector(1 downto 0);
    signal idx     : integer range 0 to 255;
    signal sin_raw : integer range 0 to 255;
    signal cos_raw : integer range 0 to 255;

begin

    quad <= angle(15 downto 14);
    idx  <= to_integer(unsigned(angle(13 downto 6)));

    -- ---- Table lookup with quadrant mirroring ----
    process(quad, idx)
    begin
        case quad is
            when "00" | "10" =>          -- Q0 / Q2
                sin_raw <= QSIN(idx);
                cos_raw <= QSIN(255 - idx);
            when others =>               -- Q1 / Q3
                sin_raw <= QSIN(255 - idx);
                cos_raw <= QSIN(idx);
        end case;
    end process;

    -- ---- Sign application ----
    process(sin_raw, cos_raw, angle)
        variable sv : signed(8 downto 0);
        variable cv : signed(8 downto 0);
    begin
        sv := signed('0' & std_logic_vector(to_unsigned(sin_raw, 8)));
        cv := signed('0' & std_logic_vector(to_unsigned(cos_raw, 8)));

        if angle(15) = '1' then
            sv := -sv;                   -- sin negative in Q2, Q3
        end if;
        if (angle(15) xor angle(14)) = '1' then
            cv := -cv;                   -- cos negative in Q1, Q2
        end if;

        sin_val <= sv;
        cos_val <= cv;
    end process;

end rtl;
