library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

--
-- trig_rom.vhd
--
-- Input   : 16-bit unsigned angle (full 0–65535 range maps to 0°–360°)
-- Output  : Signed sine and cosine results (range −255..+255)
-- Utility : Combinational sin/cos lookup via a 256-entry quarter-wave
--           ROM initialised at synthesis time with ieee.math_real.
--           Two instances in RushTunnelFPGA.vhd :
--
--					u_trig_tun  — tunnel rotation angle
--					u_trig_ship — ship screen position
--

entity trig_rom is
    port (
        angle   : in  std_logic_vector(15 downto 0);
        sin_val : out signed(8 downto 0);
        cos_val : out signed(8 downto 0)
    );
end trig_rom;

architecture rtl of trig_rom is

    -- Quarter-sine table: QSIN[i] = round(255 × sin(i × π/512)), i = 0..255
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

    signal quad    : std_logic_vector(1 downto 0);
    signal idx     : integer range 0 to 255;
    signal sin_raw : integer range 0 to 255;
    signal cos_raw : integer range 0 to 255;

begin

    -- Extract quadrant and table index from angle bits [15:14] and [13:6]
    quad <= angle(15 downto 14);
    idx  <= to_integer(unsigned(angle(13 downto 6)));

    -- Table lookup with quadrant mirroring (cos(x) = sin(90°−x) reverses the index)
    process(quad, idx)
    begin
        case quad is
            when "00" | "10" =>      -- Q0 / Q2: direct index
                sin_raw <= QSIN(idx);
                cos_raw <= QSIN(255 - idx);
            when others =>           -- Q1 / Q3: mirrored index
                sin_raw <= QSIN(255 - idx);
                cos_raw <= QSIN(idx);
        end case;
    end process;

    -- Apply sign: sin negative in Q2/Q3 (bit15=1), cos negative in Q1/Q2
    process(sin_raw, cos_raw, angle)
        variable sv : signed(8 downto 0);
        variable cv : signed(8 downto 0);
    begin
        sv := signed('0' & std_logic_vector(to_unsigned(sin_raw, 8)));
        cv := signed('0' & std_logic_vector(to_unsigned(cos_raw, 8)));

        if angle(15) = '1' then
            sv := -sv;
        end if;
        if (angle(15) xor angle(14)) = '1' then
            cv := -cv;
        end if;

        sin_val <= sv;
        cos_val <= cv;
    end process;

end rtl;
