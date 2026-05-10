library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--
-- crt_effects.vhd
--
-- Input   : Tunnel pixel colour from tunnel.vhd
--           VGA scan position
--           Animation counters from RushTunnelFPGA.vhd :
--
--					hue_cnt — shifts grid colour each frame
--					scroll  — translates grid diagonals each frame
--
-- Output  : Post-processed pixel (1-clock pipeline)
-- Utility : Draws a sliding diagonal criss-cross grid (Operius style) on
--           near-black background pixels.
--           Sits between tunnel.vhd and screen_overlay.vhd in the chain
--

entity crt_effects is
    port (
        clk_50    : in  std_logic;
        red_in    : in  std_logic_vector(3 downto 0);
        green_in  : in  std_logic_vector(3 downto 0);
        blue_in   : in  std_logic_vector(3 downto 0);
        hpos      : in  std_logic_vector(9 downto 0);
        vpos      : in  std_logic_vector(9 downto 0);
        hue_cnt   : in  std_logic_vector(9 downto 0);
        scroll    : in  std_logic_vector(7 downto 0);
        red_out   : out std_logic_vector(3 downto 0);
        green_out : out std_logic_vector(3 downto 0);
        blue_out  : out std_logic_vector(3 downto 0)
    );
end crt_effects;

architecture rtl of crt_effects is

    -- Grid line width in pixels (spacing = 128, power-of-2 → mod via bit-slice)
    constant LINE_WIDTH : integer := 6;

    -- 2-stage delay chain to align hpos/vpos with the tunnel's 2-clock pipeline
    signal hpos_d1, hpos_d2 : unsigned(9 downto 0) := (others => '0');
    signal vpos_d1, vpos_d2 : unsigned(9 downto 0) := (others => '0');

begin

    process(clk_50)
        -- 12-bit intermediates for diagonal grid phase (max value < 4096)
        variable h_slash : unsigned(11 downto 0);
        variable h_back  : unsigned(11 downto 0);
        variable on_grid : boolean;

        -- 5-bit colour accumulators
        variable r5, g5, b5  : unsigned(4 downto 0);
        variable lr, lg, lb  : unsigned(3 downto 0);

        -- Input channel sum — background pixels have sum ≤ 2
        variable in_sum  : unsigned(5 downto 0);

        variable hue_idx : integer range 0 to 7;
    begin
        if rising_edge(clk_50) then

            -- Advance hpos/vpos delay chain
            hpos_d1 <= unsigned(hpos);  hpos_d2 <= hpos_d1;
            vpos_d1 <= unsigned(vpos);  vpos_d2 <= vpos_d1;

            -- Dim hue-cycling line colour (8 states, ~1.8 s each at 72 Hz)
            hue_idx := to_integer(unsigned(hue_cnt(9 downto 7)));
            case hue_idx is
                when 0      => lr := "0011"; lg := "0000"; lb := "0010";
                when 1      => lr := "0011"; lg := "0000"; lb := "0011";
                when 2      => lr := "0010"; lg := "0000"; lb := "0011";
                when 3      => lr := "0001"; lg := "0000"; lb := "0011";
                when 4      => lr := "0000"; lg := "0001"; lb := "0011";
                when 5      => lr := "0000"; lg := "0010"; lb := "0011";
                when 6      => lr := "0000"; lg := "0011"; lb := "0011";
                when others => lr := "0000"; lg := "0011"; lb := "0010";
            end case;

            -- "/" family: (hpos + vpos + 1024 − scroll) mod 128 — scrolls diagonally
            h_slash := resize(hpos_d2, 12)
                     + resize(vpos_d2, 12)
                     + to_unsigned(1024, 12)
                     - resize(unsigned(scroll), 12);

            -- "\" family: (hpos − vpos + 1024) mod 128 — fixed (no scroll)
            h_back  := resize(hpos_d2, 12)
                     + to_unsigned(1024, 12)
                     - resize(vpos_d2, 12);

            -- Pixel is on a grid line when its low 7 bits (mod 128) < LINE_WIDTH
            on_grid := (to_integer(h_slash(6 downto 0)) < LINE_WIDTH) or
                       (to_integer(h_back (6 downto 0)) < LINE_WIDTH);

            -- Grid replaces only near-black background pixels (R+G+B ≤ 2)
            in_sum := resize(unsigned(red_in),   6)
                    + resize(unsigned(green_in), 6)
                    + resize(unsigned(blue_in),  6);

            if on_grid and in_sum <= 2 then
                r5 := resize(lr, 5);
                g5 := resize(lg, 5);
                b5 := resize(lb, 5);
            else
                r5 := resize(unsigned(red_in),   5);
                g5 := resize(unsigned(green_in), 5);
                b5 := resize(unsigned(blue_in),  5);
            end if;

            -- Register output
            red_out   <= std_logic_vector(r5(3 downto 0));
            green_out <= std_logic_vector(g5(3 downto 0));
            blue_out  <= std_logic_vector(b5(3 downto 0));

        end if;
    end process;

end rtl;
