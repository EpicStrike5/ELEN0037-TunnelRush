library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--
-- snes_controller.vhd
--
-- Input   : 50 MHz clock and serial data line from the SNES pad
-- Output  : Latch and clock lines to the SNES pad
--           16-bit parallel button register (active high) :
--
--					bit 3 — Start
--					bit 6 — Right
--					bit 7 — Left
--
-- Utility : Reads a Super Nintendo controller via its 16-bit serial protocol
--           at ~60 Hz.  Button state forwarded to spaceship.vhd and start_screen.vhd
--

entity snes_controller is
    port (
        clk_50   : in  std_logic;
        gp_latch : out std_logic;
        gp_clk   : out std_logic;
        gp_data  : in  std_logic;
        buttons  : out std_logic_vector(15 downto 0)
    );
end snes_controller;

architecture rtl of snes_controller is

    -- Timing constants at 50 MHz
    constant LATCH_CYCLES : integer := 600;    -- 12 µs latch pulse
    constant BIT_CYCLES   : integer := 600;    -- 12 µs per bit period
    constant FRAME_CYCLES : integer := 833333; -- ~60 Hz poll rate

    type state_t is (S_WAIT, S_LATCH, S_BIT_LOW, S_BIT_HIGH, S_DONE);
    signal state : state_t := S_WAIT;

    signal timer      : integer range 0 to FRAME_CYCLES - 1 := 0;
    signal bit_cnt    : integer range 0 to 15               := 0;
    signal shift_reg  : std_logic_vector(15 downto 0)       := (others => '0');
    signal buttons_r  : std_logic_vector(15 downto 0)       := (others => '0');
    signal gp_clk_r   : std_logic := '1';
    signal gp_latch_r : std_logic := '0';

begin

    gp_clk   <= gp_clk_r;
    gp_latch <= gp_latch_r;
    buttons  <= buttons_r;

    process(clk_50)
    begin
        if rising_edge(clk_50) then
            case state is

                -- Wait for next 60 Hz poll cycle
                when S_WAIT =>
                    gp_latch_r <= '0';
                    gp_clk_r   <= '1';
                    if timer = FRAME_CYCLES - 1 then
                        timer      <= 0;
                        state      <= S_LATCH;
                        gp_latch_r <= '1';
                    else
                        timer <= timer + 1;
                    end if;

                -- Assert LATCH high for 12 µs to latch all buttons
                when S_LATCH =>
                    gp_latch_r <= '1';
                    gp_clk_r   <= '1';
                    if timer = LATCH_CYCLES - 1 then
                        timer        <= 0;
                        gp_latch_r   <= '0';
                        shift_reg(0) <= not gp_data; -- bit 0 (B) immediately on DATA
                        bit_cnt      <= 1;
                        state        <= S_BIT_LOW;
                    else
                        timer <= timer + 1;
                    end if;

                -- CLK low phase — controller shifts to next bit
                when S_BIT_LOW =>
                    gp_clk_r <= '0';
                    if timer = BIT_CYCLES/2 - 1 then
                        timer <= 0;
                        state <= S_BIT_HIGH;
                    else
                        timer <= timer + 1;
                    end if;

                -- CLK high phase — sample DATA (active low → invert)
                when S_BIT_HIGH =>
                    gp_clk_r <= '1';
                    if timer = BIT_CYCLES/2 - 1 then
                        timer                  <= 0;
                        shift_reg(bit_cnt) <= not gp_data;
                        if bit_cnt = 15 then
                            state <= S_DONE;
                        else
                            bit_cnt <= bit_cnt + 1;
                            state   <= S_BIT_LOW;
                        end if;
                    else
                        timer <= timer + 1;
                    end if;

                -- Latch completed shift register to output, return to idle
                when S_DONE =>
                    buttons_r <= shift_reg;
                    timer     <= 0;
                    state     <= S_WAIT;

            end case;
        end if;
    end process;

end rtl;
