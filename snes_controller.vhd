library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- snes_controller.vhd
-- ================================================================
-- Reads a Super Nintendo (SNES) controller via its serial protocol.
-- Outputs a 16-bit button register, active high (pressed = '1').
--
-- Protocol (active low on DATA, active high for LATCH/CLK):
--   1. Assert LATCH high for 12 us  → controller latches all buttons
--   2. De-assert LATCH
--   3. Bit 0 (B) is immediately available on DATA (active low)
--   4. 15 more CLK pulses shift out bits 1..15
--      Data is stable during CLK high; changes on CLK falling edge
-- Repeat at ~60 Hz (every 833 333 clk_50 cycles).
--
-- Button index (bit 0 = first bit):
--   0:B  1:Y  2:Sel  3:Start  4:Up  5:Down  6:Left  7:Right
--   8:A  9:X  10:L   11:R     12-15: always '1' (inactive)
--
-- Pin mapping (from QSF):
--   gp_latch → GP_L_1  (PIN_L14)
--   gp_clk   → GP_C_1  (PIN_K15)
--   gp_data  ← GP_D_1  (PIN_J16)
-- ================================================================

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

    -- Timing at 50 MHz
    constant HALF_US      : integer := 50;          -- 1 us = 50 cycles
    constant LATCH_CYCLES : integer := 600;         -- 12 us latch pulse
    constant BIT_CYCLES   : integer := 600;         -- 12 us per bit period
    constant FRAME_CYCLES : integer := 833333;      -- ~60 Hz frame

    type state_t is (S_WAIT, S_LATCH, S_BIT_LOW, S_BIT_HIGH, S_DONE);
    signal state : state_t := S_WAIT;

    signal timer   : integer range 0 to FRAME_CYCLES - 1 := 0;
    signal bit_cnt : integer range 0 to 15 := 0;

    signal shift_reg  : std_logic_vector(15 downto 0) := (others => '0');
    signal buttons_r  : std_logic_vector(15 downto 0) := (others => '0');

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

                -- ---- Wait for next frame ----
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

                -- ---- Pulse LATCH high for 12 us ----
                when S_LATCH =>
                    gp_latch_r <= '1';
                    gp_clk_r   <= '1';
                    if timer = LATCH_CYCLES - 1 then
                        timer      <= 0;
                        gp_latch_r <= '0';
                        -- Bit 0 (B) is now on DATA; CLK stays high, sample it
                        shift_reg(0) <= not gp_data;  -- active low → invert
                        bit_cnt      <= 1;
                        state        <= S_BIT_LOW;    -- next: clock low for bit 1
                    else
                        timer <= timer + 1;
                    end if;

                -- ---- CLK low phase (controller shifts to next bit) ----
                when S_BIT_LOW =>
                    gp_clk_r <= '0';
                    if timer = BIT_CYCLES/2 - 1 then
                        timer <= 0;
                        state <= S_BIT_HIGH;
                    else
                        timer <= timer + 1;
                    end if;

                -- ---- CLK high phase (sample DATA) ----
                when S_BIT_HIGH =>
                    gp_clk_r <= '1';
                    if timer = BIT_CYCLES/2 - 1 then
                        timer <= 0;
                        shift_reg(bit_cnt) <= not gp_data;   -- active low → invert
                        if bit_cnt = 15 then
                            state <= S_DONE;
                        else
                            bit_cnt <= bit_cnt + 1;
                            state   <= S_BIT_LOW;
                        end if;
                    else
                        timer <= timer + 1;
                    end if;

                -- ---- Latch output register, return to wait ----
                when S_DONE =>
                    buttons_r <= shift_reg;
                    timer     <= 0;
                    state     <= S_WAIT;

            end case;
        end if;
    end process;

end rtl;
