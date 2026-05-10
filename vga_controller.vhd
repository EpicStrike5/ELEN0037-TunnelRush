library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--
-- vga_controller.vhd
--
-- Input   : 50 MHz pixel clock and synchronous reset
-- Output  : Horizontal and vertical sync pulses (active low)
--           Active-video flag and pixel scan coordinates
-- Utility : Generates 800×600 @ 72 Hz VGA timing.
--           Consumed by tunnel.vhd, crt_effects.vhd, screen_overlay.vhd
--

entity vga_controller is
    port (
        clk_50   : in  std_logic;
        rst      : in  std_logic;
        hsync    : out std_logic;
        vsync    : out std_logic;
        video_on : out std_logic;
        hpos     : out std_logic_vector(9 downto 0);
        vpos     : out std_logic_vector(9 downto 0)
    );
end vga_controller;

architecture rtl of vga_controller is

    -- Horizontal timing constants (pixels at 50 MHz)
    constant H_ACTIVE   : integer := 800;
    constant H_FP       : integer := 56;
    constant H_SYNC_W   : integer := 120;
    constant H_BP       : integer := 64;
    constant H_TOTAL    : integer := 1040;

    -- Vertical timing constants (lines)
    constant V_ACTIVE   : integer := 600;
    constant V_FP       : integer := 37;
    constant V_SYNC_W   : integer := 6;
    constant V_BP       : integer := 23;
    constant V_TOTAL    : integer := 666;

    -- Sync pulse start / end positions
    constant H_SYNC_BEG : integer := H_ACTIVE + H_FP;
    constant H_SYNC_END : integer := H_ACTIVE + H_FP + H_SYNC_W;
    constant V_SYNC_BEG : integer := V_ACTIVE + V_FP;
    constant V_SYNC_END : integer := V_ACTIVE + V_FP + V_SYNC_W;

    signal h_cnt : unsigned(10 downto 0) := (others => '0');
    signal v_cnt : unsigned(10 downto 0) := (others => '0');

begin

    -- Horizontal and vertical counter update
    process(clk_50)
    begin
        if rising_edge(clk_50) then
            if rst = '1' then
                h_cnt <= (others => '0');
                v_cnt <= (others => '0');
            else
                if h_cnt = H_TOTAL - 1 then
                    h_cnt <= (others => '0');
                    if v_cnt = V_TOTAL - 1 then
                        v_cnt <= (others => '0');
                    else
                        v_cnt <= v_cnt + 1;
                    end if;
                else
                    h_cnt <= h_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    -- Active video region flag
    video_on <= '1' when (h_cnt < H_ACTIVE and v_cnt < V_ACTIVE) else '0';

    -- Sync pulses (active low for 800×600 @ 72 Hz)
    hsync <= '0' when (h_cnt >= H_SYNC_BEG and h_cnt < H_SYNC_END) else '1';
    vsync <= '0' when (v_cnt >= V_SYNC_BEG and v_cnt < V_SYNC_END) else '1';

    -- Pixel coordinates (valid only when video_on = '1')
    hpos <= std_logic_vector(h_cnt(9 downto 0));
    vpos <= std_logic_vector(v_cnt(9 downto 0));

end rtl;
