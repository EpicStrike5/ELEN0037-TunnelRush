library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- stage_manager.vhd
-- ================================================================
-- Tracks the current polygon shape and drives the inter-zone
-- blackout transition.
--
-- TRANSITION SEQUENCE (on zone_advance pulse):
--   1. LFSR ticks to produce a new random polygon selection.
--   2. transition_active goes '1' immediately.
--   3. colour_palette sees transition_active → outputs black.
--   4. After HALF_TRANS frames the screen is fully black:
--      num_faces is updated to the new polygon type.
--   5. After TRANSITION_LEN frames total, transition_active → '0'.
--      The new tunnel type is now visible.
--
-- POLYGON MAPPING (lfsr[2:0]):
--   0 → 4 faces  (square)
--   1 → 6 faces  (hexagon)
--   2 → 8 faces  (octagon — default)
--   3 → 3 faces  (triangle-ish)
--   4 → 5 faces  (pentagon-ish)
--   5 → 7 faces  (heptagon-ish)
--   6 → 4 faces  (square, duplicate)
--   7 → 6 faces  (hexagon, duplicate)
-- Duplicating 4 and 6 makes them more common (better-looking metric).
--
-- RESET:
--   resets to 8-face octagon with no transition.
-- ================================================================

entity stage_manager is
    port (
        clk_50            : in  std_logic;
        frame_tick        : in  std_logic;
        zone_advance      : in  std_logic;   -- 1-cycle pulse: unlock next zone
        reset_game        : in  std_logic;
        -- Outputs
        num_faces         : out std_logic_vector(3 downto 0);   -- current N (3..8)
        transition_active : out std_logic                        -- '1' during blackout
    );
end stage_manager;

architecture rtl of stage_manager is

    constant TRANSITION_LEN : integer := 32;   -- frames of total blackout
    constant HALF_TRANS     : integer := 16;   -- frame at which shape swaps (mid-black)
    constant DEMO_CYCLE     : integer := 360;  -- auto-cycle every ~5 s (360 frames @ 72 Hz)

    signal lfsr_r        : std_logic_vector(7 downto 0) := "10110101";
    signal num_faces_r   : unsigned(3 downto 0)         := to_unsigned(8, 4);
    signal pending_faces : unsigned(3 downto 0)         := to_unsigned(8, 4);
    signal trans_cnt     : integer range 0 to TRANSITION_LEN := TRANSITION_LEN;
    signal demo_cnt      : integer range 0 to DEMO_CYCLE - 1 := 0;
    -- trans_cnt = TRANSITION_LEN → idle (no transition)

    -- Decode LFSR[2:0] → polygon N
    function lfsr_to_faces(l : std_logic_vector(2 downto 0)) return unsigned is
    begin
        case l is
            when "000"  => return to_unsigned(4, 4);
            when "001"  => return to_unsigned(6, 4);
            when "010"  => return to_unsigned(8, 4);
            when "011"  => return to_unsigned(3, 4);
            when "100"  => return to_unsigned(5, 4);
            when "101"  => return to_unsigned(7, 4);
            when "110"  => return to_unsigned(4, 4);  -- square (common)
            when others => return to_unsigned(6, 4);  -- hexagon (common)
        end case;
    end function;

begin

    num_faces         <= std_logic_vector(num_faces_r);
    transition_active <= '0' when trans_cnt >= TRANSITION_LEN else '1';

    process(clk_50)
        variable next_lfsr : std_logic_vector(7 downto 0);
        variable do_change : boolean;
    begin
        if rising_edge(clk_50) then

            if reset_game = '1' then
                num_faces_r <= to_unsigned(8, 4);
                trans_cnt   <= TRANSITION_LEN;   -- idle
                demo_cnt    <= 0;

            else
                do_change := false;

                -- External zone_advance always wins if not already transitioning
                if zone_advance = '1' and trans_cnt >= TRANSITION_LEN then
                    do_change := true;
                end if;

                -- Demo auto-cycle: fire every DEMO_CYCLE frames
                if frame_tick = '1' then
                    if demo_cnt = DEMO_CYCLE - 1 then
                        demo_cnt <= 0;
                        if trans_cnt >= TRANSITION_LEN then
                            do_change := true;
                        end if;
                    else
                        demo_cnt <= demo_cnt + 1;
                    end if;
                end if;

                -- Kick off a new transition
                if do_change then
                    next_lfsr     := lfsr_r(6 downto 0) & (lfsr_r(7) xor lfsr_r(5) xor lfsr_r(4) xor lfsr_r(3));
                    lfsr_r        <= next_lfsr;
                    pending_faces <= lfsr_to_faces(next_lfsr(2 downto 0));
                    trans_cnt     <= 0;

                -- Advance transition counter each frame (only when not starting a new one)
                elsif frame_tick = '1' and trans_cnt < TRANSITION_LEN then
                    trans_cnt <= trans_cnt + 1;

                    -- Mid-blackout: swap polygon type (screen fully black at this point)
                    if trans_cnt = HALF_TRANS - 1 then
                        num_faces_r <= pending_faces;
                    end if;
                end if;

            end if;
        end if;
    end process;

end rtl;
