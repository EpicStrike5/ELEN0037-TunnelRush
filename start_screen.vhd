library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ================================================================
-- start_screen.vhd
-- ================================================================
-- Generates NUM_SQ = 6 particle squares that fly outward from the
-- screen centre (400, 300).  Each square:
--   1. Spawns at the centre with size = 0
--   2. Flies toward a random target on screen
--   3. Grows from 0 to HALF_MAX (= 100, so 200×200 px at full size)
--   4. Deactivates when size reaches HALF_MAX → respawn next cycle
--
-- Movement: integer-lerp each frame
--   step_x = (target_x - pos_x) >> LERP_SHIFT
--   step_y = (target_y - pos_y) >> LERP_SHIFT
-- LERP_SHIFT = 4 → 1/16 of remaining gap per frame (slow start,
-- decelerate near target).
--
-- Randomness: 16-bit Galois LFSR, ticks every 50 MHz clock.
--   Target X: lfsr[9:0]  → mapped to 80..719
--   Target Y: lfsr[15:6] → mapped to 50..549
--
-- Flat output buses (NUM_SQ items each, index i occupies the slice
-- width*(i+1)-1 downto width*i):
--   sq_px     : 6 × 11-bit signed   centre X
--   sq_py     : 6 × 11-bit signed   centre Y
--   sq_half   : 6 × 8-bit unsigned  half-size (0..HALF_MAX)
--   sq_active : 6 × 1-bit           active flag
--   sq_col    : 6 × 3-bit           colour index (0..5)
-- ================================================================

entity start_screen is
    port (
        CLK_50     : in  std_logic;
        frame_tick : in  std_logic;
        buttons    : in  std_logic_vector(15 downto 0);  -- unused; kept for future use

        sq_px     : out std_logic_vector(65 downto 0);   -- 6 × 11-bit signed  pos-X
        sq_py     : out std_logic_vector(65 downto 0);   -- 6 × 11-bit signed  pos-Y
        sq_half   : out std_logic_vector(47 downto 0);   -- 6 × 8-bit          half-size
        sq_active : out std_logic_vector(5  downto 0);   -- 6 × 1-bit          active
        sq_col    : out std_logic_vector(17 downto 0)    -- 6 × 3-bit          colour
    );
end start_screen;

architecture rtl of start_screen is

    constant NUM_SQ    : integer := 6;
    constant HALF_MAX  : integer := 100;   -- maximum half-size in pixels
    constant GROW_STEP : integer := 2;     -- half-size grows by this many px/frame
    constant LERP_SHIFT: integer := 4;     -- position moves 1/16 of remaining gap/frame
    constant CX        : integer := 400;   -- spawn centre X
    constant CY        : integer := 300;   -- spawn centre Y
    constant SPAWN_GAP : integer := 12;    -- frames between spawn attempts

    type pos_arr  is array (0 to NUM_SQ-1) of signed(10 downto 0);
    type half_arr is array (0 to NUM_SQ-1) of unsigned(7 downto 0);
    type col_arr  is array (0 to NUM_SQ-1) of unsigned(2 downto 0);
    type bool_arr is array (0 to NUM_SQ-1) of std_logic;

    signal s_px     : pos_arr  := (others => to_signed(CX, 11));
    signal s_py     : pos_arr  := (others => to_signed(CY, 11));
    signal s_tx     : pos_arr  := (others => to_signed(CX, 11));  -- target X
    signal s_ty     : pos_arr  := (others => to_signed(CY, 11));  -- target Y
    signal s_half   : half_arr := (others => (others => '0'));
    signal s_active : bool_arr := (others => '0');
    signal s_col    : col_arr  := (others => (others => '0'));

    -- 16-bit Galois LFSR (polynomial x^16 + x^15 + x^13 + x^4 + 1)
    signal lfsr      : unsigned(15 downto 0) := x"ACE1";
    signal next_col  : unsigned(2 downto 0)  := (others => '0');
    signal spawn_timer : integer range 0 to SPAWN_GAP-1 := 0;

begin

    -- Flatten outputs: particle i occupies width*(i+1)-1 downto width*i
    gen_out: for i in 0 to NUM_SQ-1 generate
        sq_px  (11*(i+1)-1 downto 11*i) <= std_logic_vector(s_px(i));
        sq_py  (11*(i+1)-1 downto 11*i) <= std_logic_vector(s_py(i));
        sq_half( 8*(i+1)-1 downto  8*i) <= std_logic_vector(s_half(i));
        sq_active(i)                     <= s_active(i);
        sq_col ( 3*(i+1)-1 downto  3*i) <= std_logic_vector(s_col(i));
    end generate;

    process(CLK_50)
        variable dx, dy           : signed(10 downto 0);
        variable step_x, step_y  : signed(10 downto 0);
        variable new_x, new_y    : signed(10 downto 0);
        variable found            : boolean;
        variable tx11, ty11       : signed(10 downto 0);
    begin
        if rising_edge(CLK_50) then

            -- LFSR ticks every clock for pseudo-random target generation
            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            if frame_tick = '1' then

                -- Move and grow all active particles
                for i in 0 to NUM_SQ-1 loop
                    if s_active(i) = '1' then

                        -- Grow; deactivate when full-size is reached
                        if s_half(i) + GROW_STEP >= HALF_MAX then
                            s_half(i)   <= to_unsigned(HALF_MAX, 8);
                            s_active(i) <= '0';
                        else
                            s_half(i) <= s_half(i) + GROW_STEP;
                        end if;

                        -- Integer lerp: move 1/16 of remaining distance
                        dx     := s_tx(i) - s_px(i);
                        dy     := s_ty(i) - s_py(i);
                        step_x := shift_right(dx, LERP_SHIFT);
                        step_y := shift_right(dy, LERP_SHIFT);

                        -- Guarantee at least 1 px of movement while not at target
                        if dx > 0 and step_x = 0 then step_x := to_signed(1, 11); end if;
                        if dx < 0 and step_x = 0 then step_x := to_signed(-1,11); end if;
                        if dy > 0 and step_y = 0 then step_y := to_signed(1, 11); end if;
                        if dy < 0 and step_y = 0 then step_y := to_signed(-1,11); end if;

                        new_x := s_px(i) + step_x;
                        new_y := s_py(i) + step_y;

                        -- Clamp to screen bounds
                        if new_x < 0   then new_x := to_signed(0,   11); end if;
                        if new_x > 799 then new_x := to_signed(799, 11); end if;
                        if new_y < 0   then new_y := to_signed(0,   11); end if;
                        if new_y > 599 then new_y := to_signed(599, 11); end if;

                        s_px(i) <= new_x;
                        s_py(i) <= new_y;
                    end if;
                end loop;

                -- Every SPAWN_GAP frames, fill the first free slot
                if spawn_timer = SPAWN_GAP - 1 then
                    spawn_timer <= 0;
                    found := false;

                    for i in 0 to NUM_SQ-1 loop
                        if s_active(i) = '0' and not found then
                            found := true;

                            -- Target X: lfsr[9:0] → 80..719 (≈ ×5/8 = >>1 + >>3)
                            tx11 := to_signed(
                                80 + to_integer(
                                    shift_right(lfsr(9 downto 0), 1) +
                                    shift_right(lfsr(9 downto 0), 3)), 11);

                            -- Target Y: lfsr[15:6] → 50..549
                            ty11 := to_signed(
                                50 + to_integer(
                                    shift_right(lfsr(15 downto 6), 1) +
                                    shift_right(lfsr(15 downto 6), 3)), 11);

                            -- Clamp targets to safe range
                            if tx11 > 720 then tx11 := to_signed(720, 11); end if;
                            if tx11 <  80 then tx11 := to_signed( 80, 11); end if;
                            if ty11 > 550 then ty11 := to_signed(550, 11); end if;
                            if ty11 <  50 then ty11 := to_signed( 50, 11); end if;

                            s_px(i)     <= to_signed(CX, 11);
                            s_py(i)     <= to_signed(CY, 11);
                            s_tx(i)     <= tx11;
                            s_ty(i)     <= ty11;
                            s_half(i)   <= (others => '0');
                            s_active(i) <= '1';
                            s_col(i)    <= next_col;

                            if next_col = 5 then
                                next_col <= (others => '0');
                            else
                                next_col <= next_col + 1;
                            end if;
                        end if;
                    end loop;

                else
                    spawn_timer <= spawn_timer + 1;
                end if;

            end if; -- frame_tick
        end if; -- rising_edge
    end process;

end rtl;
