----------------------------------------------------------------------
----                                                              ----
---- ATARI ST BLITTER compatible IP Core			              ----
----                                                              ----
---- This file is part of the SUSKA ATARI clone project.          ----
---- http://www.experiment-s.de                                   ----
----                                                              ----
---- Description:                                                 ----
---- ATARI ST and STE compatible Bit Block Transfer Processor	  ----
---- (BLITTER) IP core.									          ----
----                                                              ----
---- Data processing (core) file of the BLITTER implementation.   ----
----                                                              ----
----                                                              ----
---- To Do:                                                       ----
---- -                                                            ----
----                                                              ----
---- Author(s):                                                   ----
---- - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de   ----
----                                                              ----
----------------------------------------------------------------------
----                                                              ----
---- Copyright (C) 2006 - 2010 Wolfgang Foerster                  ----
----                                                              ----
---- This source file may be used and distributed without         ----
---- restriction provided that this copyright statement is not    ----
---- removed from the file and that any derivative work contains  ----
---- the original copyright notice and the associated disclaimer. ----
----                                                              ----
---- This source file is free software; you can redistribute it   ----
---- and/or modify it under the terms of the GNU Lesser General   ----
---- Public License as published by the Free Software Foundation; ----
---- either version 2.1 of the License, or (at your option) any   ----
---- later version.                                               ----
----                                                              ----
---- This source is distributed in the hope that it will be       ----
---- useful, but WITHOUT ANY WARRANTY; without even the implied   ----
---- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ----
---- PURPOSE. See the GNU Lesser General Public License for more  ----
---- details.                                                     ----
----                                                              ----
---- You should have received a copy of the GNU Lesser General    ----
---- Public License along with this source; if not, download it   ----
---- from http://www.gnu.org/licenses/lgpl.html                   ----
----                                                              ----
----------------------------------------------------------------------
-- 
-- Revision History
-- 
-- Revision 2K6A  2006/06/03 WF
--   Initial Release.
-- Revision 2K6B	2006/11/05 WF
--   Modified Source to compile with the Xilinx ISE.
-- Revision 2K8A  2008/07/14 WF
--   Minor changes.
-- 

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity WF101643IP_CORE is
port (  CLK			: in std_logic;
		CLK_EN_p	: in bit;
		CLK_EN_n	: in bit;
		RESETn		: in bit;
		ADR_IN		: in std_logic_vector(23 downto 1); -- ADDRESS inputs.
		ADR_OUT		: out bit_vector(23 downto 1);
		ADR_SEL		: in bit;
		ASn			: in bit; -- ADDRESS select, low active.
		LDSn		: in bit; -- Lower data strobe, low active.
		UDSn		: in bit; -- Higher data strobe, low active.
		RWn			: in bit; -- Register read write control, low active.
		FC			: in bit_vector(2 downto 0); -- Processor function code.
		BERRn		: in bit; -- Bus error.
		DATA_IN		: in std_logic_vector(15 downto 0); -- Data bus.
		DATA_OUT	: out bit_vector(15 downto 0);
		DATA_EN		: out bit; 

		-- BLITTER data processing controls:
		SWAPSRC			: in bit;
		FETCHSRC		: in bit;
		FETCHDEST		: in bit;
		PUSHDEST		: in bit;
		FORCE_X			: in boolean;
		SRCADR_MODIFY	: in boolean;
		DESTADR_MODIFY	: in boolean;


		-- BLITTER progress controls:
		X_COUNT_DEC		: in boolean;
		BLT_RESTART		: buffer bit;
		XCNT_RELOAD		: out bit_vector(15 downto 0);
		XCNT_VALUE		: out bit_vector(15 downto 0);
		FORCE_DEST		: out bit;
		BLT_BSY			: out bit; -- Flag indicates with '0' that the Y counter is x"0000".

		-- The flags:
		FXSR	: buffer bit;
		NFSR	: buffer bit;
		OP		: buffer bit_vector(3 downto 0);
		HOP		: buffer bit_vector(1 downto 0);
		HOG		: buffer bit;
		BUSY	: buffer bit;
		SMUDGE	: buffer bit
      );
end WF101643IP_CORE;

architecture BEHAVIOR of WF101643IP_CORE is
-- System signals:
signal ADR_I			: bit_vector(23 downto 0);
signal SUPERUSER		: boolean; -- Indicates supervisor modus.
signal WRITEWORD		: boolean; -- Indicates correct word write conditions.
signal WRITEHI			: boolean; -- Indicates correct high byte write conditions.
signal WRITELOW			: boolean; -- Indicates correct low byte write conditions.
signal READWORD			: boolean; -- Indicates correct word read conditions.
signal READHI			: boolean; -- Indicates correct high byte read conditions.
signal READLOW			: boolean; -- Indicates correct low byte read conditions.
signal BUSY_INT			: bit;

-- Data processing signals:
type HALFTONE_TYPE is array(0 to 15) of bit_vector(15 downto 0);
signal HALFTONE_RAM		: HALFTONE_TYPE;
signal HALFTONE			: bit_vector(15 downto 0);
signal SRC_BUFF			: bit_vector(31 downto 0);
signal HOP_SRC			: bit_vector(15 downto 0);
signal LOGOP_SRC		: bit_vector(15 downto 0);
signal DEST_BUFF		: bit_vector(15 downto 0);
signal DEST_SRC			: bit_vector(15 downto 0);
signal NEW_DEST			: bit_vector(15 downto 0);
signal ENDMASK1			: bit_vector(15 downto 0);
signal ENDMASK2			: bit_vector(15 downto 0);
signal ENDMASK3			: bit_vector(15 downto 0);
signal DATAMASK			: bit_vector(15 downto 0);

-- Data processing controls:
signal LINENUMBER		: std_logic_vector(3 downto 0);
signal SRC_X_INCR		: std_logic_vector(15 downto 1);
signal SRC_Y_INCR		: std_logic_vector(15 downto 1);
signal SRC_ADR			: std_logic_vector(23 downto 1);
signal DEST_X_INCR		: std_logic_vector(15 downto 1);
signal DEST_Y_INCR		: std_logic_vector(15 downto 1);
signal DEST_ADR			: std_logic_vector(23 downto 1);
signal X_COUNT			: std_logic_vector(15 downto 0);
signal X_RELOAD			: std_logic_vector(15 downto 0); -- Reload value for X_COUNT.
signal Y_COUNT			: std_logic_vector(15 downto 0);
signal SKEW				: bit_vector(3 downto 0);
begin
	ADR_I <= To_BitVector(ADR_IN) & '0';
	SUPERUSER <= true when FC = "101" or FC = "110" else false;

	BLT_BSY <= '0' when Y_COUNT = x"0000" else '1';
	FORCE_DEST <= '1' when DATAMASK /= x"FFFF" else '0'; -- Force a read modify write cycle.
	BLT_RESTART <= '1' when ADR_I = x"FF8A3C" and WRITEHI = true and SUPERUSER = true and DATA_IN(15) = '1' else '0';
	XCNT_VALUE <= To_BitVector(X_COUNT);
	XCNT_RELOAD <= To_BitVector(X_RELOAD);

	-- BLT_RESTART pulls BUSY down
	BUSY <= BUSY_INT and not BLT_RESTART;

	-- Write registers:
	-- WRITEWORD is a control signal indicating correct conditions for the write access to the
	-- BLITTER registers.
	WRITEWORD <= true when LDSn = '0' and UDSn = '0' and ASn = '0' and RWn = '0' else false;
	WRITELOW  <= true when LDSn = '0' and ASn = '0' and RWn = '0' else false;
	WRITEHI   <= true when UDSn = '0' and ASn = '0' and RWn = '0' else false;

	MODIFY_REGISTERS: process(RESETn, CLK, CLK_EN_p)
	-- In this process all BLITTER registers are loaded by the data bus or are modified
	-- like incremeting or decrementing the counter registers.
	begin
		if RESETn = '0' then
			HALFTONE_RAM <= (others => (others => '0'));
			SRC_X_INCR <= (others => '0');
			SRC_Y_INCR <= (others => '0');
			SRC_ADR <= (others => '0');
			ENDMASK1 <= (others => '0');
			ENDMASK2 <= (others => '0');
			ENDMASK3 <= (others => '0');
			DEST_X_INCR <= (others => '0');
			DEST_Y_INCR <= (others => '0');
			DEST_ADR <= (others => '0');
			X_COUNT <= (others => '0');
			Y_COUNT <= (others => '0');
			HOP <= (others => '0');
			OP <= (others => '0');
			BUSY_INT <= '0';
			HOG <= '0';
			SMUDGE <= '0';
			LINENUMBER <= x"0";
			FXSR <= '0';
			NFSR <= '0';
			SKEW <= x"0";
		elsif rising_edge(CLK) and CLK_EN_p = '1' then
			-- Loading the registers from the data bus - write access:
			if 	WRITEWORD = true and SUPERUSER = true then
				case ADR_I is
					when x"FF8A00"	=> HALFTONE_RAM(0) <= To_BitVector(DATA_IN);
					when x"FF8A02"	=> HALFTONE_RAM(1) <= To_BitVector(DATA_IN);
					when x"FF8A04"	=> HALFTONE_RAM(2) <= To_BitVector(DATA_IN);
					when x"FF8A06"	=> HALFTONE_RAM(3) <= To_BitVector(DATA_IN);
					when x"FF8A08"	=> HALFTONE_RAM(4) <= To_BitVector(DATA_IN);
					when x"FF8A0A"	=> HALFTONE_RAM(5) <= To_BitVector(DATA_IN);
					when x"FF8A0C"	=> HALFTONE_RAM(6) <= To_BitVector(DATA_IN);
					when x"FF8A0E"	=> HALFTONE_RAM(7) <= To_BitVector(DATA_IN);
					when x"FF8A10"	=> HALFTONE_RAM(8) <= To_BitVector(DATA_IN);
					when x"FF8A12"	=> HALFTONE_RAM(9) <= To_BitVector(DATA_IN);
					when x"FF8A14"	=> HALFTONE_RAM(10) <= To_BitVector(DATA_IN);
					when x"FF8A16"	=> HALFTONE_RAM(11) <= To_BitVector(DATA_IN);
					when x"FF8A18"	=> HALFTONE_RAM(12) <= To_BitVector(DATA_IN);
					when x"FF8A1A"	=> HALFTONE_RAM(13) <= To_BitVector(DATA_IN);
					when x"FF8A1C"	=> HALFTONE_RAM(14) <= To_BitVector(DATA_IN);
					when x"FF8A1E"	=> HALFTONE_RAM(15) <= To_BitVector(DATA_IN);
					when x"FF8A20"	=> SRC_X_INCR <= DATA_IN(15 downto 1);
					when x"FF8A22"	=> SRC_Y_INCR <= DATA_IN(15 downto 1);
					when x"FF8A24"	=> SRC_ADR(23 downto 16) <= DATA_IN(7 downto 0);
					when x"FF8A26"	=> SRC_ADR(15 downto 1) <= DATA_IN(15 downto 1);
					when x"FF8A28"	=> ENDMASK1 <= To_BitVector(DATA_IN);
					when x"FF8A2A"	=> ENDMASK2 <= To_BitVector(DATA_IN);
					when x"FF8A2C"	=> ENDMASK3 <= To_BitVector(DATA_IN);
					when x"FF8A2E"	=> DEST_X_INCR <= DATA_IN(15 downto 1);
					when x"FF8A30"	=> DEST_Y_INCR <= DATA_IN(15 downto 1);
					when x"FF8A32"	=> DEST_ADR(23 downto 16) <= DATA_IN(7 downto 0);
					when x"FF8A34"	=> DEST_ADR(15 downto 1) <= DATA_IN(15 downto 1);
					-- In case of write access to the X_COUNT register there is a need
					-- to store the initial data to reload the correct value.
					when x"FF8A36"	=> X_COUNT <= DATA_IN;
									   X_RELOAD <= DATA_IN;
					when x"FF8A38"	=> Y_COUNT <= DATA_IN;
					when others		=> null;
				end case;
			end if;

			-- Byte wide access:
			-- The kind of modelling in four separate 'if' statements allows on the one hand
			-- word access and on the other hand byte wide access.
			if ADR_I = x"FF8A3A" and WRITEHI = true and SUPERUSER = true then -- FF8A3A.
				HOP <= To_BitVector(DATA_IN(9 downto 8));
			end if;
			if ADR_I = x"FF8A3A" and WRITELOW = true and SUPERUSER = true then -- FF8A3B.
				OP <= To_BitVector(DATA_IN(3 downto 0));
			end if;
			if ADR_I = x"FF8A3C" and WRITEHI = true and SUPERUSER = true then -- FF8A3C.
				HOG <= To_Bit(DATA_IN(14));
				SMUDGE <= To_Bit(DATA_IN(13));
				LINENUMBER <= DATA_IN(11 downto 8); -- Load via data bus access.
			-- Line number calculation:
			elsif X_COUNT_DEC = true and X_COUNT = x"0001" and DEST_Y_INCR(15) = '0' then
				LINENUMBER <= LINENUMBER + '1'; -- Increment for positive Y stepping.
			elsif X_COUNT_DEC = true and X_COUNT = x"0001" and DEST_Y_INCR(15) = '1' then
				LINENUMBER <= LINENUMBER - '1'; -- Decrement for negative Y stepping.
			end if;
			if ADR_I = x"FF8A3C" and WRITELOW = true and SUPERUSER = true then -- FF8A3D.
				FXSR <= To_Bit(DATA_IN(7));
				NFSR <= To_Bit(DATA_IN(6));
				SKEW <= To_BitVector(DATA_IN(3 downto 0));
			end if;

			-- Start-Stop condition:
			if ADR_I = x"FF8A3C" and WRITEHI = true and SUPERUSER = true then -- x"FF8A3C".
				if DATA_IN(15) = '1' and Y_COUNT > x"0000" then
					BUSY_INT <= '1'; -- Set via bus access. But only if there is work to do!
				end if;
			elsif Y_COUNT = x"0000" then
				-- Reset the BUSY flag after writing the last word of the last line.
				BUSY_INT <= '0';
			elsif BERRn = '0' then
				-- Reset, if there occurs a bus error.
				BUSY_INT <= '0';
			end if;

			-- Modification of the counter registers (higher priority than loading from the data bus):
			-- The x and y counters consist of two register pairs each (16 bits wide).
			-- During a write access from the data bus the offset and count registers are
			-- loaded together. During a read cycle the count register is read back. Thus
			-- the offset register is write only. During a reload, the offset is copied to
			-- the count register and can be read back by reading the counter in this moment.
			-- The Y_COUNT register is modified each reload of the X_COUNT register.
			if X_COUNT_DEC = true then
				case X_COUNT is
					when x"0001" =>
						X_COUNT <= X_RELOAD;
						Y_COUNT <= Y_COUNT - '1';
					when others =>
						-- PRE-FETCH of the source data does not take any extra counting effect.
						X_COUNT <= X_COUNT - '1';
				end case;
			elsif BERRn = '0' then
				-- Clear the progress status counters when there is a bus error.
				X_COUNT <= x"0000";
				Y_COUNT <= x"0000";
			end if;

			-- Source address modifications (higher priority than loading from the data bus).
			-- The source increment registers are signed (2's complement).
			-- For a FORCE_X explanation see the control file.
			if SRCADR_MODIFY = true and (X_COUNT /= x"0001" or FORCE_X = true) then
				-- If the line counter is not at the end of a line, the modification ratio
				-- is taken from the SRC_X_INC register.
				if SRC_X_INCR(15) = '0' then -- Positive value.
					SRC_ADR <= SRC_ADR + SRC_X_INCR(14 downto 1);
				else -- Negative value.
					SRC_ADR <= SRC_ADR - not SRC_X_INCR(14 downto 1) - '1';
				end if;
			elsif SRCADR_MODIFY = true then
				-- At the end of the line, the modification ratio is taken from the SRC_Y_INC
				-- register.
				if SRC_Y_INCR(15) = '0' then -- Positive value.
					SRC_ADR <= SRC_ADR + SRC_Y_INCR(14 downto 1);
				else -- Negative value.
					SRC_ADR <= SRC_ADR - not SRC_Y_INCR(14 downto 1) - '1';
				end if;
			end if;

			-- Destination address modifications (higher priority than loading from the data bus):
			-- The destination increment registers are signed (2's complement).
			if DESTADR_MODIFY = true and X_COUNT /= x"0001" then
				-- If the line counter is not at the end of a line, the modification ratio
				-- is taken from the SRC_X_INC register.
				if DEST_X_INCR(15) = '0' then -- Positive value.
					DEST_ADR <= DEST_ADR + DEST_X_INCR(14 downto 1);
				else -- Negative value.
					DEST_ADR <= DEST_ADR - not DEST_X_INCR(14 downto 1) - '1';
				end if;
			elsif DESTADR_MODIFY = true then
				if DEST_Y_INCR(15) = '0' then -- Positive value.
					DEST_ADR <= DEST_ADR + DEST_Y_INCR(14 downto 1);
				else -- Negative value.
					DEST_ADR <= DEST_ADR - not DEST_Y_INCR(14 downto 1) - '1';
				end if;
			end if;
		end if;
	end process MODIFY_REGISTERS;

	-- Adress output:
	ADR_OUT <= To_BitVector(SRC_ADR) when ADR_SEL = '1' else To_BitVector(DEST_ADR);
	
	-- Read registers:
	-- READWORD is a control signal indicating correct conditions for the read access from the
	-- BLITTER registers. Read access is possible even if there is no superuser access.
	READWORD <= true when LDSn = '0' and UDSn = '0' and ASn = '0' and RWn = '1' else false;
	READLOW  <= true when LDSn = '0' and ASn = '0' and RWn = '1' else false;
	READHI   <= true when UDSn = '0' and ASn = '0' and RWn = '1' else false;
	DATA_EN  <= '1' when READWORD = true and ADR_I >= x"FF8A00" and ADR_I <= x"FF8A38" else
			    '1' when (READLOW = true or READHI = true) and ADR_I >= x"FF8A3A" and ADR_I <= x"FF8A3C" else
			    '1' when PUSHDEST = '1' else '0'; 
	DATA_OUT <= HALFTONE_RAM(0)	when ADR_I = x"FF8A00" and READWORD = true else
				HALFTONE_RAM(1)	when ADR_I = x"FF8A02" and READWORD = true else
				HALFTONE_RAM(2)	when ADR_I = x"FF8A04" and READWORD = true else
				HALFTONE_RAM(3)	when ADR_I = x"FF8A06" and READWORD = true else
				HALFTONE_RAM(4)	when ADR_I = x"FF8A08" and READWORD = true else
				HALFTONE_RAM(5)	when ADR_I = x"FF8A0A" and READWORD = true else
				HALFTONE_RAM(6)	when ADR_I = x"FF8A0C" and READWORD = true else
				HALFTONE_RAM(7)	when ADR_I = x"FF8A0E" and READWORD = true else
				HALFTONE_RAM(8)	when ADR_I = x"FF8A10" and READWORD = true else
				HALFTONE_RAM(9)	when ADR_I = x"FF8A12" and READWORD = true else
				HALFTONE_RAM(10) when ADR_I = x"FF8A14" and READWORD = true else
				HALFTONE_RAM(11) when ADR_I = x"FF8A16" and READWORD = true else
				HALFTONE_RAM(12) when ADR_I = x"FF8A18" and READWORD = true else
				HALFTONE_RAM(13) when ADR_I = x"FF8A1A" and READWORD = true else
				HALFTONE_RAM(14) when ADR_I = x"FF8A1C" and READWORD = true else
				HALFTONE_RAM(15)	when ADR_I = x"FF8A1E" and READWORD = true else
				To_BitVector(SRC_X_INCR) & '0' when ADR_I = x"FF8A20" and READWORD = true else
				To_BitVector(SRC_Y_INCR) & '0' when ADR_I = x"FF8A22" and READWORD = true else
				x"00" & To_BitVector(SRC_ADR(23 downto 16)) when ADR_I = x"FF8A24" and READWORD = true else
				To_BitVector(SRC_ADR(15 downto 1)) & '0' when ADR_I = x"FF8A26" and READWORD = true else
				ENDMASK1 when ADR_I = x"FF8A28" and READWORD = true else
				ENDMASK2 when ADR_I = x"FF8A2A" and READWORD = true else
				ENDMASK3 when ADR_I = x"FF8A2C" and READWORD = true else
				To_BitVector(DEST_X_INCR) & '0' when ADR_I = x"FF8A2E" and READWORD = true else
				To_BitVector(DEST_Y_INCR) & '0' when ADR_I = x"FF8A30" and READWORD = true else
				x"00" & To_BitVector(DEST_ADR(23 downto 16)) when ADR_I = x"FF8A32" and READWORD = true else
				To_BitVector(DEST_ADR(15 downto 1)) & '0' when ADR_I = x"FF8A34" and READWORD = true else
				To_BitVector(X_COUNT) when ADR_I = x"FF8A36" and READWORD = true else
				To_BitVector(Y_COUNT) when ADR_I = x"FF8A38" and READWORD = true else
				-- Word access for byte wide registers:
				"000000" & HOP & x"0" & OP when ADR_I = x"FF8A3A" and READWORD = true else
				BUSY & HOG & SMUDGE & '0' & To_BitVector(LINENUMBER) & FXSR & NFSR & "00" & SKEW when ADR_I = x"FF8A3C" and READWORD = true else
				-- Byte wide access:
				"000000" & HOP & x"00" when ADR_I = x"FF8A3A" and READHI = true else  -- x"FF8A3A".
				x"000" & OP when ADR_I = x"FF8A3A" and READLOW = true else -- x"FF8A3B".
				BUSY & HOG & SMUDGE & '0' & To_BitVector(LINENUMBER) & x"00" when ADR_I = x"FF8A3C" and READHI = true else -- x"FF8A3C".
				x"00" & FXSR & NFSR & "00" & SKEW when ADR_I = x"FF8A3C" and READLOW = true else -- x"FF8A3D".
				-- Write modified video data to the memory:
				NEW_DEST when PUSHDEST = '1' else (others => '0');

	-- Source data processing:
	-- This process works on the negative clock edge!
	P_SRC_BUFF: process(RESETn, CLK, CLK_EN_n, SKEW, SRC_BUFF)
	begin
		if RESETn = '0' then
			SRC_BUFF <= (others => '0');
		elsif rising_edge(CLK) and CLK_EN_n = '1' then
			if FETCHSRC = '1' and SRC_X_INCR(15) = '0' then -- Positive SRC_X_INCR value.
				SRC_BUFF <= SRC_BUFF(15 downto 0) & To_BitVector(DATA_IN);
			elsif FETCHSRC = '1' and SRC_X_INCR(15) = '1' then -- Negative SRC_X_INCR value.
				SRC_BUFF <= To_BitVector(DATA_IN) & SRC_BUFF(31 downto 16);
			elsif SWAPSRC = '1' and SRC_X_INCR(15) = '0' then -- Positive SRC_X_INCR value.
				SRC_BUFF <= SRC_BUFF(15 downto 0) & x"0000";
			elsif SWAPSRC = '1' and SRC_X_INCR(15) = '1' then -- Negative SRC_X_INCR value.
				SRC_BUFF <= x"0000" & SRC_BUFF(31 downto 16);
			end if;
		end if;
		case SKEW is
			-- Dependant on the SKEW data there is an adjustment
			-- of n bits necessary, where n depends on the value 
			-- of SKEW.
			when x"0" => HOP_SRC <= SRC_BUFF(15 downto 0);
			when x"1" => HOP_SRC <= SRC_BUFF(16 downto 1);
			when x"2" => HOP_SRC <= SRC_BUFF(17 downto 2);
			when x"3" => HOP_SRC <= SRC_BUFF(18 downto 3);
			when x"4" => HOP_SRC <= SRC_BUFF(19 downto 4);
			when x"5" => HOP_SRC <= SRC_BUFF(20 downto 5);
			when x"6" => HOP_SRC <= SRC_BUFF(21 downto 6);
			when x"7" => HOP_SRC <= SRC_BUFF(22 downto 7);
			when x"8" => HOP_SRC <= SRC_BUFF(23 downto 8);
			when x"9" => HOP_SRC <= SRC_BUFF(24 downto 9);
			when x"A" => HOP_SRC <= SRC_BUFF(25 downto 10);
			when x"B" => HOP_SRC <= SRC_BUFF(26 downto 11);
			when x"C" => HOP_SRC <= SRC_BUFF(27 downto 12);
			when x"D" => HOP_SRC <= SRC_BUFF(28 downto 13);
			when x"E" => HOP_SRC <= SRC_BUFF(29 downto 14);
			when x"F" => HOP_SRC <= SRC_BUFF(30 downto 15);
		end case;
	end process P_SRC_BUFF;
	
	-- Halftone source data processing:
	HALFTONE <= HALFTONE_RAM(Conv_Integer(LINENUMBER)) when SMUDGE = '0' else -- Line number addressing.
				HALFTONE_RAM(Conv_Integer(To_StdLogicVector(HOP_SRC(3 downto 0)))); -- Source data addressing.

	P_HOP: process(HOP, HOP_SRC, HALFTONE)
	begin
		case HOP is
			when "11" => -- SOURCE and HALFTONE
				LOGOP_SRC <= HOP_SRC and HALFTONE;
			when "10" => -- SOURCE
				LOGOP_SRC <= HOP_SRC;
			when "01" => -- HALFTONE
				LOGOP_SRC <= HALFTONE;
			when "00" => -- ALL ONES
				LOGOP_SRC <= (others => '1');
		end case;
	end process P_HOP;
	
	P_DEST_BUFF: process(RESETn, CLK, CLK_EN_n)
	-- Store the old destination data in the data processing unit:
	-- This process works on the negative clock edge!
	begin
		if RESETn = '0' then
			DEST_BUFF <= (others => '0');
		elsif rising_edge(CLK) and CLK_EN_n = '1' then
			if FETCHDEST = '1' then
				DEST_BUFF <= To_BitVector(DATA_IN);
			end if;
		end if;
	end process P_DEST_BUFF;

	P_LOGOP: process(OP, LOGOP_SRC, DEST_BUFF)
	begin
		case OP is
			when x"0" => -- all zeros.
				DEST_SRC <= x"0000";
			when x"1" => -- Source and destination.
				DEST_SRC <= LOGOP_SRC and DEST_BUFF;
			when x"2" => -- Source and not destination.
				DEST_SRC <= LOGOP_SRC and not DEST_BUFF;
			when x"3" => -- Source.
				DEST_SRC <= LOGOP_SRC;
			when x"4" => -- Not source and destination.
				DEST_SRC <= not LOGOP_SRC and DEST_BUFF;
			when x"5" => -- Destination.
				DEST_SRC <= DEST_BUFF;
			when x"6" => -- Source xor destination.
				DEST_SRC <= LOGOP_SRC xor DEST_BUFF;
			when x"7" => -- Source or destination.
				DEST_SRC <= LOGOP_SRC or DEST_BUFF;
			when x"8" => -- Not source and not destination.
				DEST_SRC <= not LOGOP_SRC and not DEST_BUFF;
			when x"9" => -- Not source xor destination.
				DEST_SRC <= not LOGOP_SRC xor DEST_BUFF;
			when x"A" => -- Not destination.
				DEST_SRC <= not DEST_BUFF;
			when x"B" => -- Source or not destination.
				DEST_SRC <= LOGOP_SRC or not DEST_BUFF;
			when x"C" => -- Not source.
				DEST_SRC <= not LOGOP_SRC;
			when x"D" => -- Not source or destination.
				DEST_SRC <= not LOGOP_SRC or DEST_BUFF;
			when x"E" => -- Not source or not destination.
				DEST_SRC <= not LOGOP_SRC or not DEST_BUFF;
			when x"F" => -- all ones.
				DEST_SRC <= x"FFFF";
		end case;	
	end process P_LOGOP;

	DATAMASK <= ENDMASK1 when X_COUNT = X_RELOAD else -- First data word of a line or one word line.
				ENDMASK3 when X_COUNT = x"0001" else -- Last data word of a line.
				ENDMASK2; -- All other positions use the ENDMASK2.

	P_ENDMASK:process(RESETn, CLK, CLK_EN_n)
	-- To achieve a correct bus timing, this process must work
	-- on the negative clock edge!
	begin
		if RESETn = '0' then
			NEW_DEST <= (others => '0');
		elsif rising_edge(CLK) and CLK_EN_n = '1' then
			for i in 0 to 15 loop
				case DATAMASK(i) is
					when '1' =>	NEW_DEST(i) <= DEST_SRC(i); -- Write modified data.
					when others => NEW_DEST(i) <= DEST_BUFF(i); -- Do not modify destination data.
				end case;
			end loop;
		end if;
	end process P_ENDMASK;
end architecture BEHAVIOR;
