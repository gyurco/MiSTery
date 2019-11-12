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
---- Control unit with adress logic and bus arbitration.	      ----
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

entity WF101643IP_CTRL is
port (  CLK			: in bit;
		RESETn		: in bit;

		-- Bus control:
		BERRn		: in bit; -- Bus error.
		DTACKn		: in bit;
		AS_INn		: in bit;
		AS_OUTn		: out bit;
		UDSn		: out bit;
		LDSn		: out bit;
		RWn			: out bit;
		BUSCTRL_EN	: out bit;
		FC_OUT		: out bit_vector(2 downto 0);
		
		-- Bus arbitration:
		BGIn		: in bit;
		BGKIn		: in bit; -- DMA transfer acknowledge from the GLUE.
		BRn			: out bit;
		BGACK_INn	: in bit;
		BGACK_OUTn	: out bit;
		BGOn		: out bit;

		-- The flags:
		OP		: in bit_vector(3 downto 0);
		HOP		: in bit_vector(1 downto 0);
		HOG		: in bit;
		BUSY	: in bit;
		SMUDGE	: in bit;

		-- Adress counter controls (DMA mode).
		ADR_SEL			: out bit;
		ADR_OUT_EN		: out bit;

		-- Data processing controls:
		BLT_BSY			: in bit; -- '0' if Y counter is x"0000".
		FORCE_DEST		: in bit; -- Force a read modify write cycle.
		FXSR			: in bit;
		NFSR			: in bit;
		XCNT_RELOAD		: in bit_vector(15 downto 0);
		XCNT_VALUE		: in bit_vector(15 downto 0);
		BLT_RESTART		: in bit;
		SWAPSRC			: out bit;
		FETCHSRC		: out bit;
		FETCHDEST		: out bit;
		PUSHDEST		: out bit;
		FORCE_X			: out boolean;
		SRCADR_MODIFY	: out boolean;
		DESTADR_MODIFY	: out boolean;
		X_COUNT_DEC		: out boolean
      );
end WF101643IP_CTRL;

architecture BEHAVIOR of WF101643IP_CTRL is
type BLITTER_STATES is (IDLE, BUSREQUEST, SELECT_CMD, T1_RD_DEST, T1_WR, T1_CHECK, T2_RD_DEST, T2_RD_SRC_1,
						T2_RD_SRC_2, T2_WR, T2_CHECK, T3_RD_SRC_1, T3_RD_SRC_2, T3_RD_DEST, T3_WR, T3_CHECK);
type TIME_SLICES is (IDLE, S0, S1, S2, S3, S4, S5, S6, S7);
signal BLT_STATE 		: BLITTER_STATES;
signal NEXT_BLT_STATE 	: BLITTER_STATES;
signal BGACK_In			: bit;
signal BUS_FREE			: boolean;
signal SLICE_CNT		: std_logic_vector(2 downto 0);
signal T_SLICE			: TIME_SLICES;
signal UDS_WR_EN		: bit;
signal LDS_WR_EN		: bit;
signal UDS_RD_EN		: bit;
signal LDS_RD_EN		: bit;
signal AS_EN			: bit;
signal DATA_EN			: bit;
signal ADDRESS_EN		: bit;
signal WAITSTATES		: bit;
signal HOG_START		: bit;
signal HOG_STOP			: bit;
signal PRE_FETCH		: bit;
signal POST_FLUSH		: bit;
signal SKIP_CALC		: bit;

begin
	-- Declare the BLITTER video data as user data. This is achieved by a value of "001" for FC during
	-- active bus cycles. This functionality is necessary not to produce interrupt requests in the GLUE
	-- chip (FC = "111") during bus accesses.
	FC_OUT <= "001" when T_SLICE /= IDLE else "000";
	BUSCTRL_EN <= '1' when BLT_STATE /= IDLE and BLT_STATE /= BUSREQUEST else '0';

	-- Bus arbitration:
	BRn      	<=	'0' when BLT_STATE = BUSREQUEST else '1';
	BGOn     	<=	'0' when BGIn = '0' and BUSY = '0' else '1'; -- External bus request if BLITTER is not busy.
	BGACK_In 	<= 	'1' when BUSY = '0' else
					'1' when BUSY = '1' and BLT_STATE = IDLE else
					'1' when BUSY = '1' and BLT_STATE = BUSREQUEST and BUS_FREE = false else '0';
	BGACK_OUTn  <= 	'0' when BGKIn = '0' and BLT_STATE = IDLE else -- DMA acknowledge.
					'0' when BGACK_In = '0' else '1'; -- BLITTER acknowledge.
	-- Address controlling ('1' = SRC_ADR, '0' = DEST_ADR.):
	ADR_SEL  <= '0' when BLT_STATE = T1_RD_DEST else 	-- Read from destination.
				'0' when BLT_STATE = T1_WR else  		-- Write to destination.
				'0' when BLT_STATE = T2_RD_DEST else 	-- Read from destination.
				'0' when BLT_STATE = T2_WR else 		-- Write to destination.
				'0' when BLT_STATE = T3_RD_DEST else	-- Read from destination.
				'0' when BLT_STATE = T3_WR else	'1';	-- Source is default.
	ADR_OUT_EN   <= '1' when BLT_STATE = T1_RD_DEST and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T1_WR and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T2_RD_DEST and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T2_RD_SRC_1 and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T2_RD_SRC_2 and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T2_WR and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T3_RD_SRC_1 and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T3_RD_SRC_2 and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T3_RD_DEST and ADDRESS_EN = '1' else
					'1' when BLT_STATE = T3_WR and ADDRESS_EN = '1' else '0';
	AS_OUTn  <= '0' when BLT_STATE = T1_RD_DEST and AS_EN = '1' else
				'0' when BLT_STATE = T1_WR and AS_EN = '1' else
				'0' when BLT_STATE = T2_RD_DEST and AS_EN = '1' else
				'0' when BLT_STATE = T2_RD_SRC_1 and AS_EN = '1' else
				'0' when BLT_STATE = T2_RD_SRC_2 and AS_EN = '1' else
				'0' when BLT_STATE = T2_WR and AS_EN = '1' else
				'0' when BLT_STATE = T3_RD_SRC_1 and AS_EN = '1' else
				'0' when BLT_STATE = T3_RD_SRC_2 and AS_EN = '1' else
				'0' when BLT_STATE = T3_RD_DEST and AS_EN = '1' else
				'0' when BLT_STATE = T3_WR and AS_EN = '1' else '1';

	UDSn 	 <= '0' when BLT_STATE = T1_RD_DEST and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T1_WR and UDS_WR_EN = '1' else
				'0' when BLT_STATE = T2_RD_DEST and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T2_RD_SRC_1 and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T2_RD_SRC_2 and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T2_WR and UDS_WR_EN = '1' else
				'0' when BLT_STATE = T3_RD_SRC_1 and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T3_RD_SRC_2 and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T3_RD_DEST and UDS_RD_EN = '1' else
				'0' when BLT_STATE = T3_WR and UDS_WR_EN = '1' else '1';
	LDSn 	 <= '0' when BLT_STATE = T1_RD_DEST and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T1_WR and LDS_WR_EN = '1' else
				'0' when BLT_STATE = T2_RD_DEST and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T2_RD_SRC_1 and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T2_RD_SRC_2 and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T2_WR and LDS_WR_EN = '1' else
				'0' when BLT_STATE = T3_RD_SRC_1 and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T3_RD_SRC_2 and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T3_RD_DEST and LDS_RD_EN = '1' else
				'0' when BLT_STATE = T3_WR and LDS_WR_EN = '1' else '1';

	RWn 	 <= '0' when BLT_STATE = T1_WR else
				'0' when BLT_STATE = T2_WR else
				'0' when BLT_STATE = T3_WR else '1'; -- Default is read.

	-- NFSR / FXSR controls:
	PRE_FETCH <= '1' when XCNT_VALUE = XCNT_RELOAD and FXSR = '1' else '0'; -- Read twice at the beginning of a line.
	POST_FLUSH <= '1' when XCNT_VALUE = x"0001" and NFSR = '1' else '0'; -- Do not read the last data word of each line.

	-- The SKIP_CALC control covers several special conditions:
	-- During FXSR = '0' and NFSR = '1' (post flushing data), the second last source read must occur without
	-- source address modification to handle correct source address incrementing with SRC_Y_INCR.
	-- During FXSR = '1' and NFSR = '1' (pre fetching and post flushing data), a special case is the
	-- writing of just two destination words (XCNT_RELOAD = x"0002"). The second source read must happen
	-- in this case without source address modification.
	-- In case of XCNT_RELOAD less than x"0002" there will result unpredictionable behavior of the BLITTER,
	-- if both, FXSR and NFSR are asserted. Nevertheless these settings does not make sense.
	SKIP_CALC <= '1' when XCNT_VALUE = x"0002" and NFSR = '1' and FXSR = '1' and XCNT_RELOAD > x"0002" else
			 	 '1' when XCNT_VALUE = x"0002" and NFSR = '1' and FXSR = '0' else
			 	 '1' when XCNT_RELOAD = x"0002" and NFSR = '1' and FXSR = '1' and BLT_STATE = T2_RD_SRC_2 else
			 	 '1' when XCNT_RELOAD = x"0002" and NFSR = '1' and FXSR = '1' and BLT_STATE = T3_RD_SRC_2 else '0';

	-- The FORCE_X is a special case of the FXSR (Force Extra Source Read) mode:
	-- When a force extra source read is required and the first data word is read, the source address
	-- must in any case be incremented with the SRC_X_INCR value even if the value of XCNT_RELOAD is one.
	-- Means a source address calculation error would occur, because the XCNT_RELOAD value of one would 
	-- force the source address counter to increment with the SRC_Y_INCR value.
	FORCE_X <= 	true when BLT_STATE = T2_RD_SRC_1 and XCNT_VALUE = XCNT_RELOAD and FXSR = '1' else
				true when BLT_STATE = T3_RD_SRC_1 and XCNT_VALUE = XCNT_RELOAD and FXSR = '1' else false;

	SRCADR_MODIFY <= true when BLT_STATE = T2_RD_SRC_1 and SKIP_CALC = '0' and T_SLICE = S7 else
 					 true when BLT_STATE = T2_RD_SRC_2 and SKIP_CALC = '0' and T_SLICE = S7 else
					 true when BLT_STATE = T3_RD_SRC_1 and SKIP_CALC = '0' and T_SLICE = S7 else
					 true when BLT_STATE = T3_RD_SRC_2 and SKIP_CALC = '0' and T_SLICE = S7 else
					 -- Pay attention here to assert the control in odd time slices due to the
					 -- rising edge of the register process in the core file!
					 true when BLT_STATE = T2_WR and OP = x"3" and POST_FLUSH = '1' and T_SLICE = S1 else
					 true when BLT_STATE = T2_WR and OP = x"C" and POST_FLUSH = '1' and T_SLICE = S1 else
					 true when BLT_STATE = T3_WR and POST_FLUSH = '1' and T_SLICE = S1 else false;

	-- Fetch the source data in the end of the time cycle S6. This is
	-- rather late but ensures valid data on the bus. The even time slices
	-- require the source process working on the negative clock edge!
	FETCHSRC <= '1' when BLT_STATE = T2_RD_SRC_1 and T_SLICE = S6 else
			    '1' when BLT_STATE = T2_RD_SRC_2 and T_SLICE = S6 else
			   	'1' when BLT_STATE = T3_RD_SRC_1 and T_SLICE = S6 else
			   	'1' when BLT_STATE = T3_RD_SRC_2 and T_SLICE = S6 else '0';

	-- The following control provides source buffer swapping without bus read access. This is necessary
	-- for some combinations of NFSR, FXSR and SKEW. The even time slices require the source process
	-- working on the negative clock edge!
	SWAPSRC <=	'1' when BLT_STATE = T2_WR and OP = x"3" and POST_FLUSH = '1' and T_SLICE = S0 else
				'1' when BLT_STATE = T2_WR and OP = x"C" and POST_FLUSH = '1' and T_SLICE = S0 else
				'1' when BLT_STATE = T3_WR and POST_FLUSH = '1' and T_SLICE = S0 else '0';

	DESTADR_MODIFY	 <= true when BLT_STATE = T1_WR and T_SLICE = S7 else
						true when BLT_STATE = T2_WR and T_SLICE = S7 else
						true when BLT_STATE = T3_WR and T_SLICE = S7 else false;
						
	-- Fetch the destination data in the end of the time cycle S6. This is
	-- rather late but ensures valid data on the bus. The even time slices 
	-- require the destination process working on the negative clock edge!
	FETCHDEST 		<= '1' when BLT_STATE = T1_RD_DEST and T_SLICE = S6 else
					   '1' when BLT_STATE = T2_RD_DEST and T_SLICE = S6 else
					   '1' when BLT_STATE = T3_RD_DEST and T_SLICE = S6 else '0';
					
	PUSHDEST 		<= '1' when BLT_STATE = T1_WR and DATA_EN = '1' else
					   '1' when BLT_STATE = T2_WR and DATA_EN = '1' else
					   '1' when BLT_STATE = T3_WR and DATA_EN = '1' else '0';

	X_COUNT_DEC	 <= true when BLT_STATE = T1_WR and T_SLICE = S7 else
					true when BLT_STATE = T2_WR and T_SLICE = S7 else
					true when BLT_STATE = T3_WR and T_SLICE = S7 else false;
	
	P_CYCLE_CNT: process(RESETn, CLK)
	-- This process provides counting the read or read modify write cycles. This is required
	-- for the HOG = '0' operation. After 64 clock cycles, the BLITTER stops operation and releases
	-- the bus. It is restarted again by setting the BUSY flag or after HOG_START.
	-- Against the original timing, the BLITTER is implemented here with another timing. This
	-- speeds slightly up the bit block transfer operations.
	-- For further details see the Atari related bit block transfer processor documentation.
	variable CYCLE_CNT	: std_logic_vector(7 downto 0);
	begin
		if RESETn = '0' then
			CYCLE_CNT := (others => '0');
			HOG_STOP <= '0';
			HOG_START <= '0';
		elsif CLK = '1' and CLK' event then
			if BLT_STATE = BUSREQUEST or HOG = '1' then
				-- Initialize, if BLITTER enters arbitration 
				-- or HOG is off.
				CYCLE_CNT := (others => '0');
			elsif CYCLE_CNT < x"80" then
				CYCLE_CNT := CYCLE_CNT + '1';
			end if;
			-- Original Timing: if CYCLE_CNT = x"80" then -- Release the bus for 64 CLK cycles.
			if CYCLE_CNT = x"C0" then -- Release the bus for 64 CLK cycles.
				HOG_START <= '1';
			-- Original Timing: elsif CYCLE_CNT >= x"40" then -- Hogging 64 CLK cycles.
			elsif CYCLE_CNT >= x"80" then -- Hogging 128 CLK cycles.
				HOG_STOP <= '1';
			else
				HOG_STOP <= '0';
				HOG_START <= '0';
			end if;
		end if;
	end process P_CYCLE_CNT;

	BLT_STATE_MEM: process(RESETn, CLK)
	-- Main state machine register.
	begin
		if RESETn = '0' then
			BLT_STATE <= IDLE;
		elsif CLK = '1' and CLK' event then
			if BERRn = '0' then
				BLT_STATE <= IDLE; -- Break!
			else
				BLT_STATE <= NEXT_BLT_STATE;
			end if;
		end if;
	end process BLT_STATE_MEM;
	
	BLT_STATE_LOGIC: process(BLT_STATE, BUSY, BUS_FREE, HOG, HOG_START, HOG_STOP, BLT_RESTART, BLT_BSY,
										OP, HOP, FORCE_DEST, SMUDGE, T_SLICE, BGKIn, PRE_FETCH, POST_FLUSH)
	begin
		case BLT_STATE is
			when IDLE =>
				-- The BLITTER state machine starts if there is a request
				-- and if there is no external DMA request (BGKIn).
				if HOG = '1' and BUSY = '1' and BGKIn = '1' then
					-- Start in HOG mode after BUSY is set:
					NEXT_BLT_STATE <= BUSREQUEST;
				elsif HOG = '0' and BUSY = '1' and HOG_START = '1' and BGKIn = '1' then
					-- Restart in cooperative mode after cycle enable:
					NEXT_BLT_STATE <= BUSREQUEST;
				elsif HOG = '0' and BUSY = '1' and BLT_RESTART = '1' and BGKIn = '1' then
					-- Restart in cooperative mode by processor launch (immediately):
					-- The BGKIn locking prevents the BLITTER to be restarted by 
					-- other DMA devices than the CPU.
					NEXT_BLT_STATE <= BUSREQUEST;
				else
					NEXT_BLT_STATE <= IDLE;
				end if;
			when BUSREQUEST =>
				if BUS_FREE = true then -- The bus is now free for data transfer.
					NEXT_BLT_STATE <= SELECT_CMD;
				else
					NEXT_BLT_STATE <= BUSREQUEST;
				end if;
			when SELECT_CMD =>
				-- Type1 commands are characterized by writing just a fixed value '0' or '1' or halftone
				-- patterns to the destination.
				-- Type2 commands need the destination data or the source data for new data processing.
				-- Type 3 commands need all, the source and destination (and halftone) data for new data
				-- processing. 
				-- The determination which kind of data is necessary for the bit block transfer operation
				-- is done exclusively here because a running bit block transfer uses always the same
				-- type of data which can only change after wrapping the 'IDLE' state.
				case OP is
					when x"0" | x"F" =>
						if FORCE_DEST = '1' then
							 NEXT_BLT_STATE <= T1_RD_DEST; -- Data required.
						else
							 NEXT_BLT_STATE <= T1_WR; -- No data required.
						end if;
					when x"5" | x"A" => NEXT_BLT_STATE <= T2_RD_DEST; -- Destination data processing.
					when x"3" | x"C" =>
						case HOP is
							when "00" | "01" => -- Halftone processing.
								if SMUDGE = '1' then
									if FORCE_DEST = '1' then
										 NEXT_BLT_STATE <= T2_RD_DEST; -- Destination data required.
									elsif POST_FLUSH = '1' then
										 NEXT_BLT_STATE <= T2_WR; -- Skip source data read.
									else
										 NEXT_BLT_STATE <= T2_RD_SRC_1; -- Source data required.
									end if;
								else
									if FORCE_DEST = '1' then
										NEXT_BLT_STATE <= T1_RD_DEST; -- Destination data required.
									else
										NEXT_BLT_STATE <= T1_WR; -- Only halftone data required.
									end if;
								end if;
							when others => -- Sorce data processing.
								if FORCE_DEST = '1' then
									 NEXT_BLT_STATE <= T2_RD_DEST; -- Destination data required.
								elsif POST_FLUSH = '1' then
									 NEXT_BLT_STATE <= T2_WR; -- Skip source data read.
								else
									 NEXT_BLT_STATE <= T2_RD_SRC_1; -- Source data required.
								end if;
						end case;
					when others =>
						case HOP is
							when "00" | "01" => -- Halftone processing.
								if SMUDGE = '1' and POST_FLUSH = '0' then
									NEXT_BLT_STATE <= T3_RD_SRC_1; -- All data required.
								else
									NEXT_BLT_STATE <= T2_RD_DEST; -- Halftone and destination data required.								
								end if;
							when others =>
								if POST_FLUSH = '1' then
									NEXT_BLT_STATE <= T3_RD_DEST; -- Skip source data read.
								else
									NEXT_BLT_STATE <= T3_RD_SRC_1; -- All data required.
								end if;
						end case;
				end case;
			---------------------------------- TYPE I COMMANDS -----------------------------------------
			when T1_RD_DEST =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T1_WR; -- Read cycle finished.
				else
					NEXT_BLT_STATE <= T1_RD_DEST;
				end if;
			when T1_WR =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T1_CHECK; -- Write cycle finished.
				else
					NEXT_BLT_STATE <= T1_WR;
				end if;
			when T1_CHECK =>
				-- The BLT state machine is interrupted when there is an external bus request from a
				-- DMA device (BGKIn = '0'). The BGACK_OUTn is delayed in this case until the BLT
				-- state machine has entered it's IDLE state.
				if HOG_STOP = '1' or BLT_BSY = '0' or BGKIn = '0' then
					NEXT_BLT_STATE <= IDLE; -- BLITTER finished.
				elsif FORCE_DEST = '1' then
					 NEXT_BLT_STATE <= T1_RD_DEST; -- Data required.
				else
					 NEXT_BLT_STATE <= T1_WR; -- No data required.
				end if;
			---------------------------------- TYPE II COMMANDS -----------------------------------------
			when T2_RD_DEST =>
				if T_SLICE = S7 then 
					case OP is
						when x"3" | x"C" =>	-- Go on processing source data.
							if POST_FLUSH = '0' then
								NEXT_BLT_STATE <= T2_RD_SRC_1;
							else -- Post-flush source data:
								NEXT_BLT_STATE <= T2_WR; -- Do not read further source data.
							end if;
						when others =>	NEXT_BLT_STATE <= T2_WR; -- Read cycle finished.
					end case;
				else
					NEXT_BLT_STATE <= T2_RD_DEST;
				end if;
			when T2_RD_SRC_1 =>
				if T_SLICE = S7 and PRE_FETCH = '1' then 
					NEXT_BLT_STATE <= T2_RD_SRC_2; -- Read two words at the beginning of the line.
				elsif T_SLICE = S7 then 
					NEXT_BLT_STATE <= T2_WR; -- Read cycle finished by no final source read.
				else
					NEXT_BLT_STATE <= T2_RD_SRC_1;
				end if;
			when T2_RD_SRC_2 =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T2_WR; -- Read cycle finished.
				else
					NEXT_BLT_STATE <= T2_RD_SRC_2;
				end if;
			-- The complete data processing delay of the BLITTER core is exactly one clock
			-- cycle. Therefore there is no need for an extra delay between reading and
			-- writing the modified video data.
			when T2_WR =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T2_CHECK; -- Write cycle finished.
				else
					NEXT_BLT_STATE <= T2_WR;
				end if;
			when T2_CHECK =>
				if HOG_STOP = '1' or BLT_BSY = '0' or BGKIn = '0' then
					NEXT_BLT_STATE <= IDLE; -- BLITTER finished.
				else
					case OP is
						when x"3" | x"C" =>	-- Go on processing source data.
							if FORCE_DEST = '1' then
								 NEXT_BLT_STATE <= T2_RD_DEST; -- Destination data required.
							elsif POST_FLUSH = '0' then
								NEXT_BLT_STATE <= T2_RD_SRC_1;
							else -- Post-flush source data:
								NEXT_BLT_STATE <= T2_WR; -- Do not read further source data.
							end if;
						when others =>	NEXT_BLT_STATE <= T2_RD_DEST; -- Go on processing destination / halftone.
					end case;
				end if;
			---------------------------------- TYPE III COMMANDS -----------------------------------------
			when T3_RD_SRC_1 =>
				if T_SLICE = S7 and PRE_FETCH = '1' then 
					NEXT_BLT_STATE <= T3_RD_SRC_2; -- Read two words at the beginning of the line.
				elsif T_SLICE = S7 then 
					NEXT_BLT_STATE <= T3_RD_DEST; -- Source read cycle finished.
				else
					NEXT_BLT_STATE <= T3_RD_SRC_1;
				end if;
			when T3_RD_SRC_2 =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T3_RD_DEST; -- Source read cycle finished.
				else
					NEXT_BLT_STATE <= T3_RD_SRC_2;
				end if;
			when T3_RD_DEST =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T3_WR; -- Read cycle finished.
				else
					NEXT_BLT_STATE <= T3_RD_DEST;
				end if;
			-- The complete data processing delay of the BLITTER core is exactly one clock
			-- cycle. Therefore there is no need for an extra delay between reading and
			-- writing the modified video data.
			when T3_WR =>
				if T_SLICE = S7 then 
					NEXT_BLT_STATE <= T3_CHECK; -- Write cycle finished.
				else
					NEXT_BLT_STATE <= T3_WR;
				end if;
			when T3_CHECK =>
				if HOG_STOP = '1' or BLT_BSY = '0' or BGKIn = '0' then
					NEXT_BLT_STATE <= IDLE; -- BLITTER finished.
				elsif POST_FLUSH = '1' then
					NEXT_BLT_STATE <= T3_RD_DEST; -- Post-flush source data.
				else
					NEXT_BLT_STATE <= T3_RD_SRC_1; -- Go on, cycle not finished.
				end if;
		end case;
	end process BLT_STATE_LOGIC;

	BUS_REQUEST: process
	begin
		wait until CLK = '1' and CLK' event;
		-- BUSYn is used to distinguish between internal or external bus requests.
		if BUSY = '1' and BGIn = '0' and BGKIn = '1' and BGACK_INn = '1' and AS_INn = '1' and DTACKn = '1' then
			BUS_FREE <= true;
		else
			BUS_FREE <= false;
		end if;
	end process BUS_REQUEST;

	P_WAITSTATES: process
	-- During read or write, the bus access is delayed by wait states (slow read) if
	-- there is no DTACKn asserted until the end of S4. This is done by stopping the slice
	-- counter. After the halt, in principle a S5 would be possible. This is not correct
	-- for not asserted DTACKn. This process provides a locking of this forbidden case and the
	-- stop control for the slice counter. For more information see the 68000 processor data 
	-- sheet (bus cycles). The process is required for the cycle accurate switching of T_SLICE.
	begin
		wait until CLK = '0' and CLK' event;
		case SLICE_CNT is
			when "010" => WAITSTATES <= DTACKn;
			when others => WAITSTATES <= '0';
		end case;
	end process P_WAITSTATES;

	SLICES: process(RESETn, CLK)
	begin
		if RESETn = '0' then
			SLICE_CNT <= "111";
		elsif CLK = '1' and CLK' event then
			case BLT_STATE is
				when T1_RD_DEST | T1_WR | T2_RD_DEST | T2_RD_SRC_1 | T2_RD_SRC_2 | T2_WR | T3_RD_SRC_1 |
			 	 											 			T3_RD_SRC_2 | T3_RD_DEST | T3_WR =>
					if SLICE_CNT = "011" then
						-- The counter modeled in this way does not need any clock cycles for
						-- re-initialization. A direct read write operation is possible.
						SLICE_CNT <= "111";
					elsif WAITSTATES = '0' then
						SLICE_CNT <= SLICE_CNT + '1';
					end if;
				when others => SLICE_CNT <= "111"; -- IDLE.
			end case;
		end if;
	end process SLICES;

	T_SLICE <=  S0 when SLICE_CNT = "000" and CLK = '1' else
			    S1 when SLICE_CNT = "000" and CLK = '0' else
			    S2 when SLICE_CNT = "001" and CLK = '1' else
			    S3 when SLICE_CNT = "001" and CLK = '0' else
			    S4 when SLICE_CNT = "010" and CLK = '1' else
				S4 when SLICE_CNT = "010" and WAITSTATES = '1' and CLK = '0' else
			    S5 when SLICE_CNT = "010" and WAITSTATES = '0' and CLK = '0' else
			    S6 when SLICE_CNT = "011" and CLK = '1' else
			    S7 when SLICE_CNT = "011" and CLK = '0' else IDLE;

	-- UDSn and LDSn are always asserted together for the read and write access
	-- to the source and destination is always word wide. Read and write controls
	-- have different timings!
	with T_SLICE select UDS_WR_EN  <= '1' when 			 S4 | S5 | S6, 		'0' when others;
	with T_SLICE select LDS_WR_EN  <= '1' when 			 S4 | S5 | S6, 		'0' when others;
	with T_SLICE select UDS_RD_EN  <= '1' when S2 | S3 | S4 | S5 | S6, 		'0' when others;
	with T_SLICE select LDS_RD_EN  <= '1' when S2 | S3 | S4 | S5 | S6, 		'0' when others;
	with T_SLICE select AS_EN      <= '1' when S2 | S3 | S4 | S5 | S6, 		'0' when others;
	with T_SLICE select DATA_EN    <= '1' when 		S3 | S4 | S5 | S6 | S7, '0' when others;
	with T_SLICE select ADDRESS_EN <= '0' when IDLE | S0,              		'1' when others;
end architecture BEHAVIOR;
