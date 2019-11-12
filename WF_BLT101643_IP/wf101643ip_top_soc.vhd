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
---- Top level file for use in systems on programmable chips.     ----
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
--   Top level file provided for SOC (systems on programmable chips).
-- Revision 2K8A  2008/07/14 WF
--   Minor changes.
-- Revision 2K9A  2009/06/20 WF
--   DTACK_OUT_In has now synchronous reset to meet preset requirement.
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity WF101643IP_TOP_SOC is
	port (
		-- System controls:
		CLK			: in bit;
		RESETn		: in bit;
		AS_INn		: in bit;
		AS_OUTn		: out bit;
		LDS_INn		: in bit;
		LDS_OUTn	: out bit;
		UDS_INn		: in bit;
		UDS_OUTn	: out bit;
		RWn_IN		: in bit;
		RWn_OUT		: out bit;
		DTACK_INn	: in bit;
		DTACK_OUTn	: out bit;
		BERRn		: in bit;
		FC_IN		: in bit_vector(2 downto 0);
		FC_OUT		: out std_logic_vector(2 downto 0);
		BUSCTRL_EN	: out bit;
		INTn		: out bit;

		-- The bus:
		ADR_IN		: in bit_vector(23 downto 1);
		ADR_OUT		: out std_logic_vector(23 downto 1);
		ADR_EN		: out bit;
		DATA_IN		: in std_logic_vector(15 downto 0);
		DATA_OUT	: out std_logic_vector(15 downto 0);
		DATA_EN		: out bit;

		-- Bus arbitration:
		BGIn		: in bit;
		BGKIn		: in bit;
		BRn			: out bit;
		BGACK_INn	: in bit;
		BGACK_OUTn	: out bit;
		BGOn		: out bit
	);
end entity WF101643IP_TOP_SOC;

architecture STRUCTURE of WF101643IP_TOP_SOC is
component WF101643IP_CORE
port (  CLK				: in bit;
		RESETn			: in bit;
		ADR_IN			: in std_logic_vector(23 downto 1);
		ADR_OUT			: out bit_vector(23 downto 1);
		ADR_SEL			: in bit;
		ASn				: in bit;
		LDSn			: in bit;
		UDSn			: in bit;
		RWn				: in bit;
		FC				: in bit_vector(2 downto 0);
		BERRn			: in bit;
		DATA_IN			: in std_logic_vector(15 downto 0);
		DATA_OUT		: out bit_vector(15 downto 0);
		DATA_EN			: out bit;
		SWAPSRC			: in bit;
		FETCHSRC		: in bit;
		FETCHDEST		: in bit;
		PUSHDEST		: in bit;
		FORCE_X			: in boolean;
		SRCADR_MODIFY	: in boolean;
		DESTADR_MODIFY	: in boolean;
		X_COUNT_DEC		: in boolean;
		BLT_RESTART		: out bit;
		BLT_BSY			: out bit;
		XCNT_RELOAD		: out bit_vector(15 downto 0);
		XCNT_VALUE		: out bit_vector(15 downto 0);
		FORCE_DEST		: out bit;
		FXSR			: out bit;
		NFSR			: out bit;
		OP				: out bit_vector(3 downto 0);
		HOP				: out bit_vector(1 downto 0);
		HOG				: out bit;
		BUSY			: out bit;
		SMUDGE			: out bit
      );
end component;
component WF101643IP_CTRL
port (  CLK				: in bit;
		RESETn			: in bit;
		BERRn			: in bit;
		DTACKn			: in bit;
		AS_INn			: in bit;
		AS_OUTn			: out bit;
		UDSn			: out bit;
		LDSn			: out bit;
		RWn				: out bit;
		BUSCTRL_EN		: out bit;
		FC_OUT			: out bit_vector(2 downto 0);
		OP				: in bit_vector(3 downto 0);
		HOP				: in bit_vector(1 downto 0);
		HOG				: in bit;
		BUSY			: in bit;
		SMUDGE			: in bit;
		BGIn			: in bit;
		BGKIn			: in bit;
		ADR_SEL			: out bit;
		ADR_OUT_EN		: out bit;
		BLT_BSY			: in bit;
		FORCE_DEST		: in bit;
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
		X_COUNT_DEC		: out boolean;
		BRn				: out bit;
		BGACK_INn		: in bit;
		BGACK_OUTn		: out bit;
		BGOn			: out bit
      );
end component;

signal DTACK_In				: bit;
signal DTACK_LOCK			: boolean;
signal DTACK_OUT_In			: bit;
signal SU					: boolean;
signal ADR_IN_I				: std_logic_vector(23 downto 1);
signal ADR_OUT_I			: bit_vector(23 downto 1);
signal ADR_SEL_I			: bit;
signal ADR_OUT_EN_I			: bit;
signal DATA_OUT_I			: bit_vector(15 downto 0);
signal DATA_EN_I			: bit;
signal SWAPSRC_I			: bit;
signal FETCHSRC_I			: bit;
signal FETCHDEST_I			: bit;
signal PUSHDEST_I			: bit;
signal FORCE_X_I			: boolean;
signal SRCADR_MODIFY_I		: boolean;
signal DESTADR_MODIFY_I		: boolean;
signal BLT_RESTART_I		: bit;
signal X_COUNT_DEC_I		: boolean;
signal OP_I					: bit_vector(3 downto 0);
signal HOP_I				: bit_vector(1 downto 0);
signal HOG_I				: bit;
signal BUSY_I				: bit;
signal SMUDGE_I				: bit;
signal AS_OUT_In			: bit;
signal UDS_In				: bit;
signal LDS_In				: bit;
signal RWn_I				: bit;
signal BUSCTRL_EN_I			: bit;
signal FC_OUT_I				: bit_vector(2 downto 0);
signal BLT_BSY_I			: bit;
signal FORCE_DEST_I			: bit;
signal FXSR_I				: bit;
signal NFSR_I				: bit;
signal XCNT_RELOAD_I		: bit_vector(15 downto 0);
signal XCNT_VALUE_I			: bit_vector(15 downto 0);
begin
	ADR_IN_I	<= To_StdLogicVector(ADR_IN);
	ADR_OUT		<= To_StdLogicVector(ADR_OUT_I);
	ADR_EN		<= ADR_OUT_EN_I;
	DATA_OUT	<= To_StdLogicVector(DATA_OUT_I);
	DATA_EN		<= DATA_EN_I;
	AS_OUTn 	<= '0' when AS_OUT_In = '0' else '1';
	UDS_OUTn 	<= '0' when UDS_In = '0' else '1';
	LDS_OUTn 	<= '0' when LDS_In = '0' else '1';
	RWn_OUT 	<= '0' when RWn_I = '0' else '1';

	FC_OUT		<= To_StdLogicVector(FC_OUT_I);
	BUSCTRL_EN	<= BUSCTRL_EN_I;
	SU 			<= true when FC_IN = "101" or FC_IN = "110" else false; -- Superuser access.

	INTn 		<= BUSY_I;
    DTACK_In 	<= '0' when ADR_IN_I & '0' >= x"FF8A00" and ADR_IN_I & '0' <= x"FF8A3C" and AS_INn = '0' and SU = true else '1';

	P_DTACK_LOCK: process
	-- This process releases a data acknowledge detect, one rising clock
	-- edge after the DTACK_In occured. This is necessary to ensure write
	-- data to registers for there is one rising clock edge required.
	begin
		wait until CLK = '1' and CLK' event;
		if DTACK_In = '0' then
			DTACK_LOCK <= false;
		else
			DTACK_LOCK <= true;
		end if;
	end process P_DTACK_LOCK;

	DTACK_OUT: process
	-- The DTACKn port pin is released on the falling clock edge after the data
	-- acknowledge detect (DTACK_LOCK) is asserted. The DTACKn is deasserted
	-- immediately when there is no further register access DTACK_In = '1';
	begin
		wait until CLK = '0' and CLK' event;
		if RESETn = '0' then
			DTACK_OUT_In <= '1';
		elsif DTACK_In = '1' then
			DTACK_OUT_In <= '1';
		elsif DTACK_LOCK = false then
			DTACK_OUT_In <= '0';
		end if;
	end process DTACK_OUT;
	DTACK_OUTn <= '0' when DTACK_OUT_In = '0' else '1';

	I_CORE: WF101643IP_CORE
		port map(
			CLK				=> CLK,
			RESETn			=> RESETn,
			ADR_IN			=> ADR_IN_I,
			ADR_OUT			=> ADR_OUT_I,
			ADR_SEL			=> ADR_SEL_I,
			ASn				=> AS_INn,
			LDSn			=> LDS_INn,
			UDSn			=> UDS_INn,
			RWn				=> RWn_IN,
			FC				=> FC_IN,
			BERRn			=> BERRn,
			DATA_IN			=> DATA_IN,
			DATA_OUT		=> DATA_OUT_I,
			DATA_EN			=> DATA_EN_I,
			SWAPSRC			=> SWAPSRC_I,
			FETCHSRC		=> FETCHSRC_I,
			FETCHDEST		=> FETCHDEST_I,
			PUSHDEST		=> PUSHDEST_I,
			FORCE_X			=> FORCE_X_I,
			SRCADR_MODIFY	=> SRCADR_MODIFY_I,
			DESTADR_MODIFY	=> DESTADR_MODIFY_I,
			BLT_RESTART		=> BLT_RESTART_I,
			X_COUNT_DEC		=> X_COUNT_DEC_I,
			BLT_BSY			=> BLT_BSY_I,
			FORCE_DEST		=> FORCE_DEST_I,
			XCNT_RELOAD		=> XCNT_RELOAD_I,
			XCNT_VALUE		=> XCNT_VALUE_I,
			FXSR			=> FXSR_I,
			NFSR			=> NFSR_I,
			OP				=> OP_I,
			HOP				=> HOP_I,
			HOG				=> HOG_I,
			BUSY			=> BUSY_I,
			SMUDGE			=> SMUDGE_I
		);

	I_CTRL: WF101643IP_CTRL
		port map(
			CLK				=> CLK,
			RESETn			=> RESETn,
			BERRn			=> BERRn,
			DTACKn			=> DTACK_INn,
			AS_INn			=> AS_INn,
			AS_OUTn			=> AS_OUT_In,
			UDSn			=> UDS_In,
			LDSn			=> LDS_In,
			RWn				=> RWn_I,
			BUSCTRL_EN		=> BUSCTRL_EN_I,
			FC_OUT			=> FC_OUT_I,
			OP				=> OP_I,
			HOP				=> HOP_I,
			HOG				=> HOG_I,
			BUSY			=> BUSY_I,
			SMUDGE			=> SMUDGE_I,
			ADR_SEL			=> ADR_SEL_I,
			ADR_OUT_EN		=> ADR_OUT_EN_I,
			SWAPSRC			=> SWAPSRC_I,
			FETCHSRC		=> FETCHSRC_I,
			FETCHDEST		=> FETCHDEST_I,
			PUSHDEST		=> PUSHDEST_I,
			FORCE_X			=> FORCE_X_I,
			SRCADR_MODIFY	=> SRCADR_MODIFY_I,
			DESTADR_MODIFY	=> DESTADR_MODIFY_I,
			BLT_RESTART		=> BLT_RESTART_I,
			X_COUNT_DEC		=> X_COUNT_DEC_I,
			BLT_BSY			=> BLT_BSY_I,
			FORCE_DEST		=> FORCE_DEST_I,
			FXSR			=> FXSR_I,
			NFSR			=> NFSR_I,
			XCNT_RELOAD		=> XCNT_RELOAD_I,
			XCNT_VALUE		=> XCNT_VALUE_I,
			BRn				=> BRn,
			BGIn			=> BGIn,
			BGKIn			=> BGKIn,
			BGACK_INn		=> BGACK_INn,
			BGACK_OUTn		=> BGACK_OUTn,
			BGOn			=> BGOn
		);
end architecture STRUCTURE;