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
---- Top level file with component declarations.		          ----
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
---- Copyright (C) 2006 - 2008 Wolfgang Foerster                  ----
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
--   Changed several bus controls from open drain to tri state.
-- Revision 2K8B  2008/12/24 WF
--   Rewritten this top level file as a wrapper for the top_soc file.

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity WF101643IP_TOP is
	port (
		-- System controls:
		CLK		: in bit;
		RESETn	: in bit;
		ASn		: inout std_logic;
		LDSn	: inout std_logic;
		UDSn	: inout std_logic;
		RWn		: inout std_logic;
		DTACKn	: inout std_logic; -- Open drain.
		BERRn	: in bit;
		FC		: inout std_logic_vector(2 downto 0);
		INTn	: out bit;

		-- The bus:
		ADR		: inout std_logic_vector(23 downto 1);
		DATA	: inout std_logic_vector(15 downto 0);

		-- Bus arbitration:
		BGIn	: in bit;
		BGKIn	: in bit; -- Open drain.
		BRn		: out std_logic; -- Open drain.
		BGACKn	: inout std_logic; -- Open drain.
		BGOn	: out bit
	);
end entity WF101643IP_TOP;

architecture STRUCTURE of WF101643IP_TOP is
component WF101643IP_TOP_SOC
	port (
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
		ADR_IN		: in bit_vector(23 downto 1);
		ADR_OUT		: out std_logic_vector(23 downto 1);
		ADR_EN		: out bit;
		DATA_IN		: in std_logic_vector(15 downto 0);
		DATA_OUT	: out std_logic_vector(15 downto 0);
		DATA_EN		: out bit;
		BGIn		: in bit;
		BGKIn		: in bit;
		BRn			: out bit;
		BGACK_INn	: in bit;
		BGACK_OUTn	: out bit;
		BGOn		: out bit
	);
end component;
--
signal AS_INn	        : bit;
signal LDS_INn	        : bit;
signal UDS_INn	        : bit;
signal RWn_IN		    : bit;
signal AS_OUTn		    : bit;
signal LDS_OUTn	        : bit;
signal UDS_OUTn	        : bit;
signal RWn_OUT		    : bit;
signal DTACK_INn        : bit;
signal DTACK_OUTn       : bit;
signal BUSCTRL_EN_I     : bit;
signal FC_IN            : bit_vector(2 downto 0);
signal FC_OUT           : std_logic_vector(2 downto 0);
signal ADR_IN           : bit_vector(23 downto 1);
signal ADR_OUT          : std_logic_vector(23 downto 1);
signal ADR_EN_I         : bit;
signal DATA_OUT         : std_logic_vector(15 downto 0);
signal DATA_EN_I        : bit;
signal BR_In	        : bit;
signal BGACK_INn        : bit;
signal BGACK_OUTn       : bit;
begin
    AS_INn <= To_Bit(ASn);
    UDS_INn <= To_Bit(UDSn);
    LDS_INn <= To_Bit(LDSn);
    RWn_IN <= To_Bit(RWn);
    DTACK_INn <= To_Bit(DTACKn);
    FC_IN <= To_BitVector(FC);
    ADR_IN <= To_BitVector(ADR);
    BGACK_INn <= To_Bit(BGACKn);


    DTACKn <= '0' when DTACK_OUTn = '0' else 'Z'; -- Open drain.
    RWn <= '0' when RWn_OUT = '0' and BUSCTRL_EN_I = '1' else
           '1' when RWn_OUT = '1' and BUSCTRL_EN_I = '1' else'Z';
    ASn <= '0' when AS_OUTn = '0' and BUSCTRL_EN_I = '1' else
           '1' when AS_OUTn = '1' and BUSCTRL_EN_I = '1' else 'Z';
    LDSn <= '0' when LDS_OUTn = '0' and BUSCTRL_EN_I = '1' else
            '1' when LDS_OUTn = '1' and BUSCTRL_EN_I = '1' else 'Z';
    UDSn <= '0' when UDS_OUTn = '0' and BUSCTRL_EN_I = '1' else
            '1' when UDS_OUTn = '1' and BUSCTRL_EN_I = '1' else 'Z';
    FC <= FC_OUT when  BUSCTRL_EN_I = '1' else "ZZZ";

    ADR <= ADR_OUT when ADR_EN_I = '1' else (others => 'Z');
    DATA <= DATA_OUT when DATA_EN_I = '1' else (others => 'Z');

    BRn <= '0' when BR_In = '0' else 'Z';
    BGACKn <= '0' when BGACK_OUTn = '0' else 'Z';

    I_BLITTER: WF101643IP_TOP_SOC
        port map(CLK                => CLK,
                 RESETn             => RESETn,
                 AS_INn             => AS_INn,
                 AS_OUTn            => AS_OUTn,
                 LDS_INn            => LDS_INn,
                 LDS_OUTn           => LDS_OUTn,
                 UDS_INn            => UDS_INn,
                 UDS_OUTn           => UDS_OUTn,
                 RWn_IN             => RWn_IN,
                 RWn_OUT            => RWn_OUT,
                 DTACK_INn          => DTACK_INn,
                 DTACK_OUTn         => DTACK_OUTn,
                 BERRn		        => BERRn,
                 FC_IN		        => FC_IN,
                 FC_OUT		        => FC_OUT,
                 BUSCTRL_EN         => BUSCTRL_EN_I,
                 INTn		        => INTn,
                 ADR_IN		        => ADR_IN,
                 ADR_OUT		    => ADR_OUT,
                 ADR_EN		        => ADR_EN_I,
                 DATA_IN		    => DATA,
                 DATA_OUT	        => DATA_OUT,
                 DATA_EN		    => DATA_EN_I,
                 BGIn		        => BGIn,
                 BGKIn		        => BGKIn,
                 BRn			    => BR_In,
                 BGACK_INn	        => BGACK_INn,
                 BGACK_OUTn	        => BGACK_OUTn,
                 BGOn		        => BGOn
        );
end architecture STRUCTURE;