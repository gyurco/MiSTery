	component vidclkcntrl is
		port (
			inclk1x   : in  std_logic := 'X'; -- inclk1x
			inclk0x   : in  std_logic := 'X'; -- inclk0x
			clkselect : in  std_logic := 'X'; -- clkselect
			outclk    : out std_logic         -- outclk
		);
	end component vidclkcntrl;

	u0 : component vidclkcntrl
		port map (
			inclk1x   => CONNECTED_TO_inclk1x,   --  altclkctrl_input.inclk1x
			inclk0x   => CONNECTED_TO_inclk0x,   --                  .inclk0x
			clkselect => CONNECTED_TO_clkselect, --                  .clkselect
			outclk    => CONNECTED_TO_outclk     -- altclkctrl_output.outclk
		);

