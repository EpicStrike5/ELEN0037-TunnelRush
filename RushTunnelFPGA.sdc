# RushTunnelFPGA.sdc
# Timing constraints for Cyclone IV EP4CE22F17C6 at 50 MHz

# Primary clock: 50 MHz on CLK_50 (period = 20 ns)
create_clock -name CLK_50 -period 20.0 [get_ports CLK_50]

# Derive uncertainty from PLL (no PLL in this design, but good practice)
derive_clock_uncertainty
