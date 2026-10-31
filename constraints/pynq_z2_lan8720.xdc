# ==============================================================================
# PYNQ-Z2 Constraints for LAN8720 RMII Ethernet PHY
# Board: PYNQ-Z2 (xc7z020clg400-1)
# I/O Bank 35 (3.3V LVCMOS)
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. RMII Reference Clock (50 MHz from LAN8720 Onboard Crystal)
# PMOD A Pin 4 -> Pin Y17 (Clock-Capable IO_L7N_T1_34 / Bank 35)
# ------------------------------------------------------------------------------
set_property PACKAGE_PIN Y17 [get_ports ref_clk_0]
set_property IOSTANDARD LVCMOS33 [get_ports ref_clk_0]

# Define 50.0 MHz Clock (20.0 ns Period) on physical port ref_clk_0
create_clock -period 20.000 -name ref_clk_0 -waveform {0.000 10.000} [get_ports ref_clk_0]

# ------------------------------------------------------------------------------
# 2. RMII Transmit Signals (FPGA -> PHY)
# ------------------------------------------------------------------------------
# PMOD A Pin 1 -> Pin Y18
set_property PACKAGE_PIN Y18 [get_ports rmii_tx_en]
set_property IOSTANDARD LVCMOS33 [get_ports rmii_tx_en]
set_property SLEW FAST [get_ports rmii_tx_en]
set_property DRIVE 8 [get_ports rmii_tx_en]

# PMOD A Pin 2 -> Pin Y19
set_property PACKAGE_PIN Y19 [get_ports {rmii_txd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rmii_txd[0]}]
set_property SLEW FAST [get_ports {rmii_txd[0]}]
set_property DRIVE 8 [get_ports {rmii_txd[0]}]

# PMOD A Pin 3 -> Pin Y16
set_property PACKAGE_PIN Y16 [get_ports {rmii_txd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rmii_txd[1]}]
set_property SLEW FAST [get_ports {rmii_txd[1]}]
set_property DRIVE 8 [get_ports {rmii_txd[1]}]

# ------------------------------------------------------------------------------
# 3. RMII Receive Signals (PHY -> FPGA)
# ------------------------------------------------------------------------------
# PMOD A Pin 7 -> Pin U18
set_property PACKAGE_PIN U18 [get_ports {rmii_rxd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rmii_rxd[0]}]

# PMOD A Pin 8 -> Pin U19
set_property PACKAGE_PIN U19 [get_ports {rmii_rxd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rmii_rxd[1]}]

# PMOD A Pin 9 -> Pin W18
set_property PACKAGE_PIN W18 [get_ports rmii_crs_dv]
set_property IOSTANDARD LVCMOS33 [get_ports rmii_crs_dv]

# ------------------------------------------------------------------------------
# 4. MDIO Management Interface (PMOD A Pin 10 & PMOD B Pin 1)
# ------------------------------------------------------------------------------
# PMOD A Pin 10 -> Pin W19 (MDIO Data - bidirectional)
set_property PACKAGE_PIN W19 [get_ports mdio_io]
set_property IOSTANDARD LVCMOS33 [get_ports mdio_io]
set_property PULLUP true [get_ports mdio_io]

# PMOD B Pin 1 -> Pin W14 (MDC Clock - output from FPGA)
set_property PACKAGE_PIN W14 [get_ports mdio_mdc]
set_property IOSTANDARD LVCMOS33 [get_ports mdio_mdc]

# ------------------------------------------------------------------------------
# 5. Timing Constraints (Setup/Hold for RMII at 50 MHz)
# LAN8720 RMII specification:
# Output delay: Min 2 ns, Max 14 ns relative to 50 MHz ref_clk
# Input setup: 4 ns, hold: 2 ns
# ------------------------------------------------------------------------------
# Receive side (PHY -> FPGA input delays)
set_input_delay -clock ref_clk_0 -max 14.000 [get_ports {rmii_rxd[*] rmii_crs_dv}]
set_input_delay -clock ref_clk_0 -min 2.000  [get_ports {rmii_rxd[*] rmii_crs_dv}]

# Transmit side (FPGA -> PHY output delays)
set_output_delay -clock ref_clk_0 -max 4.000 [get_ports {rmii_txd[*] rmii_tx_en}]
set_output_delay -clock ref_clk_0 -min -2.000 [get_ports {rmii_txd[*] rmii_tx_en}]

# ------------------------------------------------------------------------------
# 6. Asynchronous Clock Domain Crossing (CDC) Ignore Constraints
# Isolates the 50 MHz RMII domain from the 25 MHz MII and 100 MHz AXI domains
# ------------------------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks ref_clk_0] \
    -group [get_clocks -include_generated_clocks clk_fpga_0]

