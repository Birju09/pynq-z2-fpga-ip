# Implementation constraints for the LAN8720 PHY on the PYNQ-Z2 board

create_generated_clock -name mac_lk -source [get_ports ref_clk_0] -divide_by 2 \ 
    [get_ports rtps_design_i/rmii_phy_if_0/inst/ref_clk_div_ref/0]


# Isolate RMII domain from AXI domain
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks ref_clk_0] \
    -group [get_clocks -include_generated_clocks clk_fpga_0]
