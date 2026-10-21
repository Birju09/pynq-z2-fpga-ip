sudo mkdir -p /sys/kernel/config/device-tree/overlays/dma_node
sudo sh -c "cat dma_overlay.dtbo > /sys/kernel/config/device-tree/overlays/dma_node/dtbo"

echo loopback.bin | sudo tee /sys/class/fpga_manager/fpga0/firmware

cat /sys/class/fpga_manager/fpga0/state

echo "40000000.fabric" | sudo tee /sys/bus/platform/drivers/uio_pdrv_genirq/unbind