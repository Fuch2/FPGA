# Запуск тестбенча spi_master в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_master   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do spi_master_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_master.sv spi_master_tb.sv

vsim -voptargs=+acc -onfinish stop spi_master_tb

add wave clk rst_n
add wave -divider "Byte interface"
add wave -radix hex tx_data
add wave tx_valid tx_ready tx_last
add wave -radix hex rx_data
add wave rx_valid busy
add wave -divider "SPI"
add wave spi_cs_n spi_sck spi_mosi spi_miso
add wave -divider "Inside"
add wave dut/state
add wave -radix unsigned dut/cnt dut/bit_idx

run -all
wave zoom full
