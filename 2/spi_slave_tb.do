# Запуск тестбенча spi_slave в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_slave   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do spi_slave_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_slave.sv spi_slave_tb.sv

vsim -voptargs=+acc -onfinish stop spi_slave_tb

add wave clk rst_n
add wave -divider "SPI"
add wave spi_cs_n spi_sck spi_mosi spi_miso spi_miso_oe
add wave -divider "Frame events"
add wave frame_start frame_end frame_short tx_underrun
add wave -divider "Receive"
add wave -radix hex rx_data
add wave rx_valid
add wave -divider "Transmit"
add wave -radix hex tx_data
add wave tx_valid tx_ready
add wave -divider "Inside"
add wave -radix binary dut/cs_q dut/sck_q dut/mosi_q
add wave -radix unsigned dut/bit_cnt
add wave -radix hex dut/tx_shreg dut/hold
add wave dut/hold_valid dut/tx_loaded

run -all
wave zoom full
