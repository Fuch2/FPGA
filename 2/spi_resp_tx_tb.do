# Запуск тестбенча spi_resp_tx в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_resp_tx   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do spi_resp_tx_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_resp_tx.sv spi_resp_tx_tb.sv

vsim -voptargs=+acc -onfinish stop spi_resp_tx_tb

add wave clk rst_n
add wave frame_start
add wave -radix hex status data
add wave -divider "To slave"
add wave -radix hex tx_data
add wave tx_valid tx_ready
add wave -divider "Inside"
add wave dut/state
add wave -radix hex dut/shreg
add wave -radix unsigned dut/byte_idx

run -all
wave zoom full
