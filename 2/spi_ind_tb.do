# Запуск тестбенча spi_ind в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_ind   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do spi_ind_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv seg7_dynamic.sv spi_ind.sv spi_ind_tb.sv

vsim -voptargs=+acc -onfinish stop spi_ind_tb

add wave clk rst_n
add wave -divider "Events"
add wave tx_frame rx_frame err running clear next_mode
add wave cmd_we status_we data_we
add wave -radix hex info_cmd info_status info_data
add wave -divider "Inside"
add wave dut/mode
add wave -radix hex dut/last_data dut/last_cmd dut/last_status
add wave -radix hex dut/cnt_tx dut/cnt_rx dut/cnt_err
add wave -divider "Board"
add wave -radix binary dig seg led

run -all
wave zoom full
