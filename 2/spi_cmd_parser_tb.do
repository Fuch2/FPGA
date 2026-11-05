# Запуск тестбенча spi_cmd_parser в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_cmd_parser   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do spi_cmd_parser_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_cmd_parser.sv spi_cmd_parser_tb.sv

vsim -voptargs=+acc -onfinish stop spi_cmd_parser_tb

add wave clk rst_n
add wave -divider "From slave"
add wave frame_start frame_end frame_short
add wave -radix hex rx_data
add wave rx_valid
add wave -divider "Commands"
add wave cmd_start cmd_measure cmd_stop cmd_nop cmd_unknown frame_err
add wave -radix hex last_cmd
add wave -divider "Inside"
add wave -radix unsigned dut/byte_cnt
add wave -radix hex dut/first_byte

run -all
wave zoom full
