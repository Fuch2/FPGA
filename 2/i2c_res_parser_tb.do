# Запуск тестбенча i2c_res_parser в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 i2c_res_parser   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do i2c_res_parser_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_res_parser.sv i2c_res_parser_tb.sv

vsim -voptargs=+acc -onfinish stop i2c_res_parser_tb

add wave clk rst_n
add wave -divider "Command"
add wave cmd_fire
add wave -radix hex cmd_addr
add wave cmd_read
add wave -divider "Read stream"
add wave -radix hex rd_data
add wave rd_valid rd_ready
add wave -divider "Result"
add wave done res_nack_addr res_nack_data res_timeout res_arb expect_nack
add wave -divider "Outputs"
add wave -radix hex rd_word last_addr last_status
add wave txn_p nack_p err_p
add wave -divider "Found list"
add wave found_en found_clear clear
add wave -radix unsigned found_cnt found_idx
add wave -radix hex found_addr

run -all
wave zoom full
