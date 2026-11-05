# Запуск тестбенча i2c_seq (конвейер + парсер + мастер + модели ведомых) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 i2c_seq   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do i2c_seq_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv i2c_seq.sv i2c_res_parser.sv i2c_slave_model.sv i2c_seq_tb.sv

vsim -voptargs=+acc -onfinish stop i2c_seq_tb

add wave clk rst_n
add wave -divider "Sequencer"
add wave start
add wave -radix unsigned script
add wave busy finished seq_error scan_active expect_nack
add wave dut/state
add wave -radix unsigned dut/step_n
add wave -radix hex dut/pattern
add wave -divider "Command to master"
add wave cmd_valid cmd_ready
add wave -radix hex cmd_addr
add wave cmd_read cmd_stop
add wave -radix unsigned cmd_len
add wave -radix hex wr_data
add wave wr_valid wr_ready
add wave -divider "Bus"
add wave scl sda
add wave -divider "Result"
add wave done res_nack_addr res_nack_data res_timeout res_arb
add wave -radix hex rd_data
add wave rd_valid
add wave -divider "Parser"
add wave -radix hex rd_word last_addr last_status
add wave txn_p nack_p err_p
add wave -radix unsigned found_cnt

run -all
wave zoom full
