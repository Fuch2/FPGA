# Запуск тестбенча i2c_master (i2c_txn + i2c_bit + модели ведомых) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 i2c_master   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do i2c_master_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv i2c_slave_model.sv i2c_master_tb.sv

vsim -voptargs=+acc -onfinish stop i2c_master_tb

add wave clk rst_n
add wave -divider "Bus"
add wave scl sda
add wave -divider "Command"
add wave cmd_valid cmd_ready
add wave -radix hex cmd_addr
add wave cmd_read cmd_stop
add wave -radix unsigned cmd_len
add wave -divider "Write stream"
add wave -radix hex wr_data
add wave wr_valid wr_ready
add wave -divider "Read stream"
add wave -radix hex rd_data
add wave rd_valid rd_ready rd_last
add wave -divider "Result"
add wave done
add wave -radix unsigned res_len
add wave res_nack_addr res_nack_data res_timeout res_arb
add wave ev_start ev_stop ev_ack ev_nack
add wave -divider "Inside"
add wave dut/u_txn/state
add wave dut/u_bit/state
add wave -radix unsigned dut/u_bit/step
add wave -divider "Slave 0x48"
add wave s48/state
add wave -radix unsigned s48/ptr

run -all
wave zoom full
