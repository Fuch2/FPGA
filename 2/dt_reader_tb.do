# Запуск тестбенча dt_reader (чтение LM75A через i2c_master; dt_model и i2c_slave_model)
#   из PowerShell:      .\run_sim.ps1 dt_reader   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do dt_reader_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv temp_gen.sv i2c_slave.sv dt_model.sv \
         i2c_slave_model.sv dt_reader.sv dt_reader_tb.sv

vsim -voptargs=+acc -onfinish stop dt_reader_tb

add wave clk rst_n
add wave -divider "Env A (dt_model)"
add wave {scl[0]} {sda[0]}
add wave start_read busy temp_valid read_fail
add wave -radix hex temp_data
add wave env_a/u_reader/state
add wave env_a/u_reader/cmd_valid env_a/u_reader/cmd_ready
add wave -radix hex env_a/u_reader/cmd_addr
add wave env_a/u_reader/cmd_read env_a/u_reader/cmd_stop
add wave -radix unsigned env_a/u_reader/cmd_len
add wave -divider "Model A"
add wave -radix hex model_a/conf model_a/temp_snap model_a/temp_word
add wave -divider "Env C (i2c_slave_model, config)"
add wave env_c/u_reader/state
add wave -radix hex env_c/u_reader/raw_hi env_c/u_reader/raw_lo

run -all
wave zoom full
