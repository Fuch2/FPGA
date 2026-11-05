# Запуск тестбенча eeprom_model (имитатор AT24C08, проверяется через i2c_master)
#   из PowerShell:      .\run_sim.ps1 eeprom_model   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do eeprom_model_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv i2c_slave.sv eeprom_model.sv eeprom_model_tb.sv

vsim -voptargs=+acc -onfinish stop eeprom_model_tb

add wave clk rst_n
add wave -divider "Bus"
add wave scl sda
add wave -divider "Master"
add wave -radix hex cmd_addr
add wave cmd_valid cmd_ready cmd_read cmd_stop done
add wave res_nack_addr res_nack_data
add wave -divider "Model"
add wave model_busy present
add wave -radix unsigned wr_cycles
add wave -radix hex model/ptr
add wave model/first_byte model/committing
add wave -radix hex model/page_valid
add wave -radix unsigned model/cycle_cnt

run -all
wave zoom full
