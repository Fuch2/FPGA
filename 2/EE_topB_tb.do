# Запуск тестбенча верхнего уровня EE_topB (кнопки, сценарии, микросхема на выводах I2C)
#   из PowerShell:      .\run_sim.ps1 EE_topB   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do EE_topB_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_slave.sv eeprom_model.sv eeprom_bus.sv eeprom_ctrl.sv eeprom_ring.sv \
         eeprom_store.sv eeprom_seq.sv bin2bcd.sv seg7_dynamic.sv debounce.sv eeprom_ind.sv EE_topB.sv EE_topB_tb.sv

vsim -voptargs=+acc -onfinish stop EE_topB_tb

add wave clk rst_n
add wave -divider "Keys"
add wave key
add wave -radix unsigned dut/script
add wave dut/use_model
add wave -divider "Sequencer"
add wave dut/u_seq/state dut/u_seq/busy dut/u_seq/error
add wave -radix hex dut/u_seq/info_addr dut/u_seq/info_data dut/u_seq/cursor
add wave -divider "Bus"
add wave scl sda
add wave chip_busy
add wave -radix unsigned chip_cycles
add wave -divider "Indicator"
add wave dut/u_ind/mode
add wave -radix hex seg
add wave dig led

run -all
wave zoom full
