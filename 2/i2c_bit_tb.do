# Запуск тестбенча i2c_bit в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 i2c_bit   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do i2c_bit_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_bit_tb.sv

vsim -voptargs=+acc -onfinish stop i2c_bit_tb

add wave clk rst_n
add wave -divider "Command"
add wave -radix unsigned cmd
add wave cmd_bit cmd_valid cmd_ready
add wave -divider "Response"
add wave rsp_valid rsp_bit rsp_timeout rsp_arb
add wave -divider "Bus"
add wave scl sda scl_low sda_low tb_scl_low tb_sda_low
add wave -divider "Inside"
add wave dut/state
add wave -radix unsigned dut/step dut/cnt dut/stall

run -all
wave zoom full
