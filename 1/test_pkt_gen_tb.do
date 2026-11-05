# Запуск тестбенча test_pkt_gen в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 test_pkt_gen   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do test_pkt_gen_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv test_pkt_gen.sv test_pkt_gen_tb.sv

vsim -voptargs=+acc -onfinish stop test_pkt_gen_tb

add wave clk rst_n run single restart ready
add wave -divider "Main"
add wave -radix unsigned dut/tmr
add wave valid
add wave -radix hex meas filt err
add wave -divider "Saturation"
add wave s_valid
add wave -radix hex s_meas s_filt s_err

run -all
wave zoom full
