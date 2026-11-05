# Запуск тестбенча pwm в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 pwm   (или .\run_sim.ps1 pwm -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do pwm_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv pwm.sv pwm_tb.sv

vsim -voptargs=+acc -onfinish stop pwm_tb

add wave -divider "A: WIDTH 8, DIV 12"
add wave clk rst_n
add wave -radix unsigned u_a/duty u_a/dut/duty_r u_a/dut/step_cnt
add wave u_a/period_start u_a/pwm_out
add wave -divider "B: WIDTH 4, DIV 1"
add wave -radix unsigned u_b/duty u_b/dut/step_cnt
add wave u_b/period_start u_b/pwm_out

run -all
wave zoom full
