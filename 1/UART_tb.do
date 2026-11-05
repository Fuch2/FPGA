# Запуск тестбенча UART в ModelSim / Questa
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA; do UART_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_tx.sv uart_rx.sv UART.sv UART_tb.sv

vsim -voptargs=+acc -onfinish stop UART_tb

add wave -divider "Clock / reset"
add wave clk rst_n
add wave -divider "TX"
add wave -radix hex tx_data
add wave tx_valid tx_ready tx
add wave dut/u_tx/state
add wave -divider "RX"
add wave rx
add wave -radix hex rx_data
add wave rx_valid rx_error
add wave dut/u_rx/state

run -all
wave zoom full
