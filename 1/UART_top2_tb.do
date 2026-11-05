# Запуск тестбенча UART_top2 в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 UART_top2   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do UART_top2_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_tx.sv uart_rx.sv UART.sv \
         uart_pkt_tx.sv uart_seq_tx.sv uart_tx_mux.sv \
         uart_pkt_parser.sv pkt_fifo.sv uart_cmd_parser.sv test_pkt_gen.sv \
         seg7_dynamic.sv debounce.sv uart_ind.sv \
         UART_top2.sv UART_top2_tb.sv

vsim -voptargs=+acc -onfinish stop UART_top2_tb

add wave -divider "Board"
add wave clk rst_n
add wave -radix binary key led
add wave uart_rx uart_tx
add wave -divider "RX path"
add wave -radix hex dut/rx_data
add wave dut/rx_valid dut/rx_error
add wave dut/u_pkt_parser/state
add wave dut/rx_pkt_valid dut/rx_chk_error dut/rx_timeout
add wave -radix hex dut/pass_data
add wave dut/pass_valid
add wave dut/cmd_start dut/cmd_stop dut/garbage dut/running
add wave -divider "TX path"
add wave dut/gen_valid
add wave -radix hex dut/gen_meas
add wave -radix binary dut/src_valid dut/src_ready dut/src_last
add wave dut/u_tx_mux/locked
add wave -radix unsigned dut/u_tx_mux/sel
add wave -radix hex dut/tx_data
add wave dut/tx_valid dut/tx_ready
add wave -divider "Indication"
add wave dut/u_ind/mode
add wave -radix hex dut/u_ind/cnt_rx_pkt dut/u_ind/cnt_tx_pkt dut/u_ind/cnt_err dut/u_ind/cnt_garbage

run -all
wave zoom full
