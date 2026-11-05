# Тактовый генератор платы 50 МГц
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# асинхронные входы (кнопка, линия RX) синхронизируются внутри,
# медленные выходы (TX, светодиоды) - временные проверки не нужны
set_false_path -from [get_ports {rst_n uart_rx}]
set_false_path -to   [get_ports {uart_tx led[*]}]
