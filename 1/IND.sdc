# Тактовый генератор платы 50 МГц
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# кнопки и сброс синхронизируются внутри,
# индикатор и светодиоды - медленные выходы, временные проверки не нужны
set_false_path -from [get_ports {rst_n key[*]}]
set_false_path -to   [get_ports {seg[*] dig[*] led[*]}]
