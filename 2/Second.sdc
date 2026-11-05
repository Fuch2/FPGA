# Тактовый генератор платы 50 МГц
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# Исключения для асинхронных входов и медленных выходов добавляются
# вместе с top-модулем на каждом этапе (см. UART.sdc в проекте 1).
