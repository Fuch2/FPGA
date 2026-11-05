// =============================================================================
// Парсер команд МУ -> МО (PROTOCOL.md, раздел 5.1)
//
// Команды однобайтовые:
//   CMD_START (0x01) - Старт: импульс cmd_start, автомат переходит в RUNNING;
//   CMD_STOP  (0x02) - Стоп:  импульс cmd_stop,  автомат переходит в STOPPED;
//   любой другой байт - мусор: игнорируется, импульс garbage (для счётчика).
// Повторная команда тоже выдаёт импульс (повторный Старт = перезапуск).
//
// Вход: байт data в такте, где valid = 1 (выход приёмника UART или
//   pass_* парсера пакетов).
// Выходы: cmd_start, cmd_stop, garbage - импульсы на 1 такт;
//   running = 1 после Старт и до Стоп.
// =============================================================================
module uart_cmd_parser #(
    parameter logic [7:0] CMD_START = 8'h01,
    parameter logic [7:0] CMD_STOP  = 8'h02
) (
    input  logic       clk,
    input  logic       rst_n,      // асинхронный сброс, активный 0

    input  logic [7:0] data,
    input  logic       valid,

    output logic       cmd_start,
    output logic       cmd_stop,
    output logic       garbage,
    output logic       running
);

    typedef enum logic {STOPPED, RUNNING} state_t;

    state_t state;

    assign running = (state == RUNNING);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= STOPPED;
            cmd_start <= 1'b0;
            cmd_stop  <= 1'b0;
            garbage   <= 1'b0;
        end else begin
            cmd_start <= 1'b0;
            cmd_stop  <= 1'b0;
            garbage   <= 1'b0;

            if (valid) begin
                case (data)
                    CMD_START: begin
                        cmd_start <= 1'b1;
                        state     <= RUNNING;
                    end
                    CMD_STOP: begin
                        cmd_stop <= 1'b1;
                        state    <= STOPPED;
                    end
                    default: garbage <= 1'b1;
                endcase
            end
        end
    end

endmodule
