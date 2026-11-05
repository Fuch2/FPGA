// =============================================================================
// Заглушка МИ для проверки SPI (этап 3): вместо ДТ и EEPROM - счётчик
//
// Команды приходят импульсами от spi_cmd_parser, ответ определяют выходы
// status и data (слово защёлкивает spi_resp_tx в начале следующего кадра).
//   cmd_start   - RUN = 1, счётчик значений в начало (DATA_INIT), data = 0, флаги сброшены;
//   cmd_stop    - RUN = 0;
//   cmd_measure - сбрасывает VALID и EMPTY, ставит BUSY на MEAS_US мкс (имитация
//                 чтения EEPROM по I2C), затем:
//                   RUN = 1: data = DATA_INIT + n * DATA_STEP, n++, VALID = 1;
//                   RUN = 0: EMPTY = 1 (нет непрочитанных значений);
//   «Измерение» во время BUSY игнорируется; cmd_nop и прочее ничего не меняют.
// STATUS (PROTOCOL.md, раздел 6.3):
//   {MARK = 1, 0, OVF = 0, I2C_ERR = 0, BUSY, EMPTY, VALID, RUN}.
// meas_done - импульс на 1 такт, когда измерение закончено (VALID или EMPTY).
// =============================================================================
module spi_mi_stub #(
    parameter int          CLK_FREQ  = 50_000_000,      // частота clk, Гц
    parameter int          MEAS_US   = 500,             // время «чтения EEPROM», мкс
    parameter logic [15:0] DATA_INIT = 16'h0001,        // первое отдаваемое значение
    parameter logic [15:0] DATA_STEP = 16'h0001         // приращение значения
) (
    input  logic        clk,
    input  logic        rst_n,        // асинхронный сброс, активный 0

    input  logic        cmd_start,
    input  logic        cmd_measure,
    input  logic        cmd_stop,

    output logic [7:0]  status,
    output logic [15:0] data,
    output logic        running,
    output logic        meas_done
);

    localparam int MEAS_CLKS = (CLK_FREQ / 1_000_000) * MEAS_US;
    localparam int CNT_W     = $clog2(MEAS_CLKS + 1);

    typedef enum logic {IDLE, MEASURE} state_t;

    state_t           state;
    logic [CNT_W-1:0] cnt;
    logic             run, valid, empty;
    logic [15:0]      next_value;       // значение для следующего измерения

    wire busy = (state == MEASURE);

    assign running = run;
    assign status  = {1'b1, 1'b0, 1'b0, 1'b0, busy, empty, valid, run};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            cnt        <= '0;
            run        <= 1'b0;
            valid      <= 1'b0;
            empty      <= 1'b0;
            data       <= '0;
            next_value <= DATA_INIT;
            meas_done  <= 1'b0;
        end else begin
            meas_done <= 1'b0;

            if (cmd_start) begin
                state      <= IDLE;
                run        <= 1'b1;
                valid      <= 1'b0;
                empty      <= 1'b0;
                data       <= '0;
                next_value <= DATA_INIT;
            end else if (cmd_stop) begin
                run <= 1'b0;
            end else begin
                case (state)
                    IDLE: begin
                        cnt <= '0;
                        if (cmd_measure) begin
                            valid <= 1'b0;
                            empty <= 1'b0;
                            state <= MEASURE;
                        end
                    end

                    MEASURE: begin
                        if (cnt == MEAS_CLKS - 1) begin
                            if (run) begin
                                data       <= next_value;
                                next_value <= next_value + DATA_STEP;
                                valid      <= 1'b1;
                            end else begin
                                empty <= 1'b1;
                            end
                            meas_done <= 1'b1;
                            state     <= IDLE;
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

endmodule
