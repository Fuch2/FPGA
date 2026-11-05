// =============================================================================
// Конвейер передатчика МО: команды Старт, Измерение, Стоп по SPI (PROTOCOL.md, раздел 6)
//
// Каждая команда - две транзакции по 4 байта (ответ МИ на команду приходит
// не в той же транзакции, а в следующей, раздел 6.4):
//   1. кадр {CMD, 00, 00, 00}, ответ МИ в нём - старый, только проверяется;
//   2. пауза: T_RESP_US после «Измерение» (МИ читает EEPROM), T_ACK_US после
//      «Старт» и «Стоп»; отсчитывается от конца первой транзакции (link_busy = 0);
//   3. кадр NOP {00, 00, 00, 00}, ответ МИ - результат команды.
// Ответ МИ: {STATUS, D_H, D_L, CHK}, CHK = ~(STATUS ^ D_H ^ D_L), STATUS[7] = 1.
//
// Запросы req_start, req_measure, req_stop - импульсы на 1 такт. Пока команда
// выполняется (busy = 1), новые запросы игнорируются и выдаётся импульс req_lost.
// Одновременные запросы: приоритет Стоп, Старт, Измерение.
//
// Выход (valid/ready, к spi_master):
//   байт tx_data принимается в такте, где tx_valid = 1 и tx_ready = 1;
//   tx_last = 1 на последнем байте кадра. Принятые байты - на входе rx_data/rx_valid;
//   link_busy - выход busy мастера (транзакция ещё не закончена).
// Результат каждого кадра - импульс rsp_valid на 1 такт:
//   rsp_final - это ответ на команду (кадр NOP), иначе - ответ на первый кадр;
//   rsp_cmd   - код выполняемой команды;
//   rsp_status, rsp_data - байт STATUS и слово D из ответа;
//   rsp_ok    - CHK верна и STATUS[7] = 1 (иначе ответ испорчен или МИ не подключена).
// Повторы по BUSY / RUN (раздел 6.4) делает управляющий автомат МО выше.
// =============================================================================
module spi_cmd_tx #(
    parameter int         CLK_FREQ    = 50_000_000,   // частота clk, Гц
    parameter int         T_RESP_US   = 2_000,        // пауза до чтения ответа на «Измерение», мкс
    parameter int         T_ACK_US    = 1_000,        // пауза до чтения ответа на «Старт» / «Стоп», мкс
    parameter logic [7:0] CMD_START   = 8'hA1,
    parameter logic [7:0] CMD_MEASURE = 8'hA2,
    parameter logic [7:0] CMD_STOP    = 8'hA3,
    parameter logic [7:0] CMD_NOP     = 8'h00
) (
    input  logic        clk,
    input  logic        rst_n,        // асинхронный сброс, активный 0

    input  logic        req_start,
    input  logic        req_measure,
    input  logic        req_stop,
    output logic        busy,
    output logic        req_lost,

    // к spi_master
    output logic [7:0]  tx_data,
    output logic        tx_valid,
    input  logic        tx_ready,
    output logic        tx_last,
    input  logic [7:0]  rx_data,
    input  logic        rx_valid,
    input  logic        link_busy,

    // результат кадра
    output logic        rsp_valid,
    output logic        rsp_final,
    output logic [7:0]  rsp_cmd,
    output logic [7:0]  rsp_status,
    output logic [15:0] rsp_data,
    output logic        rsp_ok
);

    localparam int FRAME_LEN = 4;
    localparam int RESP_CLKS = (CLK_FREQ / 1_000_000) * T_RESP_US;
    localparam int ACK_CLKS  = (CLK_FREQ / 1_000_000) * T_ACK_US;
    localparam int WAIT_MAX  = (RESP_CLKS > ACK_CLKS) ? RESP_CLKS : ACK_CLKS;
    localparam int WAIT_W    = $clog2(WAIT_MAX + 1);

    typedef enum logic [1:0] {IDLE, SEND, PAUSE, REPORT} state_t;

    state_t                state;
    logic [7:0]            cmd;           // выполняемая команда
    logic                  second;        // 0 - кадр с командой, 1 - кадр NOP
    logic [WAIT_W-1:0]     pause_clks;    // сколько ждать между кадрами
    logic [WAIT_W-1:0]     pause_cnt;
    logic [2:0]            tx_idx;        // сколько байт кадра передано
    logic [2:0]            rx_cnt;        // сколько байт кадра принято
    logic [8*FRAME_LEN-1:0] rx_shreg;     // принятый кадр, первый байт - старший

    wire any_req = req_start || req_measure || req_stop;

    wire [7:0] r_status = rx_shreg[31:24];
    wire [7:0] r_dh     = rx_shreg[23:16];
    wire [7:0] r_dl     = rx_shreg[15:8];
    wire [7:0] r_chk    = rx_shreg[7:0];

    assign busy     = (state != IDLE);
    assign tx_valid = (state == SEND) && (tx_idx != FRAME_LEN);
    assign tx_last  = (tx_idx == FRAME_LEN - 1);
    assign tx_data  = (tx_idx == 0) ? (second ? CMD_NOP : cmd) : 8'h00;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            cmd        <= '0;
            second     <= 1'b0;
            pause_clks <= '0;
            pause_cnt  <= '0;
            tx_idx     <= '0;
            rx_cnt     <= '0;
            rx_shreg   <= '0;
            req_lost   <= 1'b0;
            rsp_valid  <= 1'b0;
            rsp_final  <= 1'b0;
            rsp_cmd    <= '0;
            rsp_status <= '0;
            rsp_data   <= '0;
            rsp_ok     <= 1'b0;
        end else begin
            req_lost  <= any_req && (state != IDLE);
            rsp_valid <= 1'b0;

            case (state)
                IDLE: begin
                    tx_idx <= '0;
                    rx_cnt <= '0;
                    second <= 1'b0;
                    if (any_req) begin
                        if (req_stop) begin
                            cmd        <= CMD_STOP;
                            pause_clks <= WAIT_W'(ACK_CLKS);
                        end else if (req_start) begin
                            cmd        <= CMD_START;
                            pause_clks <= WAIT_W'(ACK_CLKS);
                        end else begin
                            cmd        <= CMD_MEASURE;
                            pause_clks <= WAIT_W'(RESP_CLKS);
                        end
                        state <= SEND;
                    end
                end

                // передача и приём кадра идут одновременно; конец - когда принят весь кадр
                SEND: begin
                    if (tx_valid && tx_ready)
                        tx_idx <= tx_idx + 1'b1;
                    if (rx_valid) begin
                        rx_shreg <= {rx_shreg[8*FRAME_LEN-9:0], rx_data};
                        rx_cnt   <= rx_cnt + 1'b1;
                        if (rx_cnt == FRAME_LEN - 1)
                            state <= REPORT;
                    end
                end

                // ответ принят целиком: выдаём результат, затем пауза или конец
                REPORT: begin
                    rsp_valid  <= 1'b1;
                    rsp_final  <= second;
                    rsp_cmd    <= cmd;
                    rsp_status <= r_status;
                    rsp_data   <= {r_dh, r_dl};
                    rsp_ok     <= r_status[7] && (r_chk == ~(r_status ^ r_dh ^ r_dl));
                    pause_cnt  <= '0;
                    if (second) begin
                        state <= IDLE;
                    end else begin
                        state <= PAUSE;
                    end
                end

                PAUSE: begin
                    if (link_busy) begin
                        pause_cnt <= '0;               // ждём конца транзакции (CS_N = 1 и пауза мастера)
                    end else if (pause_cnt == pause_clks) begin
                        second <= 1'b1;
                        tx_idx <= '0;
                        rx_cnt <= '0;
                        state  <= SEND;
                    end else begin
                        pause_cnt <= pause_cnt + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
