// =============================================================================
// Парсер пакетов 0xAA + 6 байт данных + XOR (формат PROTOCOL.md, раздел 5.2)
//
// Автомат:
//   HUNT  - поиск синхробайта SYNC; все остальные байты не относятся к пакету
//           и передаются дальше на выход pass_* (например, в парсер команд);
//   DATA  - приём 6 байт данных (MEAS, FILT, ERR, старшим байтом вперёд),
//           подсчёт XOR;
//   CHECK - контрольный байт: совпал с XOR -> pkt_valid, иначе chk_error.
// После CHECK автомат всегда возвращается в HUNT. При ошибке CHK поиск SYNC
// продолжается со следующего байта (байты отвергнутого пакета не
// пересматриваются: в ПЛИС для этого понадобился бы буфер; полный поиск со
// сдвигом на 1 байт делает МУ).
// Если внутри пакета байты не приходят TIMEOUT_CLKS тактов, пакет отбрасывается
// (импульс timeout), автомат возвращается в HUNT.
//
// Вход: байт rx_data в такте, где rx_valid = 1 (выход приёмника UART).
// Выходы:
//   meas, filt, err - данные последнего верного пакета, держатся до следующего;
//   pkt_valid       - импульс на 1 такт: принят верный пакет;
//   chk_error       - импульс: не совпала контрольная сумма;
//   timeout         - импульс: пакет оборвался;
//   pass_data/pass_valid - байт вне пакета (импульс valid на 1 такт);
//   busy            - идёт приём пакета (автомат не в HUNT).
// =============================================================================
module uart_pkt_parser #(
    parameter logic [7:0] SYNC         = 8'hAA,     // синхробайт
    parameter int         TIMEOUT_CLKS = 50_000     // 1 мс при 50 МГц (~11 байт на 115200)
) (
    input  logic        clk,
    input  logic        rst_n,      // асинхронный сброс, активный 0

    input  logic [7:0]  rx_data,
    input  logic        rx_valid,

    output logic [15:0] meas,
    output logic [15:0] filt,
    output logic [15:0] err,
    output logic        pkt_valid,
    output logic        chk_error,
    output logic        timeout,

    output logic [7:0]  pass_data,
    output logic        pass_valid,
    output logic        busy
);

    localparam int DATA_LEN = 6;                    // байт данных в пакете
    localparam int TMR_W    = $clog2(TIMEOUT_CLKS + 1);

    typedef enum logic [1:0] {HUNT, DATA, CHECK} state_t;

    state_t                      state;
    logic [8*DATA_LEN-1:0]       payload;           // принятые байты данных
    logic [$clog2(DATA_LEN)-1:0] data_cnt;          // сколько байт данных принято
    logic [7:0]                  chk;               // XOR принятых байт данных
    logic [TMR_W-1:0]            tmr;               // такты без нового байта

    wire tmr_done = (tmr == TIMEOUT_CLKS - 1);

    assign busy = (state != HUNT);

    // таймер тишины внутри пакета
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                         tmr <= '0;
        else if (state == HUNT || rx_valid) tmr <= '0;
        else if (!tmr_done)                 tmr <= tmr + 1'b1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= HUNT;
            payload    <= '0;
            data_cnt   <= '0;
            chk        <= '0;
            meas       <= '0;
            filt       <= '0;
            err        <= '0;
            pkt_valid  <= 1'b0;
            chk_error  <= 1'b0;
            timeout    <= 1'b0;
            pass_data  <= '0;
            pass_valid <= 1'b0;
        end else begin
            pkt_valid  <= 1'b0;
            chk_error  <= 1'b0;
            timeout    <= 1'b0;
            pass_valid <= 1'b0;

            case (state)
                HUNT: begin
                    if (rx_valid) begin
                        if (rx_data == SYNC) begin
                            data_cnt <= '0;
                            chk      <= '0;
                            state    <= DATA;
                        end else begin
                            pass_data  <= rx_data;
                            pass_valid <= 1'b1;
                        end
                    end
                end

                DATA: begin
                    if (rx_valid) begin
                        payload <= {payload[8*DATA_LEN-9:0], rx_data};
                        chk     <= chk ^ rx_data;
                        if (data_cnt == DATA_LEN - 1)
                            state <= CHECK;
                        else
                            data_cnt <= data_cnt + 1'b1;
                    end else if (tmr_done) begin
                        timeout <= 1'b1;
                        state   <= HUNT;
                    end
                end

                CHECK: begin
                    if (rx_valid) begin
                        if (rx_data == chk) begin
                            {meas, filt, err} <= payload;
                            pkt_valid         <= 1'b1;
                        end else begin
                            chk_error <= 1'b1;
                        end
                        state <= HUNT;
                    end else if (tmr_done) begin
                        timeout <= 1'b1;
                        state   <= HUNT;
                    end
                end

                default: state <= HUNT;
            endcase
        end
    end

endmodule
