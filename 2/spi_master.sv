// =============================================================================
// SPI-мастер, режим 0 (CPOL = 0, CPHA = 0), старший бит первым (PROTOCOL.md, раздел 6)
//
// Данные выставляются на MOSI по спаду SCK, MISO защёлкивается по фронту SCK.
// Передатчик и приёмник работают одновременно (полный дуплекс): с каждым
// переданным байтом принимается байт.
//
// Транзакция - несколько байт подряд при CS_N = 0. Границы задаёт tx_last:
//   первый принятый байт опускает CS_N, байт с tx_last = 1 - последний,
//   после него CS_N поднимается и не опускается ещё CS_GAP_NS.
// Если следующий байт транзакции не готов, SCK остаётся в 0, CS_N = 0.
//
// Интерфейс передатчика (valid/ready):
//   байт tx_data принимается в такте, где tx_valid = 1 и tx_ready = 1;
//   tx_last = 1 - это последний байт транзакции.
// Интерфейс приёмника:
//   rx_valid - импульс на 1 такт, rx_data - байт, принятый одновременно с
//   последним переданным (держится до следующего байта).
// busy = 1 от приёма первого байта до конца паузы между транзакциями.
//
// Время от спада CS_N до первого фронта SCK = CS_SETUP_NS + полупериод SCK
// (по умолчанию 1.5 мкс, в протоколе нужно не менее 1 мкс).
// =============================================================================
module spi_master #(
    parameter int CLK_FREQ    = 50_000_000,  // частота clk, Гц
    parameter int SCK_FREQ    = 1_000_000,   // частота SCK, Гц
    parameter int CS_SETUP_NS = 1_000,       // пауза после спада CS_N до начала тактирования, нс
    parameter int CS_GAP_NS   = 2_000        // минимальная пауза CS_N = 1 между транзакциями, нс
) (
    input  logic       clk,
    input  logic       rst_n,      // асинхронный сброс, активный 0

    // байтовый интерфейс
    input  logic [7:0] tx_data,
    input  logic       tx_valid,
    output logic       tx_ready,
    input  logic       tx_last,
    output logic [7:0] rx_data,
    output logic       rx_valid,
    output logic       busy,

    // линии SPI
    output logic       spi_cs_n,
    output logic       spi_sck,
    output logic       spi_mosi,
    input  logic       spi_miso
);

    // число тактов clk: полупериод SCK и паузы (с округлением вверх, не меньше 1)
    localparam int HALF_CLKS  = CLK_FREQ / (2 * SCK_FREQ);
    localparam int SETUP_CLKS = ((CLK_FREQ / 1000) * CS_SETUP_NS + 999_999) / 1_000_000;
    localparam int GAP_CLKS   = ((CLK_FREQ / 1000) * CS_GAP_NS   + 999_999) / 1_000_000;

    localparam int CNT_MAX = (HALF_CLKS > SETUP_CLKS) ? ((HALF_CLKS > GAP_CLKS) ? HALF_CLKS : GAP_CLKS)
                                                      : ((SETUP_CLKS > GAP_CLKS) ? SETUP_CLKS : GAP_CLKS);
    localparam int CNT_W   = $clog2(CNT_MAX + 1);

    typedef enum logic [2:0] {IDLE, SETUP, LOW, HIGH, NEXT, TAIL, GAP} state_t;

    state_t           state;
    logic [CNT_W-1:0] cnt;         // счётчик тактов внутри фазы
    logic [2:0]       bit_idx;     // номер передаваемого бита
    logic [7:0]       tx_shreg;    // сдвиговый регистр передатчика, старший бит - на MOSI
    logic [7:0]       rx_shreg;    // сдвиговый регистр приёмника
    logic             last_byte;   // текущий байт - последний в транзакции
    logic [1:0]       miso_sync;   // синхронизатор входа MISO

    wire miso_s = miso_sync[1];

    assign tx_ready = (state == IDLE) || (state == NEXT);
    assign busy     = (state != IDLE);
    assign spi_mosi = tx_shreg[7];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            miso_sync <= 2'b00;
        else
            miso_sync <= {miso_sync[0], spi_miso};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            cnt       <= '0;
            bit_idx   <= '0;
            tx_shreg  <= '0;
            rx_shreg  <= '0;
            rx_data   <= '0;
            rx_valid  <= 1'b0;
            last_byte <= 1'b0;
            spi_cs_n  <= 1'b1;
            spi_sck   <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
                // первый байт транзакции: опускаем CS_N, MOSI уже показывает старший бит
                IDLE: begin
                    cnt <= '0;
                    if (tx_valid) begin
                        tx_shreg  <= tx_data;
                        last_byte <= tx_last;
                        bit_idx   <= '0;
                        spi_cs_n  <= 1'b0;
                        state     <= SETUP;
                    end
                end

                SETUP: begin
                    if (cnt == SETUP_CLKS - 1) begin
                        cnt   <= '0;
                        state <= LOW;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // первая половина периода: SCK = 0; в конце - фронт SCK, выборка MISO
                LOW: begin
                    if (cnt == HALF_CLKS - 1) begin
                        cnt      <= '0;
                        spi_sck  <= 1'b1;
                        rx_shreg <= {rx_shreg[6:0], miso_s};
                        state    <= HIGH;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // вторая половина периода: SCK = 1; в конце - спад SCK, новый бит на MOSI
                HIGH: begin
                    if (cnt == HALF_CLKS - 1) begin
                        cnt      <= '0;
                        spi_sck  <= 1'b0;
                        tx_shreg <= tx_shreg << 1;
                        if (bit_idx == 3'd7) begin
                            rx_data  <= rx_shreg;
                            rx_valid <= 1'b1;
                            state    <= last_byte ? TAIL : NEXT;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                            state   <= LOW;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // продолжение транзакции: ждём следующий байт (SCK = 0, CS_N = 0)
                NEXT: begin
                    if (tx_valid) begin
                        tx_shreg  <= tx_data;
                        last_byte <= tx_last;
                        bit_idx   <= '0;
                        state     <= LOW;
                    end
                end

                // конец транзакции: полупериод до подъёма CS_N, затем пауза
                TAIL: begin
                    if (cnt == HALF_CLKS - 1) begin
                        cnt      <= '0;
                        spi_cs_n <= 1'b1;
                        state    <= GAP;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                GAP: begin
                    if (cnt == GAP_CLKS - 1) begin
                        cnt   <= '0;
                        state <= IDLE;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
