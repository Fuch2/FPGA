// =============================================================================
// SPI-слейв, режим 0 (CPOL = 0, CPHA = 0), старший бит первым (PROTOCOL.md, раздел 6)
//
// SCK, MOSI и CS_N асинхронны по отношению к clk: каждый вход проходит
// два триггера синхронизатора и третий - для выделения фронтов. Всё остальное
// работает в домене clk (50 МГц), поэтому SCK должен быть заметно медленнее clk
// (при SCK = 1 МГц запас 25 тактов на полупериод).
//
// Приём: по фронту SCK защёлкивается MOSI; после 8 бит выдаётся
//   rx_valid - импульс на 1 такт, rx_data - принятый байт.
// Передача (valid/ready): байт tx_data принимается во внутренний регистр в такте,
//   где tx_valid = 1 и tx_ready = 1. tx_ready = 1, пока кадр идёт (CS_N = 0) и
//   регистр пуст, поэтому источник может подготовить следующий байт заранее.
//   Первый байт должен быть принят до первого фронта SCK (в протоколе после
//   спада CS_N не менее 1 мкс); сразу после frame_start источник должен
//   начать выдавать байты. Если к началу байта данных нет, передаётся 0xFF и
//   выдаётся импульс tx_underrun.
// spi_miso_oe = 1 при CS_N = 0; при CS_N = 1 вывод MISO переводится в Z на
//   верхнем уровне: assign spi_miso = spi_miso_oe ? miso : 1'bz.
//
// События кадра (импульсы на 1 такт, формируются по CS_N):
//   frame_start - CS_N опустился;  frame_end - CS_N поднялся;
//   frame_short - CS_N поднялся посреди байта (число фронтов SCK не кратно 8).
// =============================================================================
module spi_slave (
    input  logic       clk,
    input  logic       rst_n,         // асинхронный сброс, активный 0

    // линии SPI
    input  logic       spi_cs_n,
    input  logic       spi_sck,
    input  logic       spi_mosi,
    output logic       spi_miso,
    output logic       spi_miso_oe,

    // приём байтов
    output logic [7:0] rx_data,
    output logic       rx_valid,

    // передача байтов
    input  logic [7:0] tx_data,
    input  logic       tx_valid,
    output logic       tx_ready,
    output logic       tx_underrun,

    // границы кадра
    output logic       frame_start,
    output logic       frame_end,
    output logic       frame_short
);

    localparam logic [7:0] IDLE_BYTE = 8'hFF;   // что уходит по MISO, если данных нет

    // -------------------------------------------------------------------------
    // Синхронизация входов: [0], [1] - триггеры синхронизатора, [2] - предыдущее значение
    // -------------------------------------------------------------------------
    logic [2:0] cs_q, sck_q, mosi_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_q   <= 3'b111;
            sck_q  <= 3'b000;
            mosi_q <= 3'b000;
        end else begin
            cs_q   <= {cs_q[1:0],   spi_cs_n};
            sck_q  <= {sck_q[1:0],  spi_sck};
            mosi_q <= {mosi_q[1:0], spi_mosi};
        end
    end

    wire cs_act   = ~cs_q[1];                 // кадр идёт
    wire sck_s    = sck_q[1];
    wire mosi_s   = mosi_q[1];
    wire sck_rise = sck_q[1] & ~sck_q[2];
    wire sck_fall = ~sck_q[1] & sck_q[2];

    assign frame_start = cs_q[2] & ~cs_q[1];
    assign frame_end   = ~cs_q[2] & cs_q[1];

    // -------------------------------------------------------------------------
    // Приём, передача
    // -------------------------------------------------------------------------
    logic [2:0] bit_cnt;       // сколько фронтов SCK принято в текущем байте
    logic [7:0] rx_shreg;
    logic [7:0] tx_shreg;      // сдвиговый регистр, старший бит - на MISO
    logic       tx_loaded;     // в tx_shreg лежит неотправленный байт
    logic [7:0] hold;          // принятый от источника, ещё не загруженный байт
    logic       hold_valid;

    assign frame_short = frame_end && (bit_cnt != 3'd0);
    assign tx_ready    = cs_act && !hold_valid && !frame_start;
    assign spi_miso    = tx_shreg[7];
    assign spi_miso_oe = cs_act;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt     <= '0;
            rx_shreg    <= '0;
            rx_data     <= '0;
            rx_valid    <= 1'b0;
            tx_shreg    <= IDLE_BYTE;
            tx_loaded   <= 1'b0;
            hold        <= '0;
            hold_valid  <= 1'b0;
            tx_underrun <= 1'b0;
        end else begin
            rx_valid    <= 1'b0;
            tx_underrun <= 1'b0;

            if (frame_start || frame_end) begin
                // начало и конец кадра: всё в исходное состояние
                bit_cnt    <= '0;
                tx_shreg   <= IDLE_BYTE;
                tx_loaded  <= 1'b0;
                hold_valid <= 1'b0;
            end else if (cs_act) begin
                // байт от источника в буфер
                if (tx_valid && tx_ready) begin
                    hold       <= tx_data;
                    hold_valid <= 1'b1;
                end

                // загрузка в сдвиговый регистр, пока SCK = 0 и байт не начат
                if (hold_valid && !tx_loaded && bit_cnt == 3'd0 && !sck_s) begin
                    tx_shreg   <= hold;
                    tx_loaded  <= 1'b1;
                    hold_valid <= 1'b0;
                end

                // спад SCK: следующий бит на MISO
                if (sck_fall && bit_cnt != 3'd0)
                    tx_shreg <= {tx_shreg[6:0], 1'b1};

                // фронт SCK: выборка MOSI
                if (sck_rise) begin
                    rx_shreg <= {rx_shreg[6:0], mosi_s};
                    if (bit_cnt == 3'd0 && !tx_loaded)
                        tx_underrun <= 1'b1;
                    if (bit_cnt == 3'd7) begin
                        bit_cnt   <= '0;
                        rx_data   <= {rx_shreg[6:0], mosi_s};
                        rx_valid  <= 1'b1;
                        tx_shreg  <= IDLE_BYTE;
                        tx_loaded <= 1'b0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
            end
        end
    end

endmodule
