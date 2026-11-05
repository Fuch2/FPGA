// =============================================================================
// I2C-ведомый, уровень байтов (синтезируемый; растяжку SCL не использует)
//
// Следит за линиями шины (входы scl_in, sda_in синхронизируются внутри), узнаёт
// START, STOP, адрес и биты данных. Линию SDA только прижимает (sda_low = 1) или
// отпускает; SCL ведомый не трогает.
//
// Интерфейс к устройству (всё импульсы на 1 такт, если не сказано иное):
//   enable      - 0: ведомый «не припаян», на адрес не отвечает;
//   addr_hit    - принят адрес ведомого (ACK будет выдан); addr_read = 1 - мастер читает
//                 (уровень, держится до следующего адреса);
//   rx_valid    - принят байт записи rx_data и подтверждён ACK;
//   tx_req      - ведомому нужен очередной байт для чтения: tx_data должен быть
//                 правильным в этом же такте (устройство выбирает его по своим
//                 счётчикам, изменяя их по tx_req); последний байт - тот, на который
//                 мастер ответил NACK.
// Ведомый отвечает ACK на каждый принятый байт записи. Тактовая частота clk должна
// быть не меньше чем в 10 раз выше SCL.
// =============================================================================
module i2c_slave #(
    parameter logic [6:0] ADDR = 7'h48          // 7-битный адрес ведомого
) (
    input  logic       clk,
    input  logic       rst_n,           // асинхронный сброс, активный 0

    input  logic       enable,

    output logic       addr_hit,
    output logic       addr_read,
    output logic       rx_valid,
    output logic [7:0] rx_data,
    output logic       tx_req,
    input  logic [7:0] tx_data,

    input  logic       scl_in,
    input  logic       sda_in,
    output logic       sda_low
);

    typedef enum logic [3:0] {
        IDLE,       // шина свободна или идёт чужая транзакция
        RX_ADDR,    // принимаем 8 бит: адрес и R/W
        RX_DATA,    // принимаем 8 бит данных записи
        ACK_WAIT,   // байт принят, ждём спад SCL, чтобы выдать ACK
        ACK_LOW,    // ACK выдан, ждём спад SCL, чтобы отпустить SDA
        LOAD,       // берём очередной байт для чтения (tx_req)
        TX,         // выдаём 8 бит
        MACK,       // ждём ACK / NACK мастера
        MACK_END    // ответ принят, ждём спад SCL
    } state_t;

    state_t     state;
    logic [7:0] shreg;
    logic [3:0] bit_cnt;
    logic       ack_en;         // на принятый байт отвечаем ACK
    logic       next_tx;        // после ACK начинается передача байтов мастеру
    logic       master_nack;

    // синхронизация линий и выделение событий
    logic [2:0] scl_sync, sda_sync;

    wire scl_s    = scl_sync[1];
    wire sda_s    = sda_sync[1];
    wire scl_rise = scl_sync[1] && !scl_sync[2];
    wire scl_fall = !scl_sync[1] && scl_sync[2];
    wire start_ev = scl_s && sda_sync[2] && !sda_s;
    wire stop_ev  = scl_s && !sda_sync[2] && sda_s;

    wire [7:0] byte_in = {shreg[6:0], sda_s};      // полный байт в такте 8-го фронта SCL

    assign tx_req = (state == LOAD);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl_in};
            sda_sync <= {sda_sync[1:0], sda_in};
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            shreg       <= '0;
            bit_cnt     <= '0;
            ack_en      <= 1'b0;
            next_tx     <= 1'b0;
            master_nack <= 1'b0;
            addr_hit    <= 1'b0;
            addr_read   <= 1'b0;
            rx_valid    <= 1'b0;
            rx_data     <= '0;
            sda_low     <= 1'b0;
        end else begin
            addr_hit <= 1'b0;
            rx_valid <= 1'b0;

            if (start_ev) begin
                state   <= RX_ADDR;
                bit_cnt <= '0;
                sda_low <= 1'b0;
            end else if (stop_ev) begin
                state   <= IDLE;
                sda_low <= 1'b0;
            end else begin
                case (state)
                    RX_ADDR, RX_DATA: if (scl_rise) begin
                        shreg   <= byte_in;
                        bit_cnt <= bit_cnt + 1'b1;
                        if (bit_cnt == 4'd7) begin
                            bit_cnt <= '0;
                            if (state == RX_ADDR) begin
                                ack_en    <= enable && (byte_in[7:1] == ADDR);
                                next_tx   <= byte_in[0];
                                addr_hit  <= enable && (byte_in[7:1] == ADDR);
                                if (enable && byte_in[7:1] == ADDR) addr_read <= byte_in[0];
                            end else begin
                                ack_en   <= 1'b1;
                                next_tx  <= 1'b0;
                                rx_data  <= byte_in;
                                rx_valid <= 1'b1;
                            end
                            state <= ACK_WAIT;
                        end
                    end

                    ACK_WAIT: if (scl_fall) begin
                        if (ack_en) begin
                            sda_low <= 1'b1;
                            state   <= ACK_LOW;
                        end else begin
                            state <= IDLE;                 // чужой адрес: ждём следующий START
                        end
                    end

                    ACK_LOW: if (scl_fall) begin
                        sda_low <= 1'b0;
                        state   <= next_tx ? LOAD : RX_DATA;
                    end

                    LOAD: begin
                        shreg   <= tx_data;
                        sda_low <= !tx_data[7];
                        bit_cnt <= 4'd1;
                        state   <= TX;
                    end

                    TX: if (scl_fall) begin
                        if (bit_cnt == 4'd8) begin
                            sda_low <= 1'b0;               // отпускаем SDA для ACK мастера
                            state   <= MACK;
                        end else begin
                            shreg   <= shreg << 1;
                            sda_low <= !shreg[6];
                            bit_cnt <= bit_cnt + 1'b1;
                        end
                    end

                    MACK: if (scl_rise) begin
                        master_nack <= sda_s;
                        state       <= MACK_END;
                    end

                    MACK_END: if (scl_fall) begin
                        state <= master_nack ? IDLE : LOAD;
                    end

                    default: ;
                endcase
            end
        end
    end

endmodule
