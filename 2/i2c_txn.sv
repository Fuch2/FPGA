// =============================================================================
// I2C, уровень транзакций: адрес, чтение/запись, число байт, потоки данных
//
// Одна команда = START (или повторный START), адрес + R/W, ACK адреса, байты
// данных, затем STOP или, при cmd_stop = 0, шина остаётся занятой: следующая
// команда начнётся с повторного START (чтение регистра: запись указателя
// без STOP + чтение).
//
// Команда (valid/ready), принимается в такте cmd_valid = 1 и cmd_ready = 1:
//   cmd_addr - 7-битный адрес ведомого;
//   cmd_read - 0 запись, 1 чтение;
//   cmd_len  - число байт данных. Для записи 0 допустимо - это проверка
//              адреса (ведомый ответил ACK или нет); для чтения 0 означает 1;
//   cmd_stop - 1: STOP в конце; 0: без STOP.
// Поток записи (valid/ready): wr_data принимается в такте wr_valid = 1 и
//   wr_ready = 1; wr_ready поднимается, когда модулю нужен очередной байт.
//   Пока байта нет, SCL удерживается в 0.
// Поток чтения (valid/ready): rd_data - принятый байт, rd_last = 1 у последнего
//   байта команды. Пока rd_ready = 0, SCL удерживается в 0. Последний байт
//   подтверждается NACK, остальные ACK.
// Итог - импульс done на 1 такт, флаги и res_len держатся до следующей команды:
//   res_len       - сколько байт данных передано и подтверждено (записано или прочитано);
//   res_nack_addr - ведомый не ответил на адрес (нет на шине);
//   res_nack_data - ведомый не подтвердил байт данных записи;
//   res_timeout   - ведомый удерживал SCL дольше предела (см. i2c_bit);
//   res_arb       - шина занята или SDA не соответствует выставленному.
// После NACK всегда выдаётся STOP, даже при cmd_stop = 0. После timeout и arb
// STOP не выдаётся: i2c_bit отпускает обе линии.
// События для индикации (импульсы на 1 такт): ev_start, ev_stop, ev_ack
// (ведомый подтвердил адрес или байт), ev_nack (ведомый не подтвердил).
//
// Нижний уровень - i2c_bit (интерфейс bit_*: команда valid/ready и ответ).
// =============================================================================
module i2c_txn #(
    parameter int LEN_W = 8                     // разрядность числа байт
) (
    input  logic             clk,
    input  logic             rst_n,             // асинхронный сброс, активный 0

    // команда
    input  logic             cmd_valid,
    output logic             cmd_ready,
    input  logic [6:0]       cmd_addr,
    input  logic             cmd_read,
    input  logic [LEN_W-1:0] cmd_len,
    input  logic             cmd_stop,

    // поток записи
    input  logic [7:0]       wr_data,
    input  logic             wr_valid,
    output logic             wr_ready,

    // поток чтения
    output logic [7:0]       rd_data,
    output logic             rd_valid,
    input  logic             rd_ready,
    output logic             rd_last,

    // итог
    output logic             done,
    output logic [LEN_W-1:0] res_len,
    output logic             res_nack_addr,
    output logic             res_nack_data,
    output logic             res_timeout,
    output logic             res_arb,

    // события
    output logic             ev_start,
    output logic             ev_stop,
    output logic             ev_ack,
    output logic             ev_nack,

    // нижний уровень (i2c_bit)
    output logic [1:0]       bit_cmd,
    output logic             bit_val,
    output logic             bit_valid,
    input  logic             bit_ready,
    input  logic             bit_rsp_valid,
    input  logic             bit_rsp_bit,
    input  logic             bit_rsp_timeout,
    input  logic             bit_rsp_arb
);

    localparam logic [1:0] BIT_START = 2'd0,
                           BIT_STOP  = 2'd1,
                           BIT_WRITE = 2'd2,
                           BIT_READ  = 2'd3;

    typedef enum logic [3:0] {
        IDLE,
        START,      // ждём конец START
        ADDR,       // передаём 8 бит: адрес и R/W
        ADDR_ACK,   // ACK адреса
        WR_WAIT,    // ждём байт для записи
        WR_DATA,    // передаём 8 бит данных
        WR_ACK,     // ACK байта
        RD_DATA,    // принимаем 8 бит данных
        RD_PUSH,    // ждём, пока принятый байт заберут
        RD_ACK,     // отправляем ACK / NACK
        STOP,       // ждём конец STOP
        FINISH      // выдаём done
    } state_t;

    state_t           state;
    logic [6:0]       addr_r;
    logic             read_r, stop_r;
    logic [LEN_W-1:0] remaining;   // осталось байт данных
    logic [7:0]       shreg;       // сдвиговый регистр: старший бит - очередной на линию
    logic [2:0]       bit_idx;     // сколько бит байта уже передано / принято

    wire bit_err = bit_rsp_valid && (bit_rsp_timeout || bit_rsp_arb);

    assign cmd_ready = (state == IDLE);
    assign wr_ready  = (state == WR_WAIT);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            addr_r        <= '0;
            read_r        <= 1'b0;
            stop_r        <= 1'b0;
            remaining     <= '0;
            shreg         <= '0;
            bit_idx       <= '0;
            rd_data       <= '0;
            rd_valid      <= 1'b0;
            rd_last       <= 1'b0;
            done          <= 1'b0;
            res_len       <= '0;
            res_nack_addr <= 1'b0;
            res_nack_data <= 1'b0;
            res_timeout   <= 1'b0;
            res_arb       <= 1'b0;
            ev_start      <= 1'b0;
            ev_stop       <= 1'b0;
            ev_ack        <= 1'b0;
            ev_nack       <= 1'b0;
            bit_cmd       <= BIT_START;
            bit_val       <= 1'b0;
            bit_valid     <= 1'b0;
        end else begin
            done     <= 1'b0;
            ev_start <= 1'b0;
            ev_stop  <= 1'b0;
            ev_ack   <= 1'b0;
            ev_nack  <= 1'b0;
            if (bit_valid && bit_ready) bit_valid <= 1'b0;

            if (bit_err) begin
                // аварии нижнего уровня прерывают любую транзакцию без STOP
                res_timeout <= bit_rsp_timeout;
                res_arb     <= bit_rsp_arb;
                state       <= FINISH;
            end else begin
                case (state)
                    IDLE: begin
                        if (cmd_valid) begin
                            addr_r        <= cmd_addr;
                            read_r        <= cmd_read;
                            stop_r        <= cmd_stop;
                            remaining     <= (cmd_read && cmd_len == '0) ? LEN_W'(1) : cmd_len;
                            res_len       <= '0;
                            res_nack_addr <= 1'b0;
                            res_nack_data <= 1'b0;
                            res_timeout   <= 1'b0;
                            res_arb       <= 1'b0;
                            bit_cmd       <= BIT_START;
                            bit_valid     <= 1'b1;
                            state         <= START;
                        end
                    end

                    START: begin
                        if (bit_rsp_valid) begin
                            ev_start  <= 1'b1;
                            shreg     <= {addr_r, read_r};
                            bit_idx   <= '0;
                            bit_cmd   <= BIT_WRITE;
                            bit_val   <= addr_r[6];
                            bit_valid <= 1'b1;
                            state     <= ADDR;
                        end
                    end

                    ADDR: begin
                        if (bit_rsp_valid) begin
                            bit_valid <= 1'b1;
                            if (bit_idx == 3'd7) begin
                                bit_cmd <= BIT_READ;
                                state   <= ADDR_ACK;
                            end else begin
                                shreg   <= shreg << 1;
                                bit_idx <= bit_idx + 1'b1;
                                bit_val <= shreg[6];
                            end
                        end
                    end

                    ADDR_ACK: begin
                        if (bit_rsp_valid) begin
                            bit_idx <= '0;
                            if (bit_rsp_bit) begin                  // NACK: на шине нет такого ведомого
                                res_nack_addr <= 1'b1;
                                ev_nack       <= 1'b1;
                                bit_cmd       <= BIT_STOP;
                                bit_valid     <= 1'b1;
                                state         <= STOP;
                            end else begin
                                ev_ack <= 1'b1;
                                if (remaining == '0) begin          // только проверка адреса
                                    if (stop_r) begin
                                        bit_cmd   <= BIT_STOP;
                                        bit_valid <= 1'b1;
                                        state     <= STOP;
                                    end else begin
                                        state <= FINISH;
                                    end
                                end else if (read_r) begin
                                    bit_cmd   <= BIT_READ;
                                    bit_valid <= 1'b1;
                                    state     <= RD_DATA;
                                end else begin
                                    state <= WR_WAIT;
                                end
                            end
                        end
                    end

                    WR_WAIT: begin
                        if (wr_valid) begin
                            shreg     <= wr_data;
                            bit_idx   <= '0;
                            bit_cmd   <= BIT_WRITE;
                            bit_val   <= wr_data[7];
                            bit_valid <= 1'b1;
                            state     <= WR_DATA;
                        end
                    end

                    WR_DATA: begin
                        if (bit_rsp_valid) begin
                            bit_valid <= 1'b1;
                            if (bit_idx == 3'd7) begin
                                bit_cmd <= BIT_READ;
                                state   <= WR_ACK;
                            end else begin
                                shreg   <= shreg << 1;
                                bit_idx <= bit_idx + 1'b1;
                                bit_val <= shreg[6];
                            end
                        end
                    end

                    WR_ACK: begin
                        if (bit_rsp_valid) begin
                            if (bit_rsp_bit) begin                  // ведомый не принял байт
                                res_nack_data <= 1'b1;
                                ev_nack       <= 1'b1;
                                bit_cmd       <= BIT_STOP;
                                bit_valid     <= 1'b1;
                                state         <= STOP;
                            end else begin
                                ev_ack    <= 1'b1;
                                res_len   <= res_len + 1'b1;
                                remaining <= remaining - 1'b1;
                                if (remaining == LEN_W'(1)) begin
                                    if (stop_r) begin
                                        bit_cmd   <= BIT_STOP;
                                        bit_valid <= 1'b1;
                                        state     <= STOP;
                                    end else begin
                                        state <= FINISH;
                                    end
                                end else begin
                                    state <= WR_WAIT;
                                end
                            end
                        end
                    end

                    RD_DATA: begin
                        if (bit_rsp_valid) begin
                            shreg <= {shreg[6:0], bit_rsp_bit};
                            if (bit_idx == 3'd7) begin
                                rd_data  <= {shreg[6:0], bit_rsp_bit};
                                rd_last  <= (remaining == LEN_W'(1));
                                rd_valid <= 1'b1;
                                state    <= RD_PUSH;
                            end else begin
                                bit_idx   <= bit_idx + 1'b1;
                                bit_valid <= 1'b1;
                            end
                        end
                    end

                    // байт принят, ждём rd_ready; ответ ведомому: ACK, на последнем байте NACK
                    RD_PUSH: begin
                        if (rd_valid && rd_ready) begin
                            rd_valid  <= 1'b0;
                            bit_cmd   <= BIT_WRITE;
                            bit_val   <= (remaining == LEN_W'(1));
                            bit_valid <= 1'b1;
                            state     <= RD_ACK;
                        end
                    end

                    RD_ACK: begin
                        if (bit_rsp_valid) begin
                            res_len   <= res_len + 1'b1;
                            remaining <= remaining - 1'b1;
                            if (remaining == LEN_W'(1)) begin
                                if (stop_r) begin
                                    bit_cmd   <= BIT_STOP;
                                    bit_valid <= 1'b1;
                                    state     <= STOP;
                                end else begin
                                    state <= FINISH;
                                end
                            end else begin
                                bit_idx   <= '0;
                                bit_cmd   <= BIT_READ;
                                bit_valid <= 1'b1;
                                state     <= RD_DATA;
                            end
                        end
                    end

                    STOP: begin
                        if (bit_rsp_valid) begin
                            ev_stop <= 1'b1;
                            state   <= FINISH;
                        end
                    end

                    FINISH: begin
                        done  <= 1'b1;
                        state <= IDLE;
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

endmodule
