// =============================================================================
// Чтение датчика температуры LM75A через i2c_master
//
// По импульсу start_read выполняет (если не занят):
//   1. только при WRITE_CONFIG = 1 и только в первом чтении после сброса:
//      запись регистра конфигурации (указатель 0x01, значение CONFIG_VALUE; 0x00 -
//      рабочий режим, не shutdown);
//   2. запись указателя на регистр температуры (0x00) без STOP;
//   3. повторный START и чтение двух байт: старший, младший.
// Если датчик не ответил (NACK адреса или данных), заняты линии или SCL удерживается
// дольше предела, чтение прекращается и выдаётся импульс read_fail; temp_data
// остаётся прежним, конфигурация будет записана при следующем чтении.
//
// Выход - температура в формате Q8.8 со знаком из PROTOCOL.md (1 LSB = 1/256 °C):
// регистр датчика 11 значащих бит D15..D5, шаг 0.125 °C, младшие 5 бит датчика
// маскируются в 0. Пример: 0x1960 = 25.375 °C, 0xFF00 = -1.0 °C.
//   temp_valid - импульс на 1 такт, temp_data обновлён;
//   busy       - чтение выполняется;
//   read_fail  - импульс: чтение не удалось.
//
// Интерфейс к i2c_master (команда и поток записи valid/ready, поток чтения, итог)
// повторяет порты i2c_master; потоки чтения принимаются сразу (rd_ready = 1).
// =============================================================================
module dt_reader #(
    parameter int          LEN_W        = 8,
    parameter logic [6:0]  DT_ADDR      = 7'h48,   // адрес LM75A
    parameter logic [7:0]  REG_TEMP     = 8'h00,   // указатель: регистр температуры
    parameter logic [7:0]  REG_CONFIG   = 8'h01,   // указатель: регистр конфигурации
    parameter bit          WRITE_CONFIG = 1'b1,    // писать конфигурацию перед первым чтением
    parameter logic [7:0]  CONFIG_VALUE = 8'h00    // 0x00 - обычный режим, нормальный опрос
) (
    input  logic             clk,
    input  logic             rst_n,         // асинхронный сброс, активный 0

    input  logic             start_read,
    output logic             busy,
    output logic             temp_valid,
    output logic [15:0]      temp_data,
    output logic             read_fail,

    // команды и поток записи для i2c_master
    output logic             cmd_valid,
    input  logic             cmd_ready,
    output logic [6:0]       cmd_addr,
    output logic             cmd_read,
    output logic [LEN_W-1:0] cmd_len,
    output logic             cmd_stop,
    output logic [7:0]       wr_data,
    output logic             wr_valid,
    input  logic             wr_ready,

    // поток чтения
    input  logic [7:0]       rd_data,
    input  logic             rd_valid,
    output logic             rd_ready,
    input  logic             rd_last,

    // итог транзакции
    input  logic             done,
    input  logic             res_nack_addr,
    input  logic             res_nack_data,
    input  logic             res_timeout,
    input  logic             res_arb
);

    // каждая транзакция - два состояния: SEND (выдаём команду) и RUN (ждём итог)
    typedef enum logic [2:0] {
        IDLE,
        CFG_SEND, CFG_RUN,      // запись конфигурации: указатель + значение
        PTR_SEND, PTR_RUN,      // запись указателя температуры, без STOP
        RD_SEND,  RD_RUN        // чтение двух байт
    } state_t;

    state_t     state;
    logic       configured;     // конфигурация уже записана
    logic       byte_idx;       // сколько байт записи отдано в мастер (0 или 1)
    logic       rd_idx;         // 0 - ждём старший байт, 1 - младший
    logic [7:0] raw_hi, raw_lo;

    wire in_cfg = (state == CFG_SEND) || (state == CFG_RUN);
    wire in_ptr = (state == PTR_SEND) || (state == PTR_RUN);
    wire fail   = res_nack_addr || res_nack_data || res_timeout || res_arb;

    assign busy      = (state != IDLE);
    assign cmd_valid = (state == CFG_SEND) || (state == PTR_SEND) || (state == RD_SEND);
    assign cmd_addr  = DT_ADDR;
    assign cmd_read  = (state == RD_SEND);
    assign cmd_len   = in_cfg ? LEN_W'(2) : in_ptr ? LEN_W'(1) : LEN_W'(2);
    assign cmd_stop  = !in_ptr;
    assign wr_valid  = (state == CFG_RUN) || (state == PTR_RUN);
    assign wr_data   = in_cfg ? (byte_idx ? CONFIG_VALUE : REG_CONFIG) : REG_TEMP;
    assign rd_ready  = (state == RD_RUN);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            configured <= 1'b0;
            byte_idx   <= 1'b0;
            rd_idx     <= 1'b0;
            raw_hi     <= '0;
            raw_lo     <= '0;
            temp_data  <= '0;
            temp_valid <= 1'b0;
            read_fail  <= 1'b0;
        end else begin
            temp_valid <= 1'b0;
            read_fail  <= 1'b0;

            case (state)
                IDLE: begin
                    if (start_read) begin
                        rd_idx   <= 1'b0;
                        byte_idx <= 1'b0;
                        state    <= (WRITE_CONFIG && !configured) ? CFG_SEND : PTR_SEND;
                    end
                end

                CFG_SEND, PTR_SEND, RD_SEND: begin
                    if (cmd_valid && cmd_ready) begin
                        byte_idx <= 1'b0;
                        state    <= state_t'(state + 1'b1);
                    end
                end

                CFG_RUN, PTR_RUN: begin
                    if (wr_valid && wr_ready) byte_idx <= 1'b1;
                    if (done) begin
                        if (fail) begin
                            read_fail <= 1'b1;
                            state     <= IDLE;
                        end else begin
                            if (state == CFG_RUN) configured <= 1'b1;
                            state <= state_t'(state + 1'b1);
                        end
                    end
                end

                RD_RUN: begin
                    if (rd_valid && rd_ready) begin
                        rd_idx <= 1'b1;
                        if (!rd_idx) raw_hi <= rd_data;
                        else         raw_lo <= rd_data;
                    end
                    if (done) begin
                        if (fail || !rd_idx) begin
                            read_fail <= 1'b1;
                        end else begin
                            temp_data  <= {raw_hi, raw_lo & 8'hE0};
                            temp_valid <= 1'b1;
                        end
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
