// =============================================================================
// Конвейер I2C: последовательности транзакций для проверки мастера на плате
//
// По импульсу start выполняется сценарий script - заранее заданный список
// транзакций, которые подаются на i2c_master (интерфейс команд и потока записи
// такой же, как у i2c_master). Результат разбирает i2c_res_parser.
//
//   0  SCAN      проверка адресов SCAN_FIRST..SCAN_LAST (запись 0 байт): кто ответил ACK
//   1  DT_TEMP   запись указателя 0x00 в датчик (без STOP), повторный START,
//                чтение 2 байт: температура (регистр 0)
//   2  DT_TOS    то же с указателем 0x03: регистр Tos, после включения 0x5000 (80 C)
//   3  EE_READ   EEPROM: запись адреса 0x00 (без STOP), повторный START, чтение 4 байт
//   4  EE_WRITE  EEPROM: запись байта по адресу 0xF0 (значение зависит от номера
//                запуска: PATTERN_BASE + номер), ожидание конца цикла записи
//                опросом ACK (запись 0 байт, пока не будет ACK), чтение байта обратно
//   5  ABSENT    адрес, на котором ведомого нет: ожидается NACK
//
// Сценарий прерывается, если ведомый датчика или EEPROM не ответил на адрес
// или случилась ошибка (NACK данных, timeout, arb). Для SCAN, ABSENT и опроса
// ACK NACK адреса ожидаем: expect_nack = 1 на время такой транзакции.
//
// Выходы: busy - сценарий выполняется; finished - импульс в конце; error - сценарий
// прерван ошибкой (держится до следующего start); scan_active - идёт SCAN.
// =============================================================================
module i2c_seq #(
    parameter int         LEN_W        = 8,
    parameter logic [6:0] DT_ADDR      = 7'h48,   // датчик LM75A
    parameter logic [6:0] EE_ADDR      = 7'h50,   // EEPROM AT24C08, блок 0
    parameter logic [6:0] ABSENT_ADDR  = 7'h2A,   // адрес без ведомого
    parameter logic [6:0] SCAN_FIRST   = 7'h08,
    parameter logic [6:0] SCAN_LAST    = 7'h77,
    parameter logic [7:0] EE_CELL      = 8'hF0,   // ячейка EEPROM для записи
    parameter logic [7:0] PATTERN_BASE = 8'h5A,   // значение для записи: PATTERN_BASE + номер запуска
    parameter int         POLL_MAX     = 100      // предел опросов ACK после записи в EEPROM
) (
    input  logic             clk,
    input  logic             rst_n,         // асинхронный сброс, активный 0

    input  logic             start,         // импульс: выполнить сценарий script
    input  logic [2:0]       script,
    output logic             busy,
    output logic             finished,
    output logic             error,
    output logic             scan_active,
    output logic             expect_nack,

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

    // итог транзакции
    input  logic             done,
    input  logic             res_nack_addr,
    input  logic             res_nack_data,
    input  logic             res_timeout,
    input  logic             res_arb
);

    localparam logic [2:0] SC_SCAN     = 3'd0,
                           SC_DT_TEMP  = 3'd1,
                           SC_DT_TOS   = 3'd2,
                           SC_EE_READ  = 3'd3,
                           SC_EE_WRITE = 3'd4,
                           SC_ABSENT   = 3'd5;

    localparam logic [1:0] DEV_DT     = 2'd0,
                           DEV_EE     = 2'd1,
                           DEV_ABSENT = 2'd2,
                           DEV_SCAN   = 2'd3;

    // одна транзакция сценария
    typedef struct packed {
        logic       fin;        // последняя в сценарии
        logic       poll;       // повторять при NACK адреса (опрос ACK)
        logic [1:0] dev;
        logic       read;
        logic       stop;
        logic [3:0] len;        // число байт данных
        logic [7:0] d0;         // байты записи
        logic [7:0] d1;
    } step_t;

    // Quartus не даёт присваивать поля структуры в функции, поэтому собираем целиком
    function automatic step_t mk(input logic fin, input logic poll, input logic [1:0] dev,
                                 input logic read, input logic stop, input logic [3:0] len,
                                 input logic [7:0] d0, input logic [7:0] d1);
        return step_t'({fin, poll, dev, read, stop, len, d0, d1});
    endfunction

    //                                  fin   poll  dev         read  stop  len   d0     d1
    function automatic step_t get_step(input logic [2:0] sc, input logic [1:0] n, input logic [7:0] pattern);
        case (sc)
            SC_SCAN:  return mk(1'b1, 1'b0, DEV_SCAN, 1'b0, 1'b1, 4'd0, 8'h00, 8'h00);   // запись 0 байт, адрес перебирается
            SC_DT_TEMP:
                if (n == 2'd0) return mk(1'b0, 1'b0, DEV_DT, 1'b0, 1'b0, 4'd1, 8'h00, 8'h00);   // указатель 0x00, без STOP
                else           return mk(1'b1, 1'b0, DEV_DT, 1'b1, 1'b1, 4'd2, 8'h00, 8'h00);   // чтение 2 байт
            SC_DT_TOS:
                if (n == 2'd0) return mk(1'b0, 1'b0, DEV_DT, 1'b0, 1'b0, 4'd1, 8'h03, 8'h00);   // указатель 0x03
                else           return mk(1'b1, 1'b0, DEV_DT, 1'b1, 1'b1, 4'd2, 8'h00, 8'h00);
            SC_EE_READ:
                if (n == 2'd0) return mk(1'b0, 1'b0, DEV_EE, 1'b0, 1'b0, 4'd1, 8'h00, 8'h00);   // адрес ячейки 0x00
                else           return mk(1'b1, 1'b0, DEV_EE, 1'b1, 1'b1, 4'd4, 8'h00, 8'h00);   // чтение 4 байт
            SC_EE_WRITE:
                case (n)
                    2'd0:    return mk(1'b0, 1'b0, DEV_EE, 1'b0, 1'b1, 4'd2, EE_CELL, pattern);  // адрес ячейки + байт
                    2'd1:    return mk(1'b0, 1'b1, DEV_EE, 1'b0, 1'b1, 4'd0, 8'h00, 8'h00);      // опрос ACK
                    2'd2:    return mk(1'b0, 1'b0, DEV_EE, 1'b0, 1'b0, 4'd1, EE_CELL, 8'h00);    // адрес ячейки, без STOP
                    default: return mk(1'b1, 1'b0, DEV_EE, 1'b1, 1'b1, 4'd1, 8'h00, 8'h00);     // чтение байта обратно
                endcase
            default:  return mk(1'b1, 1'b0, DEV_ABSENT, 1'b0, 1'b1, 4'd0, 8'h00, 8'h00);   // SC_ABSENT
        endcase
    endfunction

    typedef enum logic [1:0] {IDLE, SEND, RUN} state_t;

    state_t      state;
    logic [2:0]  script_r;
    logic [1:0]  step_n;
    logic [6:0]  scan_addr;
    logic [7:0]  run_cnt;       // сколько раз запускали сценарии
    logic [7:0]  pattern;       // значение для записи в EEPROM в этом запуске
    logic [7:0]  polls;         // сделано опросов ACK
    logic [3:0]  byte_idx;      // сколько байт записи отдано в мастер

    step_t cur;

    assign cur = get_step(script_r, step_n, pattern);

    always_comb begin
        case (cur.dev)
            DEV_DT:     cmd_addr = DT_ADDR;
            DEV_EE:     cmd_addr = EE_ADDR;
            DEV_ABSENT: cmd_addr = ABSENT_ADDR;
            default:    cmd_addr = scan_addr;
        endcase
    end

    assign cmd_valid   = (state == SEND);
    assign cmd_read    = cur.read;
    assign cmd_len     = LEN_W'(cur.len);
    assign cmd_stop    = cur.stop;
    assign wr_valid    = (state == RUN) && !cur.read && (byte_idx < cur.len);
    assign wr_data     = (byte_idx == 4'd0) ? cur.d0 : cur.d1;
    assign busy        = (state != IDLE);
    assign scan_active = busy && (cur.dev == DEV_SCAN);
    assign expect_nack = (cur.dev == DEV_SCAN) || (cur.dev == DEV_ABSENT) || (cur.poll && polls < POLL_MAX);

    wire hard_error = res_nack_data || res_timeout || res_arb
                      || (res_nack_addr && !expect_nack);
    wire retry_poll = res_nack_addr && cur.poll && (polls < POLL_MAX);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            script_r  <= '0;
            step_n    <= '0;
            scan_addr <= SCAN_FIRST;
            run_cnt   <= '0;
            pattern   <= PATTERN_BASE;
            polls     <= '0;
            byte_idx  <= '0;
            finished  <= 1'b0;
            error     <= 1'b0;
        end else begin
            finished <= 1'b0;

            case (state)
                IDLE: begin
                    if (start) begin
                        script_r  <= script;
                        step_n    <= '0;
                        scan_addr <= SCAN_FIRST;
                        polls     <= '0;
                        error     <= 1'b0;
                        pattern   <= PATTERN_BASE + run_cnt;
                        run_cnt   <= run_cnt + 1'b1;
                        state     <= SEND;
                    end
                end

                SEND: begin
                    if (cmd_valid && cmd_ready) begin
                        byte_idx <= '0;
                        state    <= RUN;
                    end
                end

                RUN: begin
                    if (wr_valid && wr_ready) byte_idx <= byte_idx + 1'b1;

                    if (done) begin
                        if (retry_poll) begin
                            polls <= polls + 1'b1;
                            state <= SEND;
                        end else if (hard_error) begin
                            error    <= 1'b1;
                            finished <= 1'b1;
                            state    <= IDLE;
                        end else if (cur.dev == DEV_SCAN && scan_addr != SCAN_LAST) begin
                            scan_addr <= scan_addr + 1'b1;
                            state     <= SEND;
                        end else if (cur.fin) begin
                            finished <= 1'b1;
                            state    <= IDLE;
                        end else begin
                            step_n <= step_n + 1'b1;
                            polls  <= '0;
                            state  <= SEND;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
