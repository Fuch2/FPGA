// =============================================================================
// Парсер команд МО -> МИ на стороне SPI-слейва (PROTOCOL.md, раздел 6.1, 6.2)
//
// Кадр - ровно FRAME_LEN = 4 байта между спадом и подъёмом CS_N; команда - первый
// байт кадра, остальные байты игнорируются. Команда исполняется после подъёма
// CS_N (frame_end), поэтому ответ на неё МО получает в следующей транзакции:
//   CMD_START   (0xA1) - импульс cmd_start;
//   CMD_MEASURE (0xA2) - импульс cmd_measure;
//   CMD_STOP    (0xA3) - импульс cmd_stop;
//   CMD_NOP     (0x00) - импульс cmd_nop (кадр только для чтения ответа);
//   другой байт        - импульс cmd_unknown, команда игнорируется.
// Кадр неверной длины (не 4 байта) или оборванный посреди байта (frame_short)
// не исполняется: импульс frame_err.
// Все выходы - импульсы на 1 такт после frame_end. last_cmd - первый байт
// последнего кадра правильной длины (для индикации).
//
// Входы - события и байты spi_slave (frame_start, frame_end, frame_short,
// rx_data, rx_valid).
// =============================================================================
module spi_cmd_parser #(
    parameter int         FRAME_LEN   = 4,
    parameter logic [7:0] CMD_START   = 8'hA1,
    parameter logic [7:0] CMD_MEASURE = 8'hA2,
    parameter logic [7:0] CMD_STOP    = 8'hA3,
    parameter logic [7:0] CMD_NOP     = 8'h00
) (
    input  logic       clk,
    input  logic       rst_n,         // асинхронный сброс, активный 0

    input  logic       frame_start,
    input  logic       frame_end,
    input  logic       frame_short,
    input  logic [7:0] rx_data,
    input  logic       rx_valid,

    output logic       cmd_start,
    output logic       cmd_measure,
    output logic       cmd_stop,
    output logic       cmd_nop,
    output logic       cmd_unknown,
    output logic       frame_err,
    output logic [7:0] last_cmd
);

    localparam int CNT_W = $clog2(FRAME_LEN + 2);

    logic [CNT_W-1:0] byte_cnt;       // принято байт в кадре (насыщается на FRAME_LEN + 1)
    logic [7:0]       first_byte;     // первый байт кадра

    // с учётом байта, пришедшего в этом же такте, что и frame_end
    wire [CNT_W-1:0] cnt_now = (rx_valid && byte_cnt != CNT_W'(FRAME_LEN + 1)) ? byte_cnt + 1'b1 : byte_cnt;
    wire [7:0]       cmd_now = (rx_valid && byte_cnt == 0) ? rx_data : first_byte;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt    <= '0;
            first_byte  <= '0;
            last_cmd    <= '0;
            cmd_start   <= 1'b0;
            cmd_measure <= 1'b0;
            cmd_stop    <= 1'b0;
            cmd_nop     <= 1'b0;
            cmd_unknown <= 1'b0;
            frame_err   <= 1'b0;
        end else begin
            cmd_start   <= 1'b0;
            cmd_measure <= 1'b0;
            cmd_stop    <= 1'b0;
            cmd_nop     <= 1'b0;
            cmd_unknown <= 1'b0;
            frame_err   <= 1'b0;

            if (frame_start) begin
                byte_cnt <= '0;
            end else begin
                byte_cnt <= cnt_now;
                if (rx_valid && byte_cnt == 0)
                    first_byte <= rx_data;
            end

            if (frame_end) begin
                if (frame_short || cnt_now != CNT_W'(FRAME_LEN)) begin
                    frame_err <= 1'b1;
                end else begin
                    last_cmd <= cmd_now;
                    case (cmd_now)
                        CMD_START:   cmd_start   <= 1'b1;
                        CMD_MEASURE: cmd_measure <= 1'b1;
                        CMD_STOP:    cmd_stop    <= 1'b1;
                        CMD_NOP:     cmd_nop     <= 1'b1;
                        default:     cmd_unknown <= 1'b1;
                    endcase
                end
            end
        end
    end

endmodule
