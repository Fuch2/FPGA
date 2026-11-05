module uart_rx #(
    parameter int CLKS_PER_BIT = 434
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx,
    output logic [7:0] data,
    output logic       valid,
    output logic       error
);

    localparam int CNT_W    = $clog2(CLKS_PER_BIT + 1);
    localparam int HALF_BIT = CLKS_PER_BIT / 2;

    typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;

    state_t           state;
    logic [CNT_W-1:0] clk_cnt;
    logic [2:0]       bit_idx;
    logic [7:0]       shreg;
    logic [2:0]       rx_sync;   // синхронизатор асинхронного входа + задержка для фронта

    wire rx_s     = rx_sync[1];
    wire rx_fall  = rx_sync[2] & ~rx_sync[1];   // спад линии = начало старт-бита
    wire bit_done = (clk_cnt == CLKS_PER_BIT - 1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_sync <= 3'b111;
        else
            rx_sync <= {rx_sync[1:0], rx};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            clk_cnt <= '0;
            bit_idx <= '0;
            shreg   <= '0;
            data    <= '0;
            valid   <= 1'b0;
            error   <= 1'b0;
        end else begin
            valid   <= 1'b0;
            error   <= 1'b0;
            clk_cnt <= bit_done ? '0 : clk_cnt + 1'b1;

            case (state)
                IDLE: begin
                    clk_cnt <= '0;
                    if (rx_fall)
                        state <= START;
                end

                // ждём середину старт-бита и проверяем, что это не помеха
                START: begin
                    if (clk_cnt == HALF_BIT - 1) begin
                        clk_cnt <= '0;
                        bit_idx <= '0;
                        state   <= rx_s ? IDLE : DATA;
                    end
                end

                // дальше отсчитываем целые биты - выборка попадает в середину бита
                DATA: begin
                    if (bit_done) begin
                        shreg <= {rx_s, shreg[7:1]};   // младший бит приходит первым
                        if (bit_idx == 3'd7)
                            state <= STOP;
                        else
                            bit_idx <= bit_idx + 1'b1;
                    end
                end

                // в середине стоп-бита выдаём результат и сразу ждём следующий старт
                STOP: begin
                    if (bit_done) begin
                        if (rx_s) begin
                            data  <= shreg;
                            valid <= 1'b1;
                        end else begin
                            error <= 1'b1;
                        end
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
