// =============================================================================
// Парсер результатов I2C: разбор итога транзакций и принятых данных
//
// Подсматривает команды, поток чтения и итог i2c_master (ничего не отправляет,
// rd_ready = 1) и выдаёт для индикации:
//   rd_word     - последние два принятых байта {предыдущий, последний} (датчик
//                 температуры: регистр целиком; один байт: в младших 8 битах);
//   last_addr   - адрес последней транзакции;
//   last_status - итог последней транзакции:
//                 [7] чтение, [3] arb, [2] timeout, [1] NACK данных, [0] NACK адреса;
//   txn_p       - импульс: транзакция закончена (любым итогом);
//   nack_p      - импульс: NACK на адрес;
//   err_p       - импульс: ошибка = NACK данных, timeout, arb или NACK адреса,
//                 которого не ожидали (expect_nack = 0).
// Список найденных ведомых: при found_en = 1 адрес транзакции, закончившейся
// без NACK, timeout и arb, дописывается в список (до N_FOUND штук).
// found_cnt - сколько адресов в списке, found_addr - адрес с номером found_idx.
// Сброс по импульсам:
//   clear       - rd_word, last_addr, last_status и список найденных;
//   found_clear - только список найденных (перед новым сканированием).
// =============================================================================
module i2c_res_parser #(
    parameter int N_FOUND = 8                          // длина списка найденных адресов
) (
    input  logic                       clk,
    input  logic                       rst_n,          // асинхронный сброс, активный 0

    // команда, которую принял мастер
    input  logic                       cmd_fire,       // cmd_valid && cmd_ready
    input  logic [6:0]                 cmd_addr,
    input  logic                       cmd_read,

    // итог
    input  logic                       done,
    input  logic                       res_nack_addr,
    input  logic                       res_nack_data,
    input  logic                       res_timeout,
    input  logic                       res_arb,
    input  logic                       expect_nack,

    // поток чтения
    input  logic [7:0]                 rd_data,
    input  logic                       rd_valid,
    output logic                       rd_ready,

    input  logic                       found_en,
    input  logic                       found_clear,
    input  logic                       clear,

    output logic [15:0]                rd_word,
    output logic [6:0]                 last_addr,
    output logic [7:0]                 last_status,
    output logic                       txn_p,
    output logic                       nack_p,
    output logic                       err_p,

    input  logic [$clog2(N_FOUND)-1:0] found_idx,
    output logic [6:0]                 found_addr,
    output logic [$clog2(N_FOUND):0]   found_cnt
);

    logic [6:0] cur_addr;
    logic       cur_read;
    logic [6:0] found_mem [N_FOUND];

    wire any_nack = res_nack_addr || res_nack_data;
    wire any_err  = res_nack_data || res_timeout || res_arb || (res_nack_addr && !expect_nack);

    assign rd_ready   = 1'b1;
    assign found_addr = found_mem[found_idx];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_addr    <= '0;
            cur_read    <= 1'b0;
            rd_word     <= '0;
            last_addr   <= '0;
            last_status <= '0;
            found_cnt   <= '0;
            txn_p       <= 1'b0;
            nack_p      <= 1'b0;
            err_p       <= 1'b0;
            for (int i = 0; i < N_FOUND; i++) found_mem[i] <= '0;
        end else begin
            txn_p  <= 1'b0;
            nack_p <= 1'b0;
            err_p  <= 1'b0;

            if (cmd_fire) begin
                cur_addr <= cmd_addr;
                cur_read <= cmd_read;
            end

            if (rd_valid && rd_ready) rd_word <= {rd_word[7:0], rd_data};

            if (done) begin
                last_addr   <= cur_addr;
                last_status <= {cur_read, 3'b000, res_arb, res_timeout, res_nack_data, res_nack_addr};
                txn_p       <= 1'b1;
                nack_p      <= res_nack_addr;
                err_p       <= any_err;
                if (found_en && !any_nack && !res_timeout && !res_arb && found_cnt != N_FOUND) begin
                    found_mem[found_cnt[$clog2(N_FOUND)-1:0]] <= cur_addr;
                    found_cnt <= found_cnt + 1'b1;
                end
            end

            if (clear) begin
                rd_word     <= '0;
                last_addr   <= '0;
                last_status <= '0;
            end

            if (clear || found_clear) begin
                found_cnt <= '0;
                for (int i = 0; i < N_FOUND; i++) found_mem[i] <= '0;
            end
        end
    end

endmodule
