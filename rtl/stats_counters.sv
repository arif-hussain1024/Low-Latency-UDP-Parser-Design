// ============================================================================
// stats_counters.sv — Packet / error / byte / valid-UDP counters
// ============================================================================
module stats_counters (
    input  logic        clk,
    input  logic        rst_n,

    // Event strobes (active one cycle)
    input  logic        pkt_received,       // any frame completed
    input  logic        pkt_error,          // frame with any error
    input  logic        pkt_udp_valid,      // frame delivered as valid UDP
    input  logic [15:0] byte_count,         // bytes in this frame
    input  logic        byte_count_valid,   // byte_count is valid

    // Read-side (directly wired to AXI4-Lite regs)
    output logic [31:0] total_pkt_cnt,
    output logic [31:0] error_pkt_cnt,
    output logic [63:0] total_byte_cnt,
    output logic [31:0] udp_pkt_cnt,

    // Clear
    input  logic        clear_stats
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear_stats) begin
            total_pkt_cnt  <= '0;
            error_pkt_cnt  <= '0;
            total_byte_cnt <= '0;
            udp_pkt_cnt    <= '0;
        end else begin
            if (pkt_received)
                total_pkt_cnt <= total_pkt_cnt + 1'b1;
            if (pkt_error)
                error_pkt_cnt <= error_pkt_cnt + 1'b1;
            if (byte_count_valid)
                total_byte_cnt <= total_byte_cnt + {48'h0, byte_count};
            if (pkt_udp_valid)
                udp_pkt_cnt <= udp_pkt_cnt + 1'b1;
        end
    end

endmodule
