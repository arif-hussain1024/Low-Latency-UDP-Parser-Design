// ============================================================================
// udp_parser.sv — UDP header parser
//
// Extracts: source port, destination port, UDP length, UDP checksum
// Marks start of payload region.
// UDP header starts at: ETH_HDR_BYTES + IHL*4
// ============================================================================
module udp_parser
    import udp_parser_pkg::*;
#(
    parameter int BYTE_WIDTH = 4
)(
    input  logic        clk,
    input  logic        rst_n,

    // ── Byte-level input ────────────────────────────────────────────────
    input  logic [7:0]              in_bytes [BYTE_WIDTH],
    input  logic [BYTE_WIDTH-1:0]   in_valid,
    input  logic [15:0]             byte_offset,    // absolute frame byte offset
    input  logic                    sof,

    // ── IP header info (from ipv4_parser) ───────────────────────────────
    input  logic [15:0]             ip_hdr_byte_len,    // IHL*4

    // ── Extracted fields ────────────────────────────────────────────────
    output logic [15:0]  udp_src_port,
    output logic [15:0]  udp_dst_port,
    output logic [15:0]  udp_length,
    output logic [15:0]  udp_checksum,

    output logic         udp_hdr_done,
    output logic [15:0]  payload_length,        // udp_length - 8
    output logic [15:0]  payload_start_offset   // absolute byte where payload begins
);

    logic [15:0] udp_start;     // absolute byte offset of UDP header

    // Combinational shadow of udp_length for same-cycle computation.
    // Handles the case where length bytes and completion byte arrive in
    // the same data word (possible at wider bus widths).
    logic [7:0] udp_len_hi_shadow;
    logic [7:0] udp_len_lo_shadow;
    logic       len_hi_seen, len_lo_seen;

    always_comb begin
        udp_len_hi_shadow = udp_length[15:8];   // default: registered value
        udp_len_lo_shadow = udp_length[7:0];
        len_hi_seen = 1'b0;
        len_lo_seen = 1'b0;

        for (int i = 0; i < BYTE_WIDTH; i++) begin
            if (in_valid[i] && udp_start != 0) begin
                automatic int pos     = byte_offset + i;
                automatic int udp_pos = pos - udp_start;
                if (udp_pos == 4) begin udp_len_hi_shadow = in_bytes[i]; len_hi_seen = 1'b1; end
                if (udp_pos == 5) begin udp_len_lo_shadow = in_bytes[i]; len_lo_seen = 1'b1; end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            udp_src_port         <= '0;
            udp_dst_port         <= '0;
            udp_length           <= '0;
            udp_checksum         <= '0;
            udp_hdr_done         <= 1'b0;
            payload_length       <= '0;
            payload_start_offset <= '0;
            udp_start            <= '0;
        end else if (sof) begin
            udp_src_port         <= '0;
            udp_dst_port         <= '0;
            udp_length           <= '0;
            udp_checksum         <= '0;
            udp_hdr_done         <= 1'b0;
            payload_length       <= '0;
            payload_start_offset <= '0;
            udp_start            <= '0;
        end else begin
            // Recalculate UDP start when ip_hdr_byte_len becomes valid
            if (ip_hdr_byte_len != 0 && udp_start == 0)
                udp_start <= ETH_HDR_BYTES[15:0] + ip_hdr_byte_len;

            for (int i = 0; i < BYTE_WIDTH; i++) begin
                if (in_valid[i] && udp_start != 0) begin
                    automatic int pos     = byte_offset + i;
                    automatic int udp_pos = pos - udp_start;

                    if (udp_pos >= 0 && udp_pos < UDP_HDR_BYTES && !udp_hdr_done) begin
                        case (udp_pos)
                            0: udp_src_port[15:8] <= in_bytes[i];
                            1: udp_src_port[ 7:0] <= in_bytes[i];
                            2: udp_dst_port[15:8] <= in_bytes[i];
                            3: udp_dst_port[ 7:0] <= in_bytes[i];
                            4: udp_length[15:8]   <= in_bytes[i];
                            5: udp_length[ 7:0]   <= in_bytes[i];
                            6: udp_checksum[15:8] <= in_bytes[i];
                            7: begin
                                udp_checksum[ 7:0]   <= in_bytes[i];
                                udp_hdr_done         <= 1'b1;
                                payload_start_offset <= udp_start + UDP_HDR_BYTES[15:0];
                                // Use combinational shadow to handle same-cycle arrival
                                payload_length <= {udp_len_hi_shadow, udp_len_lo_shadow} - 16'd8;
                            end
                            default: ;
                        endcase
                    end
                end
            end
        end
    end

endmodule
