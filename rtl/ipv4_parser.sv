// ============================================================================
// ipv4_parser.sv — IPv4 header parser
//
// Extracts: version, IHL, total length, protocol, src/dest IP, checksum
// Verifies: version == 4, protocol == 0x11 (UDP), header checksum
// IP header starts at Ethernet byte 14.
//
// Checksum is accumulated combinationally across all bytes in the beat
// then registered, so multi-byte-per-cycle paths are correct.
// ============================================================================
module ipv4_parser
    import udp_parser_pkg::*;
#(
    parameter int BYTE_WIDTH = 4
)(
    input  logic        clk,
    input  logic        rst_n,

    // ── Byte-level input ────────────────────────────────────────────────
    input  logic [7:0]              in_bytes [BYTE_WIDTH],
    input  logic [BYTE_WIDTH-1:0]   in_valid,
    input  logic [15:0]             byte_offset,    // absolute frame offset of in_bytes[0]
    input  logic                    sof,

    // ── Extracted fields ────────────────────────────────────────────────
    output logic [3:0]   ip_version,
    output logic [3:0]   ip_ihl,
    output logic [15:0]  ip_total_length,
    output logic [7:0]   ip_protocol,
    output logic [31:0]  ip_src_addr,
    output logic [31:0]  ip_dst_addr,
    output logic [15:0]  ip_header_checksum,

    output logic         ip_hdr_done,
    output logic         ip_hdr_valid,
    output logic         bad_ip_version,
    output logic         bad_ip_checksum,
    output logic         non_udp,

    // ── Derived ─────────────────────────────────────────────────────────
    output logic [15:0]  ip_payload_length,     // total_length - ihl*4
    output logic [15:0]  ip_hdr_byte_len        // ihl*4
);

    // IP header starts at absolute byte 14 (after 14-byte Ethernet header)
    localparam int IP_START = ETH_HDR_BYTES;

    // ── Registered state ────────────────────────────────────────────────
    logic [31:0] cksum_accum;       // checksum accumulator (may have carries)
    logic [7:0]  cksum_byte_hi;     // holds high byte for 16-bit pairing
    logic        cksum_byte_pending;
    logic        cksum_fold_done;
    logic [3:0]  ihl_captured;

    // ── Combinational checksum chain ────────────────────────────────────
    // Properly chains across all bytes in a single beat so multi-byte
    // arrivals accumulate correctly.
    logic [31:0] cksum_chain;
    logic [7:0]  hi_chain;
    logic        hi_pending_chain;

    always_comb begin
        cksum_chain      = cksum_accum;
        hi_chain         = cksum_byte_hi;
        hi_pending_chain = cksum_byte_pending;

        for (int i = 0; i < BYTE_WIDTH; i++) begin
            if (in_valid[i]) begin
                automatic int pos    = byte_offset + i;
                automatic int ip_pos = pos - IP_START;

                if (ip_pos >= 0 && ip_pos < 40 && !ip_hdr_done) begin // 40 = max IHL*4
                    if (ip_pos[0] == 0) begin
                        // Even byte = high byte of 16-bit word
                        hi_chain         = in_bytes[i];
                        hi_pending_chain = 1'b1;
                    end else begin
                        // Odd byte = low byte → form and add 16-bit word
                        cksum_chain      = cksum_chain + {16'h0, hi_chain, in_bytes[i]};
                        hi_pending_chain = 1'b0;
                    end
                end
            end
        end
    end

    // ── Sequential logic ────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ip_version          <= '0;
            ip_ihl              <= '0;
            ip_total_length     <= '0;
            ip_protocol         <= '0;
            ip_src_addr         <= '0;
            ip_dst_addr         <= '0;
            ip_header_checksum  <= '0;
            ip_hdr_done         <= 1'b0;
            ip_hdr_valid        <= 1'b0;
            bad_ip_version      <= 1'b0;
            bad_ip_checksum     <= 1'b0;
            non_udp             <= 1'b0;
            ip_payload_length   <= '0;
            ip_hdr_byte_len     <= '0;
            cksum_accum         <= '0;
            cksum_byte_hi       <= '0;
            cksum_byte_pending  <= 1'b0;
            cksum_fold_done     <= 1'b0;
            ihl_captured        <= '0;
        end else if (sof) begin
            ip_version          <= '0;
            ip_ihl              <= '0;
            ip_total_length     <= '0;
            ip_protocol         <= '0;
            ip_src_addr         <= '0;
            ip_dst_addr         <= '0;
            ip_header_checksum  <= '0;
            ip_hdr_done         <= 1'b0;
            ip_hdr_valid        <= 1'b0;
            bad_ip_version      <= 1'b0;
            bad_ip_checksum     <= 1'b0;
            non_udp             <= 1'b0;
            ip_payload_length   <= '0;
            ip_hdr_byte_len     <= '0;
            cksum_accum         <= '0;
            cksum_byte_hi       <= '0;
            cksum_byte_pending  <= 1'b0;
            cksum_fold_done     <= 1'b0;
            ihl_captured        <= '0;
        end else begin

            // ── Register combinational checksum chain output ────────────
            cksum_accum        <= cksum_chain;
            cksum_byte_hi      <= hi_chain;
            cksum_byte_pending <= hi_pending_chain;

            // ── Extract header fields ───────────────────────────────────
            for (int i = 0; i < BYTE_WIDTH; i++) begin
                if (in_valid[i]) begin
                    automatic int pos    = byte_offset + i;
                    automatic int ip_pos = pos - IP_START;

                    if (ip_pos >= 0 && !ip_hdr_done) begin
                        case (ip_pos)
                            0: begin
                                ip_version   <= in_bytes[i][7:4];
                                ip_ihl       <= in_bytes[i][3:0];
                                ihl_captured <= in_bytes[i][3:0];
                            end
                            2: ip_total_length[15:8] <= in_bytes[i];
                            3: ip_total_length[ 7:0] <= in_bytes[i];
                            9: ip_protocol           <= in_bytes[i];
                            10: ip_header_checksum[15:8] <= in_bytes[i];
                            11: ip_header_checksum[ 7:0] <= in_bytes[i];
                            12: ip_src_addr[31:24] <= in_bytes[i];
                            13: ip_src_addr[23:16] <= in_bytes[i];
                            14: ip_src_addr[15: 8] <= in_bytes[i];
                            15: ip_src_addr[ 7: 0] <= in_bytes[i];
                            16: ip_dst_addr[31:24] <= in_bytes[i];
                            17: ip_dst_addr[23:16] <= in_bytes[i];
                            18: ip_dst_addr[15: 8] <= in_bytes[i];
                            19: ip_dst_addr[ 7: 0] <= in_bytes[i];
                            default: ;
                        endcase

                        // Header complete when we've consumed IHL*4 bytes
                        if (ihl_captured != 0 &&
                            ip_pos == (ihl_captured * 4 - 1)) begin
                            ip_hdr_done     <= 1'b1;
                            ip_hdr_byte_len <= {12'h0, ihl_captured} << 2;
                        end
                    end
                end
            end

            // ── Fold checksum & validate ────────────────────────────────
            // Runs one cycle after ip_hdr_done to let cksum_accum settle
            if (ip_hdr_done && !cksum_fold_done) begin
                if (cksum_accum[31:16] != '0) begin
                    cksum_accum <= {16'h0, cksum_accum[15:0]} +
                                   {16'h0, cksum_accum[31:16]};
                end else begin
                    cksum_fold_done <= 1'b1;
                    bad_ip_version  <= (ip_version != IP_VERSION_4);
                    bad_ip_checksum <= (cksum_accum[15:0] != 16'hFFFF);
                    non_udp         <= (ip_protocol != IP_PROTO_UDP);
                    ip_hdr_valid    <= (ip_version == IP_VERSION_4) &&
                                      (cksum_accum[15:0] == 16'hFFFF) &&
                                      (ip_protocol == IP_PROTO_UDP);
                    ip_payload_length <= ip_total_length -
                                        ({12'h0, ihl_captured} << 2);
                end
            end
        end
    end

endmodule
