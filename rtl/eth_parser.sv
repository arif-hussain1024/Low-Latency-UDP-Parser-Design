// ============================================================================
// eth_parser.sv — Ethernet frame parser
//
// • Detects preamble / SFD (optional, controlled externally)
// • Extracts destination MAC (6 B), source MAC (6 B), EtherType (2 B)
// • Drives mac_filter for destination MAC check
// • Reports when Ethernet header is complete and its validity
//
// The caller streams in individual bytes (up to BYTE_WIDTH per cycle)
// together with the absolute byte position inside the frame.
// ============================================================================
module eth_parser
    import udp_parser_pkg::*;
#(
    parameter int BYTE_WIDTH = 4
)(
    input  logic        clk,
    input  logic        rst_n,

    // ── Byte-level input from top FSM ───────────────────────────────────
    input  logic [7:0]              in_bytes [BYTE_WIDTH],
    input  logic [BYTE_WIDTH-1:0]   in_valid,       // per-byte valid
    input  logic [15:0]             byte_offset,    // absolute offset of in_bytes[0]
    input  logic                    sof,            // start-of-frame pulse

    // ── Extracted fields ────────────────────────────────────────────────
    output logic [47:0]  dest_mac,
    output logic [47:0]  src_mac,
    output logic [15:0]  ethertype,
    output logic         eth_hdr_done,      // header fully parsed (registered)
    output logic         eth_hdr_valid,     // MAC + EtherType OK
    output logic         bad_ethertype,
    output logic         bad_mac,

    // ── MAC filter configuration ────────────────────────────────────────
    input  logic [47:0]  cfg_mac_addr,
    input  logic         cfg_promiscuous
);

    // ── Header byte accumulator ─────────────────────────────────────────
    // Ethernet header = 14 bytes: DMAC[5..0] SMAC[5..0] EtherType[1..0]
    // Stored big-endian (network order):
    //   dest_mac[47:40] = byte 0, dest_mac[7:0] = byte 5
    //   ethertype[15:8] = byte 12, ethertype[7:0] = byte 13

    logic mac_match;
    logic hdr_bytes_complete;   // combinational: all 14 bytes seen this cycle

    // MAC filter instance
    mac_filter u_mac_filter (
        .dest_mac   (dest_mac),
        .config_mac (cfg_mac_addr),
        .promiscuous(cfg_promiscuous),
        .valid      (eth_hdr_done),
        .match      (mac_match)
    );

    // Combinational detection of last header byte (byte 13) in this beat
    always_comb begin
        hdr_bytes_complete = 1'b0;
        for (int i = 0; i < BYTE_WIDTH; i++) begin
            if (in_valid[i] && (byte_offset + i) == 13)
                hdr_bytes_complete = 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dest_mac       <= '0;
            src_mac        <= '0;
            ethertype      <= '0;
            eth_hdr_done   <= 1'b0;
            eth_hdr_valid  <= 1'b0;
            bad_ethertype  <= 1'b0;
            bad_mac        <= 1'b0;
        end else if (sof) begin
            dest_mac       <= '0;
            src_mac        <= '0;
            ethertype      <= '0;
            eth_hdr_done   <= 1'b0;
            eth_hdr_valid  <= 1'b0;
            bad_ethertype  <= 1'b0;
            bad_mac        <= 1'b0;
        end else begin
            // Accumulate header bytes
            for (int i = 0; i < BYTE_WIDTH; i++) begin
                if (in_valid[i]) begin
                    automatic int pos = byte_offset + i;

                    // Dest MAC: bytes 0..5
                    if (pos >= 0 && pos <= 5)
                        dest_mac[(5-pos)*8 +: 8] <= in_bytes[i];

                    // Src MAC: bytes 6..11
                    if (pos >= 6 && pos <= 11)
                        src_mac[(11-pos)*8 +: 8] <= in_bytes[i];

                    // EtherType: bytes 12..13
                    if (pos == 12) ethertype[15:8] <= in_bytes[i];
                    if (pos == 13) ethertype[ 7:0] <= in_bytes[i];

                    // Header complete after byte 13
                    if (pos == 13)
                        eth_hdr_done <= 1'b1;
                end
            end

            // Validate one cycle after header complete (mac_filter needs
            // registered dest_mac to be stable)
            if (eth_hdr_done && !eth_hdr_valid && !bad_ethertype && !bad_mac) begin
                bad_ethertype <= (ethertype != ETH_TYPE_IPV4);
                bad_mac       <= ~mac_match;
                eth_hdr_valid <= (ethertype == ETH_TYPE_IPV4) && mac_match;
            end
        end
    end

endmodule
