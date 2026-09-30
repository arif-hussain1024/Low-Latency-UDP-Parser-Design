// ============================================================================
// udp_parser_pkg.sv — Protocol constants, typedefs, register map, CRC-32
// ============================================================================
package udp_parser_pkg;

    // ── Protocol constants ──────────────────────────────────────────────────
    localparam logic [15:0] ETH_TYPE_IPV4   = 16'h0800;
    localparam logic  [7:0] IP_PROTO_UDP    = 8'h11;
    localparam logic  [3:0] IP_VERSION_4    = 4'h4;

    localparam logic  [7:0] PREAMBLE_BYTE   = 8'h55;
    localparam logic  [7:0] SFD_BYTE        = 8'hD5;

    // ── Header sizes (bytes) ────────────────────────────────────────────────
    localparam int ETH_HDR_BYTES    = 14;
    localparam int IP_HDR_MIN_BYTES = 20;
    localparam int UDP_HDR_BYTES    = 8;
    localparam int FCS_BYTES        = 4;
    localparam int MIN_FRAME_BYTES  = 64;   // minimum Ethernet frame (incl. FCS)

    // ── AXI4-Lite register offsets ──────────────────────────────────────────
    localparam logic [7:0] REG_MAC_LO       = 8'h00;
    localparam logic [7:0] REG_MAC_HI       = 8'h04;
    localparam logic [7:0] REG_CONFIG       = 8'h08;
    localparam logic [7:0] REG_STATUS       = 8'h0C;
    localparam logic [7:0] REG_PKT_CNT      = 8'h10;
    localparam logic [7:0] REG_ERR_CNT      = 8'h14;
    localparam logic [7:0] REG_BYTE_CNT_LO  = 8'h18;
    localparam logic [7:0] REG_BYTE_CNT_HI  = 8'h1C;
    localparam logic [7:0] REG_UDP_CNT      = 8'h20;
    localparam logic [7:0] REG_TS_LO        = 8'h24;
    localparam logic [7:0] REG_TS_HI        = 8'h28;
    localparam logic [7:0] REG_LATENCY      = 8'h2C;

    // ── Config register bit-fields ──────────────────────────────────────────
    localparam int CFG_BIT_ENABLE       = 0;
    localparam int CFG_BIT_PROMISCUOUS  = 1;

    // ── CRC-32 (Ethernet, reflected / LSB-first) ────────────────────────────
    localparam logic [31:0] CRC32_POLY    = 32'hEDB88320;
    localparam logic [31:0] CRC32_INIT    = 32'hFFFF_FFFF;
    localparam logic [31:0] CRC32_RESIDUE = 32'hDEBB_20E3;

    // ── Error flags ─────────────────────────────────────────────────────────
    typedef struct packed {
        logic runt_frame;       // [7]
        logic truncated;        // [6]
        logic non_udp;          // [5]
        logic bad_ip_version;   // [4]
        logic bad_ethertype;    // [3]
        logic bad_mac;          // [2]
        logic bad_ip_checksum;  // [1]
        logic bad_fcs;          // [0]
    } error_flags_t;

    // ── Parser FSM states ───────────────────────────────────────────────────
    typedef enum logic [3:0] {
        S_IDLE      = 4'd0,
        S_PREAMBLE  = 4'd1,
        S_ETH_HDR   = 4'd2,
        S_IP_HDR    = 4'd3,
        S_UDP_HDR   = 4'd4,
        S_PAYLOAD   = 4'd5,
        S_FCS       = 4'd6,
        S_DROP      = 4'd7,
        S_DONE      = 4'd8
    } parser_state_t;

    // ── CRC-32 single-byte update (reflected algorithm) ─────────────────────
    function automatic logic [31:0] crc32_byte(
        input logic [31:0] crc_in,
        input logic  [7:0] data_in
    );
        logic [31:0] c;
        c = crc_in ^ {24'h0, data_in};
        for (int i = 0; i < 8; i++) begin
            if (c[0])
                c = (c >> 1) ^ CRC32_POLY;
            else
                c = c >> 1;
        end
        return c;
    endfunction

    // ── CRC-32 multi-byte update ────────────────────────────────────────────
    function automatic logic [31:0] crc32_update(
        input logic [31:0] crc_in,
        input logic  [7:0] data [],
        input int          num_bytes
    );
        logic [31:0] c;
        c = crc_in;
        for (int i = 0; i < num_bytes; i++)
            c = crc32_byte(c, data[i]);
        return c;
    endfunction

endpackage
