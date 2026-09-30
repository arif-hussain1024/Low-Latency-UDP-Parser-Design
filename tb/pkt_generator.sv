// ============================================================================
// pkt_generator.sv — Ethernet/IP/UDP packet generator for testbench
//
// Builds complete frames in a byte-array, computes correct IP checksum
// and Ethernet FCS (CRC-32), then drives them onto an AXI-Stream bus.
// Supports injection of errors: bad FCS, bad IP checksum, non-UDP
// protocol, truncated frames, runt frames.
// ============================================================================
module pkt_generator
    import udp_parser_pkg::*;
#(
    parameter int DATA_WIDTH = 32
)(
    input  logic                        clk,
    input  logic                        rst_n,

    // AXI-Stream output
    output logic [DATA_WIDTH-1:0]       m_axis_tdata,
    output logic [DATA_WIDTH/8-1:0]     m_axis_tkeep,
    output logic                        m_axis_tvalid,
    input  logic                        m_axis_tready,
    output logic                        m_axis_tlast,

    // Control
    input  logic                        send_trigger,
    output logic                        busy,
    output logic                        done_pulse
);

    localparam int BW = DATA_WIDTH / 8;

    // ── Packet storage ──────────────────────────────────────────────────
    logic [7:0] pkt_data [0:2047];
    int         pkt_len;
    int         send_idx;

    // ── Build helpers ───────────────────────────────────────────────────

    // IP header ones-complement checksum
    function automatic logic [15:0] compute_ip_checksum(
        input logic [7:0] hdr [],
        input int hdr_len
    );
        logic [31:0] sum;
        sum = 0;
        for (int i = 0; i < hdr_len; i += 2) begin
            sum = sum + {hdr[i], hdr[i+1]};
        end
        // Fold carries
        while (sum[31:16] != 0)
            sum = sum[15:0] + sum[31:16];
        return ~sum[15:0];
    endfunction

    // CRC-32 over a byte array (for FCS computation)
    function automatic logic [31:0] compute_crc32(
        input logic [7:0] data [],
        input int len
    );
        logic [31:0] crc;
        crc = 32'hFFFF_FFFF;
        for (int i = 0; i < len; i++)
            crc = crc32_byte(crc, data[i]);
        return crc ^ 32'hFFFF_FFFF;
    endfunction

    // ── Build a complete Ethernet/IP/UDP frame ──────────────────────────
    // Returns total frame length (incl. FCS)
    task automatic build_packet(
        input  logic [47:0] dest_mac,
        input  logic [47:0] src_mac,
        input  logic [7:0]  payload [],
        input  int          payload_len,
        input  logic [15:0] udp_src_port,
        input  logic [15:0] udp_dst_port,
        input  logic [31:0] ip_src,
        input  logic [31:0] ip_dst,
        // Error injection
        input  logic        inject_bad_fcs,
        input  logic        inject_bad_ip_cksum,
        input  logic        inject_non_udp,
        input  logic        inject_truncated,
        input  logic        inject_runt,
        output int          total_len
    );
        int idx;
        logic [15:0] ip_total_len;
        logic [15:0] udp_len;
        logic [15:0] ip_cksum;
        logic [7:0]  ip_hdr [20];
        logic [31:0] fcs;
        int          frame_data_len;

        idx = 0;

        // ── Ethernet header (14 bytes) ──────────────────────────────────
        // Dest MAC
        pkt_data[idx++] = dest_mac[47:40];
        pkt_data[idx++] = dest_mac[39:32];
        pkt_data[idx++] = dest_mac[31:24];
        pkt_data[idx++] = dest_mac[23:16];
        pkt_data[idx++] = dest_mac[15: 8];
        pkt_data[idx++] = dest_mac[ 7: 0];
        // Src MAC
        pkt_data[idx++] = src_mac[47:40];
        pkt_data[idx++] = src_mac[39:32];
        pkt_data[idx++] = src_mac[31:24];
        pkt_data[idx++] = src_mac[23:16];
        pkt_data[idx++] = src_mac[15: 8];
        pkt_data[idx++] = src_mac[ 7: 0];
        // EtherType: IPv4
        pkt_data[idx++] = 8'h08;
        pkt_data[idx++] = 8'h00;

        // ── IPv4 header (20 bytes, IHL=5) ───────────────────────────────
        udp_len      = 16'd8 + payload_len[15:0];
        ip_total_len = 16'd20 + udp_len;

        ip_hdr[0]  = 8'h45;                    // version=4, IHL=5
        ip_hdr[1]  = 8'h00;                    // DSCP/ECN
        ip_hdr[2]  = ip_total_len[15:8];
        ip_hdr[3]  = ip_total_len[7:0];
        ip_hdr[4]  = 8'h00; ip_hdr[5] = 8'h01; // Identification
        ip_hdr[6]  = 8'h00; ip_hdr[7] = 8'h00; // Flags/FragOffset
        ip_hdr[8]  = 8'h40;                    // TTL=64
        ip_hdr[9]  = inject_non_udp ? 8'h06 : 8'h11; // Protocol: TCP(06) or UDP(11)
        ip_hdr[10] = 8'h00; ip_hdr[11] = 8'h00; // Checksum placeholder
        ip_hdr[12] = ip_src[31:24];
        ip_hdr[13] = ip_src[23:16];
        ip_hdr[14] = ip_src[15: 8];
        ip_hdr[15] = ip_src[ 7: 0];
        ip_hdr[16] = ip_dst[31:24];
        ip_hdr[17] = ip_dst[23:16];
        ip_hdr[18] = ip_dst[15: 8];
        ip_hdr[19] = ip_dst[ 7: 0];

        // Compute IP checksum
        ip_cksum = compute_ip_checksum(ip_hdr, 20);
        if (inject_bad_ip_cksum)
            ip_cksum = ip_cksum ^ 16'hBEEF;    // corrupt it
        ip_hdr[10] = ip_cksum[15:8];
        ip_hdr[11] = ip_cksum[7:0];

        for (int i = 0; i < 20; i++)
            pkt_data[idx++] = ip_hdr[i];

        // ── UDP header (8 bytes) ────────────────────────────────────────
        pkt_data[idx++] = udp_src_port[15:8];
        pkt_data[idx++] = udp_src_port[ 7:0];
        pkt_data[idx++] = udp_dst_port[15:8];
        pkt_data[idx++] = udp_dst_port[ 7:0];
        pkt_data[idx++] = udp_len[15:8];
        pkt_data[idx++] = udp_len[ 7:0];
        pkt_data[idx++] = 8'h00;               // UDP checksum = 0 (disabled)
        pkt_data[idx++] = 8'h00;

        // ── Payload ─────────────────────────────────────────────────────
        for (int i = 0; i < payload_len; i++)
            pkt_data[idx++] = payload[i];

        // ── Padding to minimum frame size (60 bytes before FCS) ─────────
        if (!inject_runt) begin
            while (idx < 60)
                pkt_data[idx++] = 8'h00;
        end

        frame_data_len = idx;

        // ── FCS (CRC-32) ────────────────────────────────────────────────
        begin
            logic [7:0] frame_bytes [];
            frame_bytes = new[frame_data_len];
            for (int i = 0; i < frame_data_len; i++)
                frame_bytes[i] = pkt_data[i];
            fcs = compute_crc32(frame_bytes, frame_data_len);
        end

        if (inject_bad_fcs)
            fcs = fcs ^ 32'hDEAD_BEEF;

        // FCS is transmitted LSByte first
        pkt_data[idx++] = fcs[ 7: 0];
        pkt_data[idx++] = fcs[15: 8];
        pkt_data[idx++] = fcs[23:16];
        pkt_data[idx++] = fcs[31:24];

        // ── Truncation: chop the frame ──────────────────────────────────
        if (inject_truncated)
            idx = (idx > 30) ? 30 : idx;    // cut to 30 bytes

        total_len = idx;
    endtask

    // ── Transmit FSM ────────────────────────────────────────────────────
    typedef enum logic [1:0] { TX_IDLE, TX_SEND, TX_DONE } tx_state_t;
    tx_state_t tx_state;

    assign busy = (tx_state != TX_IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state      <= TX_IDLE;
            m_axis_tdata  <= '0;
            m_axis_tkeep  <= '0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            send_idx      <= 0;
            done_pulse    <= 1'b0;
        end else begin
            done_pulse <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    m_axis_tvalid <= 1'b0;
                    if (send_trigger) begin
                        send_idx <= 0;
                        tx_state <= TX_SEND;
                    end
                end

                TX_SEND: begin
                    if (!m_axis_tvalid || m_axis_tready) begin
                        // Load next data word
                        m_axis_tdata  <= '0;
                        m_axis_tkeep  <= '0;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b0;

                        for (int i = 0; i < BW; i++) begin
                            if ((send_idx + i) < pkt_len) begin
                                m_axis_tdata[i*8 +: 8] <= pkt_data[send_idx + i];
                                m_axis_tkeep[i]         <= 1'b1;
                            end
                        end

                        // Check if this is the last beat
                        if ((send_idx + BW) >= pkt_len) begin
                            m_axis_tlast <= 1'b1;
                            tx_state     <= TX_DONE;
                        end

                        send_idx <= send_idx + BW;
                    end
                end

                TX_DONE: begin
                    if (m_axis_tready || !m_axis_tvalid) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                        done_pulse    <= 1'b1;
                        tx_state      <= TX_IDLE;
                    end
                end

            endcase
        end
    end

endmodule
