// ============================================================================
// udp_parser_top.sv — Pipelined cut-through Ethernet/IP/UDP parser
//
// Top-level integration:  AXI-Stream in → parse pipeline → AXI-Stream out
//                         AXI4-Lite config / stats
//
// The main FSM tracks byte position within the frame and dispatches
// byte-level data to eth_parser, ipv4_parser, udp_parser, then streams
// payload through payload_forwarder.  CRC-32 is accumulated across the
// entire frame and checked when TLAST arrives.
//
// Cut-through: payload forwarding begins as soon as the UDP header is
// parsed, well before the full frame (and FCS) is received.  If FCS
// fails, TUSER[0] is asserted on the output TLAST beat.
// ============================================================================
module udp_parser_top
    import udp_parser_pkg::*;
#(
    parameter int DATA_WIDTH    = 32,           // 32 or 64
    parameter bit HAS_PREAMBLE  = 1'b0,         // 1 = input includes preamble/SFD
    parameter bit CHECK_FCS     = 1'b1          // 1 = verify CRC-32 at end of frame
)(
    input  logic        clk,
    input  logic        rst_n,

    // ═══════════════════════════════════════════════════════════════════
    //  AXI-Stream input  (raw Ethernet frames)
    // ═══════════════════════════════════════════════════════════════════
    input  logic [DATA_WIDTH-1:0]       s_axis_tdata,
    input  logic [DATA_WIDTH/8-1:0]     s_axis_tkeep,
    input  logic                        s_axis_tvalid,
    output logic                        s_axis_tready,
    input  logic                        s_axis_tlast,

    // ═══════════════════════════════════════════════════════════════════
    //  AXI-Stream output (extracted UDP payloads)
    // ═══════════════════════════════════════════════════════════════════
    output logic [DATA_WIDTH-1:0]       m_axis_tdata,
    output logic [DATA_WIDTH/8-1:0]     m_axis_tkeep,
    output logic                        m_axis_tvalid,
    input  logic                        m_axis_tready,
    output logic                        m_axis_tlast,
    output logic                        m_axis_tuser,       // 1 = error (bad FCS after cut-through)

    // ═══════════════════════════════════════════════════════════════════
    //  AXI4-Lite configuration / statistics
    // ═══════════════════════════════════════════════════════════════════
    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready
);

    // ────────────────────────────────────────────────────────────────────
    //  Local constants
    // ────────────────────────────────────────────────────────────────────
    localparam int BW     = DATA_WIDTH / 8;
    localparam int BW_LOG = $clog2(BW);

    // ────────────────────────────────────────────────────────────────────
    //  Wires: configuration
    // ────────────────────────────────────────────────────────────────────
    logic [47:0] cfg_mac_addr;
    logic        cfg_enable;
    logic        cfg_promiscuous;
    logic        cfg_clear_stats;

    // ────────────────────────────────────────────────────────────────────
    //  Wires: statistics
    // ────────────────────────────────────────────────────────────────────
    logic [31:0] stat_pkt_cnt;
    logic [31:0] stat_err_cnt;
    logic [63:0] stat_byte_cnt;
    logic [31:0] stat_udp_cnt;

    // ────────────────────────────────────────────────────────────────────
    //  Wires: timestamp
    // ────────────────────────────────────────────────────────────────────
    logic [63:0] ts_free_counter;
    logic [63:0] ts_rx_timestamp;
    logic [63:0] ts_tx_timestamp;
    logic [31:0] ts_latency;
    logic        ts_capture_rx;
    logic        ts_capture_tx;

    // ────────────────────────────────────────────────────────────────────
    //  Wires: stats events
    // ────────────────────────────────────────────────────────────────────
    logic        ev_pkt_received;
    logic        ev_pkt_error;
    logic        ev_pkt_udp_valid;
    logic [15:0] ev_byte_count;
    logic        ev_byte_count_valid;

    // ────────────────────────────────────────────────────────────────────
    //  Main parser FSM
    // ────────────────────────────────────────────────────────────────────
    parser_state_t state;

    logic [15:0]    byte_cnt;               // absolute byte position in frame
    logic [15:0]    frame_byte_total;       // total frame bytes seen
    logic           sof_pulse;              // start-of-frame (one cycle)
    logic           input_handshake;

    // Byte arrays for sub-modules
    logic [7:0]             in_bytes [BW];
    logic [BW-1:0]          in_valid;

    // ── Eth parser ──────────────────────────────────────────────────────
    logic [47:0]  eth_dest_mac;
    logic [47:0]  eth_src_mac;
    logic [15:0]  eth_ethertype;
    logic         eth_hdr_done;
    logic         eth_hdr_valid;
    logic         eth_bad_ethertype;
    logic         eth_bad_mac;

    // ── IP parser ───────────────────────────────────────────────────────
    logic [3:0]   ip_version;
    logic [3:0]   ip_ihl;
    logic [15:0]  ip_total_length;
    logic [7:0]   ip_protocol;
    logic [31:0]  ip_src_addr;
    logic [31:0]  ip_dst_addr;
    logic [15:0]  ip_header_checksum;
    logic         ip_hdr_done;
    logic         ip_hdr_valid;
    logic         ip_bad_version;
    logic         ip_bad_checksum;
    logic         ip_non_udp;
    logic [15:0]  ip_payload_length;
    logic [15:0]  ip_hdr_byte_len;

    // ── UDP parser ──────────────────────────────────────────────────────
    logic [15:0]  udp_src_port;
    logic [15:0]  udp_dst_port;
    logic [15:0]  udp_length;
    logic [15:0]  udp_checksum;
    logic         udp_hdr_done;
    logic [15:0]  udp_payload_length;
    logic [15:0]  udp_payload_start_offset;

    // ── Payload forwarder ───────────────────────────────────────────────
    logic         fwd_start;
    logic [BW_LOG-1:0] fwd_start_offset;
    logic         fwd_din_ready;
    logic         fwd_pkt_error;
    logic         fwd_active;           // payload forwarding in progress
    logic         fwd_complete;         // payload forwarding finished
    logic         fcs_verified;         // FCS check complete, forwarder can output last beat

    // ── Replay mechanism (recover beat consumed during S_UDP_HDR) ────────
    logic [DATA_WIDTH-1:0] prev_beat_data;
    logic [BW-1:0]         prev_beat_keep;
    logic                  prev_beat_last;
    logic                  replay_pending;

    // ── CRC-32 ──────────────────────────────────────────────────────────
    logic [31:0]  crc_reg;
    logic [31:0]  crc_next;

    // ── Error tracking ──────────────────────────────────────────────────
    error_flags_t err_flags;
    logic         any_hdr_error;        // latched: header check failed → drop
    logic         payload_started;      // latched: we began forwarding payload
    logic         first_tx_captured;    // latched: tx timestamp captured

    // ── Preamble ────────────────────────────────────────────────────────
    logic         preamble_found;

    // ────────────────────────────────────────────────────────────────────
    //  Input handshake
    // ────────────────────────────────────────────────────────────────────
    assign input_handshake = s_axis_tvalid & s_axis_tready;

    // Start-of-frame: combinational so sub-parsers reset BEFORE the first
    // data handshake (tready=0 in S_IDLE prevents data consumption this cycle).
    // On the next cycle (S_ETH_HDR), tready=1 and sof=0, so the first
    // handshake is correctly processed by all sub-parsers.
    assign sof_pulse = (state == S_IDLE) && s_axis_tvalid && cfg_enable;

    // ── Backpressure ────────────────────────────────────────────────────
    // In S_PAYLOAD: accept input if either
    //   (a) payload forwarder can take data, OR
    //   (b) payload forwarder is done and we are consuming trailing FCS bytes
    always_comb begin
        case (state)
            S_IDLE:     s_axis_tready = 1'b0;       // hold off data until FSM ready
            S_UDP_HDR:  s_axis_tready = udp_hdr_done ? 1'b0 : 1'b1; // stall when hdr done
            S_PAYLOAD:  s_axis_tready = replay_pending ? 1'b0 : (fwd_din_ready | ~fwd_active);
            S_FCS:      s_axis_tready = 1'b1;      // always accept FCS bytes
            S_DROP:     s_axis_tready = 1'b1;       // drain discarded frame
            S_DONE:     s_axis_tready = 1'b0;       // stall one cycle
            default:    s_axis_tready = 1'b1;       // header parsing — no stall
        endcase
    end

    // ────────────────────────────────────────────────────────────────────
    //  Byte extraction from input data word
    // ────────────────────────────────────────────────────────────────────
    always_comb begin
        for (int i = 0; i < BW; i++) begin
            in_bytes[i] = s_axis_tdata[i*8 +: 8];
            in_valid[i] = input_handshake & s_axis_tkeep[i];
        end
    end

    // ────────────────────────────────────────────────────────────────────
    //  CRC-32 computation (combinational chain per beat)
    // ────────────────────────────────────────────────────────────────────
    always_comb begin
        crc_next = crc_reg;
        if (input_handshake &&
            state != S_PREAMBLE && state != S_IDLE && state != S_DONE) begin
            for (int i = 0; i < BW; i++) begin
                if (s_axis_tkeep[i])
                    crc_next = crc32_byte(crc_next, in_bytes[i]);
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            crc_reg <= CRC32_INIT;
        else if (sof_pulse)
            crc_reg <= CRC32_INIT;
        else if (input_handshake &&
                 state != S_PREAMBLE && state != S_IDLE && state != S_DONE)
            crc_reg <= crc_next;
    end

    // ────────────────────────────────────────────────────────────────────
    //  Previous-beat register — captures every handshaked data word so the
    //  beat containing the first payload bytes can be replayed to the
    //  payload forwarder after the S_UDP_HDR → S_PAYLOAD transition.
    // ────────────────────────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_beat_data <= '0;
            prev_beat_keep <= '0;
            prev_beat_last <= 1'b0;
        end else if (input_handshake) begin
            prev_beat_data <= s_axis_tdata;
            prev_beat_keep <= s_axis_tkeep;
            prev_beat_last <= s_axis_tlast;
        end
    end

    // ────────────────────────────────────────────────────────────────────
    //  Count valid bytes in current beat (combinational)
    // ────────────────────────────────────────────────────────────────────
    logic [3:0] beat_valid_cnt;
    always_comb begin
        beat_valid_cnt = '0;
        for (int i = 0; i < BW; i++)
            if (s_axis_tkeep[i]) beat_valid_cnt = beat_valid_cnt + 1;
    end

    // ────────────────────────────────────────────────────────────────────
    //  Main FSM
    // ────────────────────────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            byte_cnt           <= '0;
            frame_byte_total   <= '0;
            preamble_found     <= 1'b0;
            fwd_start          <= 1'b0;
            fwd_start_offset   <= '0;
            fwd_pkt_error      <= 1'b0;
            fwd_active         <= 1'b0;
            replay_pending     <= 1'b0;
            fcs_verified       <= 1'b0;
            err_flags          <= '0;
            any_hdr_error      <= 1'b0;
            payload_started    <= 1'b0;
            first_tx_captured  <= 1'b0;
            ts_capture_rx      <= 1'b0;
            ts_capture_tx      <= 1'b0;
            ev_pkt_received    <= 1'b0;
            ev_pkt_error       <= 1'b0;
            ev_pkt_udp_valid   <= 1'b0;
            ev_byte_count      <= '0;
            ev_byte_count_valid <= 1'b0;
        end else begin
            // ── Clear single-cycle pulses ───────────────────────────────
            fwd_start           <= 1'b0;
            ts_capture_rx       <= 1'b0;
            ts_capture_tx       <= 1'b0;
            ev_pkt_received     <= 1'b0;
            ev_pkt_error        <= 1'b0;
            ev_pkt_udp_valid    <= 1'b0;
            ev_byte_count_valid <= 1'b0;

            // ── Cross-state error detection ─────────────────────────────
            // Check sub-parser error signals regardless of current state.
            // Once a header error is detected, latch it and transition to
            // S_DROP (only if we haven't already started payload output).
            if (!any_hdr_error && !payload_started) begin
                if (eth_hdr_done && (eth_bad_ethertype || eth_bad_mac)) begin
                    any_hdr_error          <= 1'b1;
                    err_flags.bad_ethertype <= eth_bad_ethertype;
                    err_flags.bad_mac       <= eth_bad_mac;
                    if (state != S_IDLE && state != S_DONE)
                        state <= S_DROP;
                end
                if (ip_hdr_done && (ip_bad_version || ip_bad_checksum || ip_non_udp)) begin
                    any_hdr_error             <= 1'b1;
                    err_flags.bad_ip_version  <= ip_bad_version;
                    err_flags.bad_ip_checksum <= ip_bad_checksum;
                    err_flags.non_udp         <= ip_non_udp;
                    if (state != S_IDLE && state != S_DONE)
                        state <= S_DROP;
                end
            end

            // ── State machine ───────────────────────────────────────────
            case (state)

                // ─────────────────────────────────────────────────────────
                S_IDLE: begin
                    if (s_axis_tvalid && cfg_enable) begin
                        ts_capture_rx    <= 1'b1;
                        byte_cnt         <= '0;
                        frame_byte_total <= '0;
                        err_flags        <= '0;
                        any_hdr_error    <= 1'b0;
                        fwd_pkt_error    <= 1'b0;
                        fwd_active       <= 1'b0;
                        replay_pending   <= 1'b0;
                        fcs_verified     <= 1'b0;
                        payload_started  <= 1'b0;
                        first_tx_captured <= 1'b0;
                        preamble_found   <= 1'b0;

                        if (HAS_PREAMBLE)
                            state <= S_PREAMBLE;
                        else
                            state <= S_ETH_HDR;
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_PREAMBLE: begin
                    if (input_handshake) begin
                        for (int i = 0; i < BW; i++) begin
                            if (s_axis_tkeep[i] && in_bytes[i] == SFD_BYTE) begin
                                preamble_found <= 1'b1;
                                byte_cnt       <= '0;
                                state          <= S_ETH_HDR;
                            end
                        end
                        if (s_axis_tlast && !preamble_found) begin
                            err_flags.runt_frame <= 1'b1;
                            state <= S_DONE;
                        end
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_ETH_HDR: begin
                    if (input_handshake) begin
                        byte_cnt         <= byte_cnt + {12'h0, beat_valid_cnt};
                        frame_byte_total <= frame_byte_total + {12'h0, beat_valid_cnt};

                        if ((byte_cnt + beat_valid_cnt) >= ETH_HDR_BYTES[15:0])
                            state <= S_IP_HDR;

                        if (s_axis_tlast) begin
                            err_flags.runt_frame <= 1'b1;
                            state <= S_DONE;
                        end
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_IP_HDR: begin
                    if (input_handshake) begin
                        byte_cnt         <= byte_cnt + {12'h0, beat_valid_cnt};
                        frame_byte_total <= frame_byte_total + {12'h0, beat_valid_cnt};

                        // Transition once all IP header bytes consumed
                        if (ip_hdr_byte_len != 0 &&
                            (byte_cnt + beat_valid_cnt) >=
                            (ETH_HDR_BYTES[15:0] + ip_hdr_byte_len))
                            state <= S_UDP_HDR;

                        if (s_axis_tlast) begin
                            err_flags.truncated <= 1'b1;
                            state <= S_DONE;
                        end
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_UDP_HDR: begin
                    if (input_handshake) begin
                        byte_cnt         <= byte_cnt + {12'h0, beat_valid_cnt};
                        frame_byte_total <= frame_byte_total + {12'h0, beat_valid_cnt};

                        if (s_axis_tlast) begin
                            err_flags.truncated <= 1'b1;
                            state <= S_DONE;
                        end
                    end

                    // UDP header done → start payload cut-through
                    // Replay mechanism: the beat that contained the last UDP
                    // header byte may also contain the first payload bytes.
                    // That beat was already consumed, so we replay it from
                    // prev_beat_data.  Stall tready (see always_comb) so no
                    // new beat is consumed this cycle.
                    if (udp_hdr_done && !fwd_start && !any_hdr_error) begin
                        fwd_start        <= 1'b1;
                        fwd_start_offset <= udp_payload_start_offset[BW_LOG-1:0];
                        fwd_active       <= 1'b1;
                        payload_started  <= 1'b1;
                        replay_pending   <= (udp_payload_start_offset[BW_LOG-1:0] != '0);
                        state            <= S_PAYLOAD;
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_PAYLOAD: begin
                    // Clear replay once the forwarder has accepted the
                    // replayed previous beat (forwarder din_ready = 1).
                    if (replay_pending && fwd_din_ready)
                        replay_pending <= 1'b0;

                    if (input_handshake) begin
                        byte_cnt         <= byte_cnt + {12'h0, beat_valid_cnt};
                        frame_byte_total <= frame_byte_total + {12'h0, beat_valid_cnt};

                        // Capture first-payload-byte-out timestamp
                        if (!first_tx_captured && m_axis_tvalid) begin
                            ts_capture_tx     <= 1'b1;
                            first_tx_captured <= 1'b1;
                        end

                        if (s_axis_tlast) begin
                            // Frame complete — check FCS
                            if (CHECK_FCS && crc_next != CRC32_RESIDUE) begin
                                err_flags.bad_fcs <= 1'b1;
                                fwd_pkt_error     <= 1'b1;
                            end
                            // Runt check
                            if ((frame_byte_total + beat_valid_cnt) < MIN_FRAME_BYTES[15:0]) begin
                                err_flags.runt_frame <= 1'b1;
                                fwd_pkt_error        <= 1'b1;
                            end
                            fcs_verified <= 1'b1;
                            fwd_active   <= 1'b0;
                            state        <= S_DONE;
                        end
                    end

                    // If payload forwarder finishes but input has more bytes
                    // (FCS tail), keep consuming.  Use fwd_complete (not
                    // !fwd_din_ready) to distinguish completion from
                    // backpressure stall.
                    if (fwd_active && fwd_complete) begin
                        fwd_active <= 1'b0;
                        // Remain in S_PAYLOAD; tready is now unblocked by
                        // the ~fwd_active term
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_FCS: begin
                    // Consume trailing bytes (FCS) after payload forwarding done
                    if (input_handshake) begin
                        frame_byte_total <= frame_byte_total + {12'h0, beat_valid_cnt};
                        if (s_axis_tlast) begin
                            if (CHECK_FCS && crc_next != CRC32_RESIDUE) begin
                                err_flags.bad_fcs <= 1'b1;
                                fwd_pkt_error     <= 1'b1;
                            end
                            state <= S_DONE;
                        end
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_DROP: begin
                    if (input_handshake) begin
                        frame_byte_total <= frame_byte_total + {12'h0, beat_valid_cnt};
                        if (s_axis_tlast)
                            state <= S_DONE;
                    end
                end

                // ─────────────────────────────────────────────────────────
                S_DONE: begin
                    ev_pkt_received     <= 1'b1;
                    ev_byte_count       <= frame_byte_total;
                    ev_byte_count_valid <= 1'b1;
                    ev_pkt_error        <= (err_flags != '0);
                    ev_pkt_udp_valid    <= (err_flags == '0) && payload_started;
                    state               <= S_IDLE;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

    // ────────────────────────────────────────────────────────────────────
    //  Sub-module instances
    // ────────────────────────────────────────────────────────────────────

    eth_parser #(.BYTE_WIDTH(BW)) u_eth_parser (
        .clk            (clk),
        .rst_n          (rst_n),
        .in_bytes       (in_bytes),
        .in_valid       (in_valid),
        .byte_offset    (byte_cnt),
        .sof            (sof_pulse),
        .dest_mac       (eth_dest_mac),
        .src_mac        (eth_src_mac),
        .ethertype      (eth_ethertype),
        .eth_hdr_done   (eth_hdr_done),
        .eth_hdr_valid  (eth_hdr_valid),
        .bad_ethertype  (eth_bad_ethertype),
        .bad_mac        (eth_bad_mac),
        .cfg_mac_addr   (cfg_mac_addr),
        .cfg_promiscuous(cfg_promiscuous)
    );

    ipv4_parser #(.BYTE_WIDTH(BW)) u_ipv4_parser (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_bytes          (in_bytes),
        .in_valid          (in_valid),
        .byte_offset       (byte_cnt),
        .sof               (sof_pulse),
        .ip_version        (ip_version),
        .ip_ihl            (ip_ihl),
        .ip_total_length   (ip_total_length),
        .ip_protocol       (ip_protocol),
        .ip_src_addr       (ip_src_addr),
        .ip_dst_addr       (ip_dst_addr),
        .ip_header_checksum(ip_header_checksum),
        .ip_hdr_done       (ip_hdr_done),
        .ip_hdr_valid      (ip_hdr_valid),
        .bad_ip_version    (ip_bad_version),
        .bad_ip_checksum   (ip_bad_checksum),
        .non_udp           (ip_non_udp),
        .ip_payload_length (ip_payload_length),
        .ip_hdr_byte_len   (ip_hdr_byte_len)
    );

    udp_parser #(.BYTE_WIDTH(BW)) u_udp_parser (
        .clk                  (clk),
        .rst_n                (rst_n),
        .in_bytes             (in_bytes),
        .in_valid             (in_valid),
        .byte_offset          (byte_cnt),
        .sof                  (sof_pulse),
        .ip_hdr_byte_len      (ip_hdr_byte_len),
        .udp_src_port         (udp_src_port),
        .udp_dst_port         (udp_dst_port),
        .udp_length           (udp_length),
        .udp_checksum         (udp_checksum),
        .udp_hdr_done         (udp_hdr_done),
        .payload_length       (udp_payload_length),
        .payload_start_offset (udp_payload_start_offset)
    );

    // ── Forwarder data-path MUX ───────────────────────────────────────
    // During replay, feed the saved previous beat to the forwarder
    // instead of the live AXI-Stream input (which is stalled).
    wire [DATA_WIDTH-1:0] fwd_din_data  = replay_pending ? prev_beat_data : s_axis_tdata;
    wire [BW-1:0]         fwd_din_keep  = replay_pending ? prev_beat_keep : s_axis_tkeep;
    wire                  fwd_din_last  = replay_pending ? prev_beat_last : s_axis_tlast;
    wire                  fwd_din_valid = replay_pending
                                          ? fwd_active
                                          : (input_handshake && (state == S_PAYLOAD) && fwd_active);

    payload_forwarder #(.DATA_WIDTH(DATA_WIDTH)) u_payload_fwd (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (fwd_start),
        .payload_length (udp_payload_length),
        .start_offset   (fwd_start_offset),
        .din_data       (fwd_din_data),
        .din_keep       (fwd_din_keep),
        .din_valid      (fwd_din_valid),
        .din_last       (fwd_din_last),
        .din_ready      (fwd_din_ready),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tkeep   (m_axis_tkeep),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tlast   (m_axis_tlast),
        .m_axis_tuser   (m_axis_tuser),
        .m_axis_tready  (m_axis_tready),
        .pkt_error_flag (fwd_pkt_error),
        .fcs_verified   (fcs_verified),
        .fwd_complete   (fwd_complete)
    );

    timestamp_unit u_timestamp (
        .clk            (clk),
        .rst_n          (rst_n),
        .capture_rx     (ts_capture_rx),
        .capture_tx     (ts_capture_tx),
        .free_counter   (ts_free_counter),
        .rx_timestamp   (ts_rx_timestamp),
        .tx_timestamp   (ts_tx_timestamp),
        .latency_cycles (ts_latency)
    );

    stats_counters u_stats (
        .clk              (clk),
        .rst_n            (rst_n),
        .pkt_received     (ev_pkt_received),
        .pkt_error        (ev_pkt_error),
        .pkt_udp_valid    (ev_pkt_udp_valid),
        .byte_count       (ev_byte_count),
        .byte_count_valid (ev_byte_count_valid),
        .total_pkt_cnt    (stat_pkt_cnt),
        .error_pkt_cnt    (stat_err_cnt),
        .total_byte_cnt   (stat_byte_cnt),
        .udp_pkt_cnt      (stat_udp_cnt),
        .clear_stats      (cfg_clear_stats)
    );

    axi4_lite_regs u_axil_regs (
        .aclk             (clk),
        .aresetn          (rst_n),
        .s_axi_awaddr     (s_axi_awaddr),
        .s_axi_awvalid    (s_axi_awvalid),
        .s_axi_awready    (s_axi_awready),
        .s_axi_wdata      (s_axi_wdata),
        .s_axi_wstrb      (s_axi_wstrb),
        .s_axi_wvalid     (s_axi_wvalid),
        .s_axi_wready     (s_axi_wready),
        .s_axi_bresp      (s_axi_bresp),
        .s_axi_bvalid     (s_axi_bvalid),
        .s_axi_bready     (s_axi_bready),
        .s_axi_araddr     (s_axi_araddr),
        .s_axi_arvalid    (s_axi_arvalid),
        .s_axi_arready    (s_axi_arready),
        .s_axi_rdata      (s_axi_rdata),
        .s_axi_rresp      (s_axi_rresp),
        .s_axi_rvalid     (s_axi_rvalid),
        .s_axi_rready     (s_axi_rready),
        .cfg_mac_addr     (cfg_mac_addr),
        .cfg_enable       (cfg_enable),
        .cfg_promiscuous  (cfg_promiscuous),
        .cfg_clear_stats  (cfg_clear_stats),
        .stat_pkt_cnt     (stat_pkt_cnt),
        .stat_err_cnt     (stat_err_cnt),
        .stat_byte_cnt    (stat_byte_cnt),
        .stat_udp_cnt     (stat_udp_cnt),
        .stat_rx_ts       (ts_rx_timestamp),
        .stat_latency     (ts_latency)
    );

endmodule