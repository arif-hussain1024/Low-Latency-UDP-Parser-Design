// ============================================================================
// payload_forwarder.sv — AXI-Stream output controller
//
// Receives raw data words from the parser core once the UDP header is
// parsed.  Handles byte-offset realignment so the first output byte is
// the first payload byte, generates TKEEP on the last beat, and
// supports backpressure from the downstream consumer.
//
// The payload starts at an arbitrary byte offset within a DATA_WIDTH
// input word.  This module buffers residual bytes and merges them with
// the next input word to produce aligned output words.
// ============================================================================
module payload_forwarder #(
    parameter int DATA_WIDTH = 32
)(
    input  logic                        clk,
    input  logic                        rst_n,

    // ── Control from parser core ────────────────────────────────────────
    input  logic                        start,              // pulse: begin forwarding
    input  logic [15:0]                 payload_length,     // total payload bytes
    input  logic [$clog2(DATA_WIDTH/8)-1:0] start_offset,  // byte lane of first payload byte

    // ── Data input from parser (raw words, incl. non-payload prefix) ────
    input  logic [DATA_WIDTH-1:0]       din_data,
    input  logic [DATA_WIDTH/8-1:0]     din_keep,
    input  logic                        din_valid,
    input  logic                        din_last,           // end of input frame
    output logic                        din_ready,          // backpressure to parser

    // ── AXI-Stream output (payload only) ────────────────────────────────
    output logic [DATA_WIDTH-1:0]       m_axis_tdata,
    output logic [DATA_WIDTH/8-1:0]     m_axis_tkeep,
    output logic                        m_axis_tvalid,
    output logic                        m_axis_tlast,
    output logic                        m_axis_tuser,       // error flag
    input  logic                        m_axis_tready,

    // ── Error flag (set at end of frame by parser core) ─────────────────
    input  logic                        pkt_error_flag,

    // ── FCS verification (cut-through: hold last beat until checked) ───
    input  logic                        fcs_verified,       // FCS check complete

    // ── Status ──────────────────────────────────────────────────────────
    output logic                        fwd_complete        // all payload bytes forwarded
);

    localparam int BW = DATA_WIDTH / 8;
    localparam int BW_LOG = $clog2(BW);

    // ── State ───────────────────────────────────────────────────────────
    typedef enum logic [2:0] {
        FWD_IDLE,
        FWD_FIRST,      // first word (may be partial)
        FWD_STREAM,     // steady-state forwarding
        FWD_RESIDUAL,   // flush leftover bytes after input ends
        FWD_WAIT_FCS,   // hold last beat until FCS verified
        FWD_DONE
    } fwd_state_t;

    fwd_state_t state;

    logic [DATA_WIDTH-1:0]  residual_data;
    logic [BW_LOG:0]        residual_cnt;       // 0..BW
    logic [15:0]            bytes_remaining;
    logic [BW_LOG-1:0]      offset_r;
    logic                   input_ended;

    // ── Output assembly ─────────────────────────────────────────────────
    logic [DATA_WIDTH*2-1:0] merge_buf;
    logic [BW*2-1:0]         merge_keep;
    logic [DATA_WIDTH-1:0]   out_data;
    logic [BW-1:0]           out_keep;
    logic                    out_last;
    logic                    out_valid;

    // Backpressure: accept input when output can take data or we're not outputting
    assign din_ready = (state != FWD_IDLE && state != FWD_DONE && state != FWD_RESIDUAL && state != FWD_WAIT_FCS)
                       ? (m_axis_tready || !m_axis_tvalid)
                       : 1'b0;

    // Completion flag: all payload bytes have been output (or are in final beat)
    assign fwd_complete = (state == FWD_DONE) ||
                          (state == FWD_WAIT_FCS) ||
                          (state == FWD_IDLE && !start);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= FWD_IDLE;
            residual_data   <= '0;
            residual_cnt    <= '0;
            bytes_remaining <= '0;
            offset_r        <= '0;
            input_ended     <= 1'b0;
            m_axis_tdata    <= '0;
            m_axis_tkeep    <= '0;
            m_axis_tvalid   <= 1'b0;
            m_axis_tlast    <= 1'b0;
            m_axis_tuser    <= 1'b0;
        end else begin

            // Clear output handshake when accepted
            if (m_axis_tvalid && m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
                m_axis_tuser  <= 1'b0;
            end

            case (state)

                FWD_IDLE: begin
                    if (start) begin
                        state           <= FWD_FIRST;
                        bytes_remaining <= payload_length;
                        offset_r        <= start_offset;
                        residual_cnt    <= '0;
                        residual_data   <= '0;
                        input_ended     <= 1'b0;
                    end
                end

                FWD_FIRST: begin
                    // Wait for the first data word containing payload bytes
                    if (din_valid && din_ready) begin
                        // Extract payload bytes from this word starting at offset_r
                        automatic int payload_in_word = BW - offset_r;
                        automatic int actual_bytes;

                        if (payload_in_word > bytes_remaining)
                            actual_bytes = bytes_remaining;
                        else
                            actual_bytes = payload_in_word;

                        // Shift data so payload starts at byte 0
                        for (int b = 0; b < BW; b++) begin
                            if (b < actual_bytes) begin
                                m_axis_tdata[b*8 +: 8] <= din_data[(offset_r + b)*8 +: 8];
                                m_axis_tkeep[b]         <= 1'b1;
                            end else begin
                                m_axis_tdata[b*8 +: 8] <= '0;
                                m_axis_tkeep[b]         <= 1'b0;
                            end
                        end

                        input_ended     <= din_last;

                        if (bytes_remaining <= actual_bytes[15:0]) begin
                            // Case 1: Entire payload fits in this first word
                            bytes_remaining <= '0;
                            // Data already in m_axis_tdata/tkeep — hold for FCS
                            state           <= FWD_WAIT_FCS;
                        end else if (actual_bytes == BW) begin
                            // Case 2: Full output word — all bytes emitted
                            bytes_remaining <= bytes_remaining - actual_bytes[15:0];
                            m_axis_tvalid   <= 1'b1;
                            residual_cnt    <= '0;
                            state           <= FWD_STREAM;
                        end else begin
                            // Case 3: Partial word — store as residual
                            // Don't decrement bytes_remaining (bytes not yet output)
                            for (int b = 0; b < BW; b++) begin
                                if (b < actual_bytes)
                                    residual_data[b*8 +: 8] <= din_data[(offset_r + b)*8 +: 8];
                                else
                                    residual_data[b*8 +: 8] <= '0;
                            end
                            residual_cnt  <= actual_bytes[BW_LOG:0];
                            m_axis_tvalid <= 1'b0;
                            state         <= FWD_STREAM;
                        end
                    end
                end

                FWD_STREAM: begin
                    if (din_valid && din_ready) begin
                        // Merge residual bytes with new input
                        automatic int total_avail = residual_cnt + BW;
                        automatic int out_bytes;
                        automatic int new_residual;

                        // How many payload bytes to emit this cycle
                        if (total_avail >= BW) begin
                            if (bytes_remaining <= BW)
                                out_bytes = bytes_remaining;
                            else
                                out_bytes = BW;
                        end else begin
                            if (bytes_remaining <= total_avail)
                                out_bytes = bytes_remaining;
                            else
                                out_bytes = total_avail;
                        end

                        // Assemble output from residual + new data
                        // Build a 2*BW byte merge buffer: [residual | new data]
                        for (int b = 0; b < BW; b++) begin
                            if (b < residual_cnt)
                                m_axis_tdata[b*8 +: 8] <= residual_data[b*8 +: 8];
                            else
                                m_axis_tdata[b*8 +: 8] <= din_data[(b - residual_cnt)*8 +: 8];
                        end

                        // Generate TKEEP
                        for (int b = 0; b < BW; b++)
                            m_axis_tkeep[b] <= (b < out_bytes) ? 1'b1 : 1'b0;

                        // Calculate new residual (bytes from this input word not yet output)
                        new_residual = (residual_cnt + BW) - BW; // = residual_cnt
                        // Actually: total bytes available = residual_cnt + BW (new input bytes)
                        // Bytes output = min(BW, bytes_remaining)
                        // New residual = total_avail - out_bytes = residual_cnt + BW - out_bytes

                        // Store residual bytes for next cycle
                        for (int b = 0; b < BW; b++) begin
                            automatic int src_idx = b + (BW - residual_cnt);
                            if (src_idx < BW)
                                residual_data[b*8 +: 8] <= din_data[src_idx*8 +: 8];
                            else
                                residual_data[b*8 +: 8] <= '0;
                        end
                        residual_cnt <= (residual_cnt + BW - out_bytes[BW_LOG:0]);

                        bytes_remaining <= bytes_remaining - out_bytes[15:0];
                        input_ended     <= din_last;

                        if (bytes_remaining <= out_bytes[15:0]) begin
                            // Last payload beat — data already in output regs,
                            // hold tvalid=0 until FCS check is complete
                            state <= FWD_WAIT_FCS;
                        end else begin
                            m_axis_tvalid <= 1'b1;
                        end

                    end else if (input_ended && residual_cnt > 0 && bytes_remaining > 0) begin
                        // Input stream ended but we have residual bytes
                        state <= FWD_RESIDUAL;
                    end
                end

                FWD_RESIDUAL: begin
                    // Flush remaining residual bytes
                    if (!m_axis_tvalid || m_axis_tready) begin
                        automatic int out_bytes;
                        if (bytes_remaining <= residual_cnt)
                            out_bytes = bytes_remaining;
                        else
                            out_bytes = residual_cnt;

                        m_axis_tdata  <= residual_data;
                        for (int b = 0; b < BW; b++)
                            m_axis_tkeep[b] <= (b < out_bytes) ? 1'b1 : 1'b0;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b1;
                        m_axis_tuser  <= pkt_error_flag;
                        state         <= FWD_DONE;
                    end
                end

                FWD_WAIT_FCS: begin
                    // Hold last-beat data in output registers until FCS verified.
                    // m_axis_tdata/tkeep already contain the final payload beat.
                    if (fcs_verified) begin
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b1;
                        m_axis_tuser  <= pkt_error_flag;
                        state         <= FWD_DONE;
                    end
                end

                FWD_DONE: begin
                    if (!m_axis_tvalid || m_axis_tready)
                        state <= FWD_IDLE;
                end

            endcase
        end
    end

endmodule