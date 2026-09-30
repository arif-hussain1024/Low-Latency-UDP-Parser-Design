// ============================================================================
// pkt_checker.sv — Payload verification for testbench
//
// Captures AXI-Stream output from the parser, compares against
// expected payload byte arrays, and reports pass/fail.
// ============================================================================
module pkt_checker
    import udp_parser_pkg::*;
#(
    parameter int DATA_WIDTH = 32
)(
    input  logic                        clk,
    input  logic                        rst_n,

    // AXI-Stream input (from DUT output)
    input  logic [DATA_WIDTH-1:0]       s_axis_tdata,
    input  logic [DATA_WIDTH/8-1:0]     s_axis_tkeep,
    input  logic                        s_axis_tvalid,
    output logic                        s_axis_tready,
    input  logic                        s_axis_tlast,
    input  logic                        s_axis_tuser,

    // Control
    input  logic                        expect_packet,      // a valid packet is expected
    input  logic                        expect_error_flag,  // TUSER should be set
    input  logic                        expect_no_output,   // no output expected (dropped pkt)
    output logic                        check_done,
    output logic                        check_pass
);

    localparam int BW = DATA_WIDTH / 8;

    // ── Expected payload storage ────────────────────────────────────────
    logic [7:0] expected_payload [0:2047];
    int         expected_len;
    int         rx_byte_idx;

    // ── Received payload ────────────────────────────────────────────────
    logic [7:0] received_payload [0:2047];
    int         received_len;

    // Always ready to accept data
    assign s_axis_tready = 1'b1;

    // ── Capture logic ───────────────────────────────────────────────────
    logic receiving;
    logic got_error_flag;
    logic got_any_output;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte_idx    <= 0;
            received_len   <= 0;
            check_done     <= 1'b0;
            check_pass     <= 1'b0;
            receiving      <= 1'b0;
            got_error_flag <= 1'b0;
            got_any_output <= 1'b0;
        end else begin
            if (expect_packet || expect_no_output) begin
                // Reset for new expected packet
                rx_byte_idx    <= 0;
                received_len   <= 0;
                check_done     <= 1'b0;
                check_pass     <= 1'b0;
                receiving      <= 1'b1;
                got_error_flag <= 1'b0;
                got_any_output <= 1'b0;
            end

            if (receiving && s_axis_tvalid && s_axis_tready) begin
                got_any_output <= 1'b1;

                for (int i = 0; i < BW; i++) begin
                    if (s_axis_tkeep[i]) begin
                        received_payload[rx_byte_idx] <= s_axis_tdata[i*8 +: 8];
                        rx_byte_idx <= rx_byte_idx + 1;
                    end
                end

                if (s_axis_tuser)
                    got_error_flag <= 1'b1;

                if (s_axis_tlast) begin
                    received_len <= rx_byte_idx;
                    receiving    <= 1'b0;
                    check_done   <= 1'b1;

                    // Verify
                    if (expect_error_flag) begin
                        check_pass <= s_axis_tuser;  // expect error flag set
                    end else begin
                        // Compare payload
                        automatic logic pass = 1'b1;
                        if (rx_byte_idx != expected_len) begin
                            pass = 1'b0;
                            $display("[CHECKER] FAIL: length mismatch. Expected %0d, got %0d",
                                     expected_len, rx_byte_idx);
                        end else begin
                            for (int i = 0; i < expected_len; i++) begin
                                if (received_payload[i] !== expected_payload[i]) begin
                                    pass = 1'b0;
                                    $display("[CHECKER] FAIL: byte %0d mismatch. Expected 0x%02h, got 0x%02h",
                                             i, expected_payload[i], received_payload[i]);
                                end
                            end
                        end
                        if (s_axis_tuser) begin
                            pass = 1'b0;
                            $display("[CHECKER] FAIL: unexpected error flag on valid packet");
                        end
                        check_pass <= pass;
                    end
                end
            end
        end
    end

    // ── Load expected payload (called from testbench) ───────────────────
    task automatic set_expected(
        input logic [7:0] payload [],
        input int         len
    );
        for (int i = 0; i < len; i++)
            expected_payload[i] = payload[i];
        expected_len = len;
    endtask

endmodule
