// ============================================================================
// timestamp_unit.sv — Free-running cycle counter with capture
// ============================================================================
module timestamp_unit (
    input  logic        clk,
    input  logic        rst_n,

    // Capture triggers
    input  logic        capture_rx,         // latch on first byte of frame
    input  logic        capture_tx,         // latch on first payload byte out

    // Outputs
    output logic [63:0] free_counter,       // current cycle count
    output logic [63:0] rx_timestamp,       // latched receive timestamp
    output logic [63:0] tx_timestamp,       // latched transmit timestamp
    output logic [31:0] latency_cycles      // tx - rx (32-bit is plenty)
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            free_counter <= '0;
        else
            free_counter <= free_counter + 1'b1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_timestamp   <= '0;
            tx_timestamp   <= '0;
            latency_cycles <= '0;
        end else begin
            if (capture_rx)
                rx_timestamp <= free_counter;
            if (capture_tx) begin
                tx_timestamp   <= free_counter;
                latency_cycles <= free_counter[31:0] - rx_timestamp[31:0];
            end
        end
    end

endmodule
