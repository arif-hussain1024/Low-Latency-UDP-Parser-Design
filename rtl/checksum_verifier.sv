// ============================================================================
// checksum_verifier.sv — IPv4 header ones-complement checksum
//
// Accumulates 16-bit words of the IP header.  When all words have been
// fed in, the result should fold to 16'hFFFF for a correct header.
// ============================================================================
module checksum_verifier (
    input  logic        clk,
    input  logic        rst_n,

    // Control
    input  logic        clear,          // start new accumulation
    input  logic        add_valid,      // a 16-bit word is presented
    input  logic [15:0] add_data,       // 16-bit word (network byte order)

    // Result (valid one cycle after last add_valid)
    output logic        done,           // verification complete
    output logic        checksum_ok     // 1 = header valid
);

    logic [31:0] accum;     // wide enough for carries
    logic [16:0] fold;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accum <= '0;
            done  <= 1'b0;
        end else if (clear) begin
            accum <= '0;
            done  <= 1'b0;
        end else if (add_valid) begin
            accum <= accum + {16'h0, add_data};
            done  <= 1'b0;
        end else if (!done && accum != '0) begin
            // Fold carries until stable (takes at most 2 cycles)
            if (accum[31:16] == '0) begin
                done <= 1'b1;
            end else begin
                accum <= {16'h0, accum[15:0]} + {16'h0, accum[31:16]};
            end
        end
    end

    // After folding, a correct IP header produces 0xFFFF
    assign fold       = accum[15:0] + accum[31:16];
    assign checksum_ok = done && (accum[15:0] == 16'hFFFF);

endmodule
