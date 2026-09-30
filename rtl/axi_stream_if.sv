// ============================================================================
// axi_stream_if.sv — Parameterized AXI-Stream interface
// ============================================================================
interface axi_stream_if #(
    parameter int DATA_WIDTH = 32,
    parameter int USER_WIDTH = 1
)(
    input logic clk,
    input logic rst_n
);

    localparam int BYTE_WIDTH = DATA_WIDTH / 8;

    logic [DATA_WIDTH-1:0]  tdata;
    logic                   tvalid;
    logic                   tready;
    logic                   tlast;
    logic [BYTE_WIDTH-1:0]  tkeep;
    logic [USER_WIDTH-1:0]  tuser;    // bit 0 = error flag on TLAST

    // Source (master) drives data
    modport master (
        output tdata, tvalid, tlast, tkeep, tuser,
        input  tready
    );

    // Sink (slave) receives data
    modport slave (
        input  tdata, tvalid, tlast, tkeep, tuser,
        output tready
    );

    // Monitor (passive observation)
    modport monitor (
        input tdata, tvalid, tready, tlast, tkeep, tuser
    );

    // Handshake fires when both valid and ready
    logic handshake;
    assign handshake = tvalid & tready;

endinterface
