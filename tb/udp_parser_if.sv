// ============================================================================
// udp_parser_if.sv — Testbench interface wrapper
// ============================================================================
interface udp_parser_if #(
    parameter int DATA_WIDTH = 32
)(
    input logic clk,
    input logic rst_n
);

    localparam int BW = DATA_WIDTH / 8;

    // AXI-Stream input (to DUT)
    logic [DATA_WIDTH-1:0]  s_tdata;
    logic [BW-1:0]          s_tkeep;
    logic                   s_tvalid;
    logic                   s_tready;
    logic                   s_tlast;

    // AXI-Stream output (from DUT)
    logic [DATA_WIDTH-1:0]  m_tdata;
    logic [BW-1:0]          m_tkeep;
    logic                   m_tvalid;
    logic                   m_tready;
    logic                   m_tlast;
    logic                   m_tuser;

    // AXI4-Lite
    logic [7:0]  awaddr;
    logic        awvalid;
    logic        awready;
    logic [31:0] wdata;
    logic [3:0]  wstrb;
    logic        wvalid;
    logic        wready;
    logic [1:0]  bresp;
    logic        bvalid;
    logic        bready;
    logic [7:0]  araddr;
    logic        arvalid;
    logic        arready;
    logic [31:0] rdata;
    logic [1:0]  rresp;
    logic        rvalid;
    logic        rready;

    // ── AXI4-Lite write task ────────────────────────────────────────────
    task automatic axil_write(input logic [7:0] addr, input logic [31:0] data);
        @(posedge clk);
        awaddr  <= addr;
        awvalid <= 1'b1;
        wdata   <= data;
        wstrb   <= 4'hF;
        wvalid  <= 1'b1;
        bready  <= 1'b1;

        fork
            begin
                wait (awready);
                @(posedge clk);
                awvalid <= 1'b0;
            end
            begin
                wait (wready);
                @(posedge clk);
                wvalid <= 1'b0;
            end
        join

        wait (bvalid);
        @(posedge clk);
        bready <= 1'b0;
    endtask

    // ── AXI4-Lite read task ─────────────────────────────────────────────
    task automatic axil_read(input logic [7:0] addr, output logic [31:0] data);
        @(posedge clk);
        araddr  <= addr;
        arvalid <= 1'b1;
        rready  <= 1'b1;

        wait (arready);
        @(posedge clk);
        arvalid <= 1'b0;

        wait (rvalid);
        data = rdata;
        @(posedge clk);
        rready <= 1'b0;
    endtask

endinterface
