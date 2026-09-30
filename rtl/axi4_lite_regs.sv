// ============================================================================
// axi4_lite_regs.sv — AXI4-Lite slave register bank
//
// Register map (all 32-bit aligned):
//   0x00  MAC_LO        R/W   dest MAC [31:0]
//   0x04  MAC_HI        R/W   dest MAC [47:32]  (upper 16 bits ignored)
//   0x08  CONFIG        R/W   [0]=enable  [1]=promiscuous  [31]=clear_stats (W1C)
//   0x0C  STATUS        R     reserved
//   0x10  PKT_CNT       R     total frames received
//   0x14  ERR_CNT       R     frames with errors
//   0x18  BYTE_CNT_LO   R     bytes processed [31:0]
//   0x1C  BYTE_CNT_HI   R     bytes processed [63:32]
//   0x20  UDP_PKT_CNT   R     valid UDP packets delivered
//   0x24  TS_LO         R     last RX timestamp [31:0]
//   0x28  TS_HI         R     last RX timestamp [63:32]
//   0x2C  LATENCY       R     last measured latency (cycles)
// ============================================================================
module axi4_lite_regs (
    input  logic        aclk,
    input  logic        aresetn,

    // ── AXI4-Lite write address channel ─────────────────────────────────
    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,

    // ── AXI4-Lite write data channel ────────────────────────────────────
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,

    // ── AXI4-Lite write response channel ────────────────────────────────
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,

    // ── AXI4-Lite read address channel ──────────────────────────────────
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,

    // ── AXI4-Lite read data channel ─────────────────────────────────────
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    // ── Configuration outputs ───────────────────────────────────────────
    output logic [47:0] cfg_mac_addr,
    output logic        cfg_enable,
    output logic        cfg_promiscuous,
    output logic        cfg_clear_stats,

    // ── Statistics inputs ───────────────────────────────────────────────
    input  logic [31:0] stat_pkt_cnt,
    input  logic [31:0] stat_err_cnt,
    input  logic [63:0] stat_byte_cnt,
    input  logic [31:0] stat_udp_cnt,
    input  logic [63:0] stat_rx_ts,
    input  logic [31:0] stat_latency
);

    import udp_parser_pkg::*;

    // ── Internal registers ──────────────────────────────────────────────
    logic [31:0] reg_mac_lo;
    logic [15:0] reg_mac_hi;
    logic [31:0] reg_config;

    assign cfg_mac_addr    = {reg_mac_hi, reg_mac_lo};
    assign cfg_enable      = reg_config[CFG_BIT_ENABLE];
    assign cfg_promiscuous = reg_config[CFG_BIT_PROMISCUOUS];

    // ── Write FSM ───────────────────────────────────────────────────────
    logic       aw_en;
    logic [7:0] wr_addr;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_awready  <= 1'b0;
            s_axi_wready   <= 1'b0;
            s_axi_bvalid   <= 1'b0;
            s_axi_bresp    <= 2'b00;
            aw_en          <= 1'b1;
            wr_addr        <= '0;
            reg_mac_lo     <= '0;
            reg_mac_hi     <= '0;
            reg_config     <= '0;
            cfg_clear_stats <= 1'b0;
        end else begin
            cfg_clear_stats <= 1'b0;     // single-cycle pulse

            // Write address accept
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                s_axi_awready <= 1'b1;
                wr_addr       <= s_axi_awaddr;
                aw_en         <= 1'b0;
            end else begin
                s_axi_awready <= 1'b0;
            end

            // Write data accept
            if (~s_axi_wready && s_axi_wvalid && s_axi_awvalid && aw_en) begin
                s_axi_wready <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end

            // Write registers
            if (s_axi_awready && s_axi_wready) begin
                case (wr_addr)
                    REG_MAC_LO: begin
                        if (s_axi_wstrb[0]) reg_mac_lo[ 7: 0] <= s_axi_wdata[ 7: 0];
                        if (s_axi_wstrb[1]) reg_mac_lo[15: 8] <= s_axi_wdata[15: 8];
                        if (s_axi_wstrb[2]) reg_mac_lo[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) reg_mac_lo[31:24] <= s_axi_wdata[31:24];
                    end
                    REG_MAC_HI: begin
                        if (s_axi_wstrb[0]) reg_mac_hi[ 7: 0] <= s_axi_wdata[ 7: 0];
                        if (s_axi_wstrb[1]) reg_mac_hi[15: 8] <= s_axi_wdata[15: 8];
                    end
                    REG_CONFIG: begin
                        if (s_axi_wstrb[0]) reg_config[ 7: 0] <= s_axi_wdata[ 7: 0];
                        if (s_axi_wstrb[1]) reg_config[15: 8] <= s_axi_wdata[15: 8];
                        if (s_axi_wstrb[2]) reg_config[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) reg_config[31:24] <= s_axi_wdata[31:24];
                        // Bit 31 = clear_stats, write-1-to-clear
                        if (s_axi_wstrb[3] && s_axi_wdata[31])
                            cfg_clear_stats <= 1'b1;
                    end
                    default: ;
                endcase
            end

            // Write response
            if (s_axi_awready && s_axi_wready && ~s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;   // OKAY
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
                aw_en        <= 1'b1;
            end
        end
    end

    // ── Read FSM ────────────────────────────────────────────────────────
    logic [7:0] rd_addr;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= '0;
            rd_addr        <= '0;
        end else begin
            if (~s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                rd_addr       <= s_axi_araddr;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_arready && s_axi_arvalid && ~s_axi_rvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                case (rd_addr)
                    REG_MAC_LO:      s_axi_rdata <= reg_mac_lo;
                    REG_MAC_HI:      s_axi_rdata <= {16'h0, reg_mac_hi};
                    REG_CONFIG:      s_axi_rdata <= reg_config;
                    REG_STATUS:      s_axi_rdata <= '0;
                    REG_PKT_CNT:    s_axi_rdata <= stat_pkt_cnt;
                    REG_ERR_CNT:    s_axi_rdata <= stat_err_cnt;
                    REG_BYTE_CNT_LO: s_axi_rdata <= stat_byte_cnt[31:0];
                    REG_BYTE_CNT_HI: s_axi_rdata <= stat_byte_cnt[63:32];
                    REG_UDP_CNT:    s_axi_rdata <= stat_udp_cnt;
                    REG_TS_LO:      s_axi_rdata <= stat_rx_ts[31:0];
                    REG_TS_HI:      s_axi_rdata <= stat_rx_ts[63:32];
                    REG_LATENCY:    s_axi_rdata <= stat_latency;
                    default:         s_axi_rdata <= 32'hDEAD_BEEF;
                endcase
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
