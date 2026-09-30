// ============================================================================
// mac_filter.sv — Destination MAC address comparator
// ============================================================================
module mac_filter (
    input  logic [47:0] dest_mac,       // extracted destination MAC
    input  logic [47:0] config_mac,     // programmed MAC via AXI4-Lite
    input  logic        promiscuous,    // accept all MACs
    input  logic        valid,          // dest_mac is valid
    output logic        match           // 1 = accept packet
);

    localparam logic [47:0] BROADCAST_MAC = 48'hFFFF_FFFF_FFFF;

    always_comb begin
        if (!valid)
            match = 1'b0;
        else if (promiscuous)
            match = 1'b1;
        else
            match = (dest_mac == config_mac) || (dest_mac == BROADCAST_MAC);
    end

endmodule
