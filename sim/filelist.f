// ============================================================================
// filelist.f — Compilation file list for UDP parser
// ============================================================================

// ── Package (must be compiled first) ────────────────────────────────────
../rtl/udp_parser_pkg.sv

// ── Interfaces ──────────────────────────────────────────────────────────
../rtl/axi_stream_if.sv

// ── RTL modules ─────────────────────────────────────────────────────────
../rtl/mac_filter.sv
../rtl/checksum_verifier.sv
../rtl/timestamp_unit.sv
../rtl/stats_counters.sv
../rtl/axi4_lite_regs.sv
../rtl/eth_parser.sv
../rtl/ipv4_parser.sv
../rtl/udp_parser.sv
../rtl/payload_forwarder.sv
../rtl/udp_parser_top.sv

// ── Testbench ───────────────────────────────────────────────────────────
../tb/udp_parser_if.sv
../tb/pkt_generator.sv
../tb/pkt_checker.sv
../tb/udp_parser_tb_top.sv
