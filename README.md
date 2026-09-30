## Author

**Arif Hussain**
| M.S. Electrical and Computer Engineering, University of Florida


# Low-Latency UDP Packet Parser

Pipelined cut-through Ethernet/IP/UDP parser for low-latency UDP payload
extraction. Designed for FPGA implementation targeting 200+ MHz Fmax.

## Architecture

```
                        ┌───────────────────────────────────────────┐
  AXI-Stream In ───────►│              udp_parser_top               │───────► AXI-Stream Out
 (raw Ethernet)         │                                           │       (UDP payloads)
                        │  ┌──────────┐ ┌──────────┐ ┌──────────┐  │
                        │  │eth_parser│→│ipv4_parse│→│udp_parser│  │
                        │  │ +mac_filt│ │ +cksum   │ │          │  │
                        │  └──────────┘ └──────────┘ └──────────┘  │
                        │        │            │            │        │
                        │        ▼            ▼            ▼        │
                        │  ┌────────────────────────────────────┐   │
                        │  │     Main FSM  +  CRC-32 engine     │   │
                        │  └───────────────┬────────────────────┘   │
                        │                  │                        │
                        │  ┌───────────────▼────────────────────┐   │
                        │  │       payload_forwarder            │───┘
                        │  │  (realign + TKEEP/TLAST + backpr.) │
                        │  └────────────────────────────────────┘
                        │                                           │
                        │  ┌──────────────┐  ┌──────────────────┐   │
  AXI4-Lite ◄──────────►│  │axi4_lite_regs│  │  stats_counters  │   │
 (config/stats)         │  └──────────────┘  └──────────────────┘   │
                        │  ┌──────────────┐                         │
                        │  │timestamp_unit│                         │
                        │  └──────────────┘                         │
                        └───────────────────────────────────────────┘
```

## Parameters

| Parameter      | Default | Description                                |
|:-------------- |:------- |:------------------------------------------ |
| `DATA_WIDTH`   | 32      | Data path width: 32 or 64 bits             |
| `HAS_PREAMBLE` | 0       | Set to 1 if input includes preamble/SFD    |
| `CHECK_FCS`    | 1       | Enable CRC-32 (FCS) verification           |

## Parser Pipeline Stages

```
IDLE → [PREAMBLE] → ETH_HEADER (14 B) → IP_HEADER (20+ B) → UDP_HEADER (8 B) → PAYLOAD_FORWARD → DONE
```

**Cut-through**: Payload forwarding begins immediately after the 8-byte UDP
header is parsed — before the full frame has been received. If the FCS check
fails later, `TUSER[0]` is asserted on the output `TLAST` beat.

## AXI4-Lite Register Map

| Offset | Name          | Access | Description                            |
|:------ |:------------- |:------ |:-------------------------------------- |
| 0x00   | MAC_LO        | R/W    | Destination MAC address [31:0]         |
| 0x04   | MAC_HI        | R/W    | Destination MAC address [47:32]        |
| 0x08   | CONFIG        | R/W    | [0] enable  [1] promiscuous  [31] clr  |
| 0x0C   | STATUS        | R      | Reserved                               |
| 0x10   | PKT_CNT       | R      | Total frames received                  |
| 0x14   | ERR_CNT       | R      | Frames with any error                  |
| 0x18   | BYTE_CNT_LO   | R      | Bytes processed [31:0]                 |
| 0x1C   | BYTE_CNT_HI   | R      | Bytes processed [63:32]                |
| 0x20   | UDP_PKT_CNT   | R      | Valid UDP packets delivered             |
| 0x24   | TS_LO         | R      | Last RX timestamp [31:0]               |
| 0x28   | TS_HI         | R      | Last RX timestamp [63:32]              |
| 0x2C   | LATENCY       | R      | Last first-byte-in to first-byte-out   |

## File Hierarchy

```
udp-parser/
├── rtl/
│   ├── udp_parser_pkg.sv           # Package: constants, typedefs, CRC function
│   ├── udp_parser_top.sv           # Top-level integration + main FSM
│   ├── eth_parser.sv               # Ethernet header extraction + MAC filter
│   ├── ipv4_parser.sv              # IPv4 header extraction + checksum verify
│   ├── udp_parser.sv               # UDP header extraction
│   ├── payload_forwarder.sv        # AXI-Stream output with realignment
│   ├── mac_filter.sv               # Configurable destination MAC comparison
│   ├── checksum_verifier.sv        # Ones-complement sum (IPv4)
│   ├── timestamp_unit.sv           # Free-running counter with capture
│   ├── stats_counters.sv           # Packet/error/byte/UDP counters
│   ├── axi4_lite_regs.sv           # AXI4-Lite register bank
│   └── axi_stream_if.sv            # AXI-Stream interface definition
├── tb/
│   ├── udp_parser_tb_top.sv        # Top-level testbench (10 directed tests)
│   ├── udp_parser_if.sv            # Testbench interface wrapper
│   ├── pkt_generator.sv            # Ethernet frame generator (valid + errors)
│   └── pkt_checker.sv              # Payload verification
├── sim/
│   ├── Makefile                    # Build targets for Vivado + Icarus
│   └── filelist.f                  # Compilation file list
└── README.md
```

## Verification

The directed testbench (`udp_parser_tb_top.sv`) covers:

1. **Valid UDP packet** — verify payload extraction byte-by-byte
2. **Bad FCS** — corrupted CRC-32, verify error flag
3. **Bad IP checksum** — verify packet is dropped
4. **Non-UDP protocol** — TCP (0x06), verify packet is dropped
5. **Truncated packet** — frame ends before headers complete
6. **Runt frame** — frame shorter than 64 bytes
7. **Back-to-back packets** — no inter-frame gap
8. **Backpressure** — TREADY toggled mid-packet
9. **Statistics counters** — read via AXI4-Lite
10. **Latency measurement** — read via AXI4-Lite

### Running simulation

**Vivado** (requires xvlog, xelab, xsim in PATH):
```bash
cd sim && make vivado
```

**Icarus Verilog**:
```bash
cd sim && make iverilog
```

## Key Metrics

| Metric       | Target   | Notes                                          |
|:------------ |:-------- |:---------------------------------------------- |
| Latency      | ~11 clk  | First byte in → first payload byte out (32-bit)|
| Fmax         | 200+ MHz | Registered outputs, parallel CRC               |
| Data width   | 32/64    | Parameterized                                  |

## Technologies

- **Language**: SystemVerilog (IEEE 1800-2017)
- **Synthesis**: Xilinx Vivado 2023.x+
- **Simulation**: Vivado xsim, Icarus Verilog, or any SV-2012 simulator
