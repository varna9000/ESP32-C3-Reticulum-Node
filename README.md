# ESP32 Reticulum Network Stack Gateway Node
## Technical Specification Document
**Document Version:** 2.1  
**Classification:** Unclassified  
**Date:** 2026-02-13  
**System Designation:** ESP32-RNS-GW

---

## ⚡ UPDATE: Heltec WiFi LoRa 32 V2 Support Added

Support has been added for the **Heltec WiFi LoRa 32 V2** board (ESP32 original with SX1276 LoRa radio).

### Quick Installation for Heltec V2

 **Build and upload:**
   ```bash
   pio run -e heltec_wifi_lora_32_V2
   pio run -e heltec_wifi_lora_32_V2 --target upload
   ```

 **Monitor output:**
   ```bash
   pio device monitor -b 115200
   ```

### V2 Hardware Specifications

| Feature | Value |
|---------|-------|
| MCU | ESP32 (Xtensa dual-core, 240 MHz) |
| LoRa Chip | SX1276 (868/915 MHz) |
| Bluetooth | Classic + BLE |
| OLED | SSD1306 128x64 (built-in) |
| Vext Control | GPIO21 (required for LoRa/OLED power) |

### V2 Pin Mapping

| Function | GPIO | Notes |
|----------|------|-------|
| LoRa CS | 18 | SPI Chip Select |
| LoRa RST | 14 | Reset |
| LoRa DIO0 | 26 | IRQ/Interrupt |
| LoRa SCK | 5 | SPI Clock |
| LoRa MISO | 19 | SPI Data In |
| LoRa MOSI | 27 | SPI Data Out |
| Vext | 21 | Power control (LOW=ON) |
| KISS RX | 13 | Serial input (alternate) |
| KISS TX | 12 | Serial output (alternate) |
| LED | 25 | User LED |
| OLED SDA | 4 | I2C Data |
| OLED SCL | 15 | I2C Clock |
| OLED RST | 16 | Display Reset |

### V2 Key Differences from V3/V4

- **Vext Power Control**: V2 requires GPIO21 set LOW before LoRa module will operate
- **UART Conflict**: Default UART2 RX (GPIO16) conflicts with OLED_RST; firmware uses GPIO12/13 instead
- **LoRa Chip**: V2 uses SX1276 (compatible with SX1278 driver), V3/V4 use SX1262

### V2 Troubleshooting

If LoRa fails to initialize:
- Verify Vext is enabled (GPIO21 should be LOW)
- Check SPI connections: SCK=5, MISO=19, MOSI=27, CS=18
- Ensure antenna is connected
- Check serial output for error codes

---

## 1.0 SCOPE

### 1.1 Purpose
This document provides complete technical specifications, operational procedures, and system requirements for the ESP32 Reticulum Network Stack (RNS) Gateway Node firmware. The system provides a multi-interface network gateway capable of bridging multiple physical and logical network interfaces using the Reticulum protocol stack.

### 1.2 Applicability
This specification applies to all ESP32-based hardware platforms including but not limited to:
- ESP32-C3 series microcontrollers
- ESP32-S2 series microcontrollers
- ESP32-S3 series microcontrollers
- ESP32-C5 series microcontrollers
- ESP32-C6 series microcontrollers
- ESP32 (original) series microcontrollers
- Heltec LoRa32 v2 development boards **(NEW)**
- Heltec LoRa32 v3/v4 development boards

### 1.3 Document Structure
- Section 2.0: System Overview and Architecture
- Section 3.0: Technical Specifications
- Section 4.0: Hardware Requirements
- Section 5.0: Software Requirements
- Section 6.0: Installation and Configuration Procedures
- Section 7.0: Operational Procedures
- Section 8.0: Interface Specifications
- Section 9.0: Performance Characteristics
- Section 10.0: Maintenance and Troubleshooting

---

## 2.0 SYSTEM OVERVIEW

### 2.1 System Description
The ESP32-RNS-GW is a firmware implementation that transforms ESP32-series microcontrollers into multi-interface Reticulum Network Stack gateway nodes. The system provides transparent packet routing and bridging capabilities across heterogeneous network interfaces, enabling seamless communication between devices operating on different physical layers.

### 2.2 System Architecture
The system implements a modular architecture consisting of the following primary components:

#### 2.2.1 Core Components
- **ReticulumNode**: Central application controller managing all subsystems
- **InterfaceManager**: Physical and logical interface abstraction layer
- **RoutingTable**: Dynamic routing information base (RIB)
- **LinkManager**: Reliable transport layer management
- **Link**: Point-to-point reliable connection state machine
- **KISSProcessor**: KISS protocol framing processor
- **ReticulumPacket**: Packet serialization/deserialization engine

#### 2.2.2 Network Interfaces
The system supports the following interface types:
1. **WiFi UDP**: IEEE 802.11 wireless local area network, UDP transport
2. **ESP-NOW**: Espressif proprietary peer-to-peer protocol
3. **Serial UART**: Asynchronous serial communication with KISS framing
4. **Bluetooth Classic**: IEEE 802.15.1 serial profile with KISS framing
5. **LoRa**: Long-range radio communication (SX1278 modules)
6. **HAM Modem**: Amateur radio TNC interface with AX.25 protocol
7. **IPFS**: InterPlanetary File System content addressing

### 2.3 Operational Modes
- **Gateway Mode**: Transparent packet forwarding between interfaces
- **Node Mode**: Endpoint node with local application processing
- **Hybrid Mode**: Simultaneous gateway and node operation

---

## 3.0 TECHNICAL SPECIFICATIONS

### 3.1 Protocol Compliance
- **Reticulum Network Stack**: Compatible with Reticulum protocol specification
- **KISS Protocol**: RFC 1055 compliant (with extensions)
- **AX.25 Protocol**: Amateur Packet Radio Protocol, Version 2.2 compliant
- **APRS Protocol**: Automatic Packet Reporting System specification compliant
- **IPFS Protocol**: InterPlanetary File System gateway API compatible

### 3.2 Network Layer Specifications

#### 3.2.1 Address Format
- **RNS Address Size**: 8 bytes (64 bits)
- **Address Type**: Cryptographically derived or randomly generated
- **Address Persistence**: Stored in non-volatile memory (EEPROM)

#### 3.2.2 Packet Format
- **Maximum Packet Size**: 219 bytes (19-byte header + 200-byte payload)
- **Header Size**: 19 bytes (RNS Header Type 1)
- **Maximum Payload**: 200 bytes (configurable via `RNS_MAX_PAYLOAD`)
- **Hop Limit**: 15 hops (configurable via `MAX_HOPS`)

#### 3.2.3 Routing Protocol
- **Protocol Type**: Distance-vector with announce-based discovery
- **Route Update Interval**: 180 seconds (configurable)
- **Route Timeout**: 555 seconds (3 × announce interval + 15 seconds)
- **Maximum Routes**: 20 entries (configurable)
- **Route Metrics**: Hop count, last-heard timestamp

### 3.3 Transport Layer Specifications

#### 3.3.1 Link Layer
- **Reliability Mechanism**: Acknowledgment-based with retransmission
- **Window Size**: 1 packet (simplified implementation)
- **Maximum Retries**: 3 attempts per packet
- **Link Request Timeout**: 10 seconds
- **Data Packet Timeout**: 5 seconds
- **Maximum Active Links**: 10 concurrent connections

#### 3.3.2 Sequence Numbers
- **Sequence Number Size**: 16 bits (0-65535)
- **Sequence Space**: Circular with wraparound detection
- **Initial Sequence**: Random or zero-based

### 3.4 Interface Specifications

#### 3.4.1 WiFi UDP Interface
- **Transport Protocol**: User Datagram Protocol (UDP)
- **Port Number**: 4242 (default, configurable)
- **Address Resolution**: Broadcast for discovery, unicast for routing
- **MTU**: Limited by WiFi frame size (typically 1500 bytes)

#### 3.4.2 ESP-NOW Interface
- **Protocol**: Espressif ESP-NOW proprietary protocol
- **Maximum Peers**: 20 (ESP32 hardware limitation)
- **Frame Size**: 250 bytes maximum
- **Encryption**: Optional (configurable)

#### 3.4.3 Serial Interface (KISS)
- **Baud Rate**: 115200 bps (configurable)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Framing**: KISS protocol (FEND/FESC encoding)

#### 3.4.4 Bluetooth Classic Interface
- **Profile**: Serial Port Profile (SPP)
- **Baud Rate**: 115200 bps (logical)
- **Framing**: KISS protocol
- **Availability**: ESP32, ESP32-S2, ESP32-S3 only

#### 3.4.5 LoRa Interface
- **Modulation**: LoRa (Long Range)
- **Frequency Range**: 915.0 MHz (configurable, region-dependent)
- **Bandwidth**: 125 kHz (configurable)
- **Spreading Factor**: 7 (configurable)
- **Coding Rate**: 5 (configurable)
- **Output Power**: 10 dBm (configurable)

#### 3.4.6 HAM Modem Interface
- **Protocol**: KISS over serial, AX.25 over packet radio
- **Baud Rate**: 9600 bps (TNC interface, configurable)
- **Audio Modem**: AFSK 1200/2200 Hz (Bell 202 compatible)
- **Sample Rate**: 8000 Hz (audio processing)

#### 3.4.7 IPFS Interface
- **Access Method**: HTTP gateway client
- **Gateway URL**: Configurable (default: https://ipfs.io/ipfs/)
- **Maximum Content Size**: 10 KB (configurable)
- **Timeout**: 10 seconds (fetch), 30 seconds (publish)

---

## 4.0 HARDWARE REQUIREMENTS

### 4.1 Minimum Hardware Requirements
- **Microcontroller**: ESP32-series (any variant)
- **Flash Memory**: 4 MB minimum (8 MB recommended)
- **RAM**: 320 KB minimum (520 KB recommended)
- **EEPROM**: 16 bytes minimum (for address storage)

### 4.2 Platform-Specific Requirements

#### 4.2.1 ESP32-C3
- **UART**: UART0 (debug), UART1 (KISS interface)
- **GPIO Pins**: 18 (RX), 19 (TX) for UART1
- **Bluetooth**: Not available (BLE only)

#### 4.2.2 ESP32-S2
- **UART**: UART0 (debug), UART2 (KISS interface)
- **GPIO Pins**: 33 (RX), 34 (TX) for UART2 (configurable)
- **Bluetooth**: Available

#### 4.2.3 ESP32-S3
- **UART**: UART0 (debug), UART2 (KISS interface)
- **GPIO Pins**: 17 (RX), 18 (TX) for UART2
- **Bluetooth**: Available

#### 4.2.4 ESP32 (Original)
- **UART**: UART0 (debug), UART2 (KISS interface)
- **GPIO Pins**: 16 (RX), 17 (TX) for UART2
- **Bluetooth**: Available

#### 4.2.5 Heltec WiFi LoRa 32 V2
- **UART**: UART0 (debug), UART2 (KISS interface)
- **GPIO Pins**: 13 (RX), 12 (TX) for UART2 (alternate pins to avoid OLED conflict)
- **Bluetooth**: Available (Classic + BLE)
- **LoRa**: SX1276 on SPI (SCK=5, MISO=19, MOSI=27, CS=18, RST=14, DIO0=26)
- **Vext**: GPIO21 must be LOW to enable LoRa/OLED power
- **OLED**: SSD1306 128x64 on I2C (SDA=4, SCL=15, RST=16)

### 4.3 Optional Hardware
- **LoRa Module**: SX1278-compatible (for LoRa interface)
- **HAM TNC**: KISS-compatible terminal node controller
- **Audio Interface**: ADC/DAC for audio modem operation
- **External Antenna**: For improved range (WiFi, LoRa, HAM)

---

## 5.0 SOFTWARE REQUIREMENTS

### 5.1 Development Environment
- **PlatformIO**: Version 6.0 or later (recommended)
- **Arduino IDE**: Version 2.0 or later (alternative)
- **VSCode**: With PlatformIO extension (recommended)

### 5.2 Required Libraries
- **Espressif ESP32 Core**: Version 3.0.0 or later
- **RadioLib**: Version 6.7.0 or later (for LoRa support)
- **HTTPClient**: Included in ESP32 core (for IPFS)
- **WiFi**: Included in ESP32 core
- **BluetoothSerial**: Included in ESP32 core (where applicable)

### 5.3 Build System Requirements
- **Compiler**: GCC for Xtensa (ESP32) or RISC-V (ESP32-C3/C6)
- **C++ Standard**: C++11 or later
- **Build System**: PlatformIO build system or Arduino build system

### 5.4 Configuration Requirements
- **WiFi Credentials**: SSID and password (for WiFi interface)
- **Node Address**: Auto-generated on first boot, stored in EEPROM

---

## 6.0 INSTALLATION AND CONFIGURATION PROCEDURES

### 6.1 Installation Procedure

#### 6.1.1 Prerequisites
1. Install PlatformIO CLI or VS Code with PlatformIO extension
2. Clone or download firmware repository
3. Connect ESP32 device via USB

#### 6.1.2 Build and Flash

**For Heltec WiFi LoRa 32 V2:**
```bash
# Build firmware
pio run -e heltec_wifi_lora_32_V2

# Upload firmware
pio run -e heltec_wifi_lora_32_V2 --target upload

# Monitor serial output
pio device monitor -b 115200
```

**For Heltec LoRa32 V3:**
```bash
pio run -e heltec_wifi_lora_32_V3
pio run -e heltec_wifi_lora_32_V3 --target upload
```

**For ESP32-C3:**
```bash
pio run -e esp32-c3-devkitm-1
pio run -e esp32-c3-devkitm-1 --target upload
```

**For generic ESP32:**
```bash
pio run -e esp32dev
pio run -e esp32dev --target upload
```

### 6.2 Available Build Environments

| Environment | Board | LoRa Support |
|-------------|-------|--------------|
| `esp32-c3-devkitm-1` | ESP32-C3 DevKit | No |
| `esp32dev` | Generic ESP32 | No |
| `esp32-s2-devkitm-1` | ESP32-S2 DevKit | No |
| `esp32-s3-devkitc-1` | ESP32-S3 DevKit | No |
| `heltec_wifi_lora_32_V2` | Heltec V2 | Yes (SX1276) |
| `heltec_wifi_lora_32_V3` | Heltec V3 | Yes (SX1262) |

### 6.3 Configuration

#### 6.3.1 Interface Configuration
- **WiFi**: Configure SSID/password in `Config.h` or `Config.cpp`
- **LoRa**: Configure frequency, bandwidth, spreading factor in `Config.h`
- **HAM Modem**: Configure callsign, SSID, TNC baud rate
- **IPFS**: Configure gateway URL (if different from default)

#### 6.3.2 Network Configuration
- **Subscribed Groups**: Add group addresses in `SUBSCRIBED_GROUPS`
- **Routing Parameters**: Adjust timeouts if needed
- **Link Parameters**: Adjust retry/timeout values if needed

---

### 6.4 Production defaults and notes
- **Debugging:** Debug output is conservative by default. The project includes a `DebugSerial` shim and `DEBUG_ENABLED` build flag; set `DEBUG_ENABLED` to 1 only when actively debugging.
- **Demo traffic:** Periodic demo/send behavior is disabled by default (`DEMO_TRAFFIC_ENABLED = 0`) to avoid generating network traffic in production builds.
- **ESP-NOW peer management:** The `RoutingTable` evicts stale ESP-NOW peers when routes are replaced; the `InterfaceManager` removes associated ESP-NOW peers to keep the peer list consistent.
- **Builds:** Use PlatformIO to build per-environment or all environments. Examples:
   - Build default env: `pio run -e esp32-c3-devkitm-1`
   - Build all envs: `pio run`
   - List available envs: `pio run --list-targets`
- **Tooling:** Keep PlatformIO and Espressif cores up to date (`pip install -U platformio` and `pio update`) to access the latest board definitions and toolchains.
- **Tests:** Test scripts are in the `tests/` directory; some tests are hardware-dependent (serial ports, radio modules) and require the corresponding devices connected and configured. Run individual scripts with `python tests/<script>.py`.


## 7.0 OPERATIONAL PROCEDURES

### 7.1 System Startup Procedure
1. Apply power to ESP32 device
2. Observe serial monitor output (115200 baud)
3. Verify initialization sequence:
   - EEPROM initialization
   - Node address generation/loading
   - Interface initialization
   - Routing table initialization
4. Verify "Setup Complete" message

### 7.2 Normal Operation
- System operates autonomously after initialization
- Periodic announce packets transmitted (every 180 seconds)
- Routing table updated automatically
- Packets forwarded based on routing decisions
- Link layer manages reliable connections

### 7.3 Interface-Specific Operations

#### 7.3.1 KISS Interface Operation
1. Connect serial device to configured UART pins
2. Configure serial device for KISS protocol
3. Send/receive KISS-framed Reticulum packets
4. Monitor via serial debug output

#### 7.3.2 WiFi Interface Operation
1. Verify WiFi connection status
2. Monitor UDP port 4242 for incoming packets
3. Verify announce packet transmission
4. Monitor routing table for learned routes

#### 7.3.3 LoRa Interface Operation
1. Verify LoRa module initialization
2. Monitor for incoming LoRa packets
3. Verify transmission success
4. Monitor signal quality (if available)

### 7.4 Shutdown Procedure
1. Gracefully close active links (if possible)
2. Save persistent state (if modified)
3. Power down device

---

## 8.0 INTERFACE SPECIFICATIONS

### 8.1 KISS Protocol Interface

#### 8.1.1 Frame Structure
```
[FEND] [CMD] [DATA...] [FEND]
```

#### 8.1.2 Special Characters
- **FEND**: 0xC0 (Frame End)
- **FESC**: 0xDB (Frame Escape)
- **TFEND**: 0xDC (Transposed FEND)
- **TFESC**: 0xDD (Transposed FESC)

#### 8.1.3 Command Byte
- **0x00**: Data frame
- **0x01-0x0F**: TNC configuration (not used)

### 8.2 Reticulum Packet Format

#### 8.2.1 Header Structure (Type 1)
```
Byte 0:     Flags (packet type, destination type, propagation, etc.)
Byte 1:     Hops
Bytes 2-17: Destination hash (16 bytes)
Byte 18:    Context
Bytes 19+:  Payload data
```

#### 8.2.2 Packet Types
- **0x00**: DATA packet
- **0x01**: ANNOUNCE packet
- **0x02**: LINKREQ packet
- **0x03**: PROOF packet

### 8.3 AX.25 Frame Format

#### 8.3.1 Frame Structure
```
[FLAG] [DEST ADDR] [SRC ADDR] [DIGI ADDRS...] [CTRL] [PID] [INFO] [FCS] [FLAG]
```

#### 8.3.2 Address Format
- **6 bytes**: Callsign (shifted left 1 bit)
- **1 byte**: SSID and control bits

#### 8.3.3 FCS Calculation
- **Algorithm**: CRC-16 CCITT (polynomial 0x8408, reversed)
- **Initial Value**: 0xFFFF
- **Final XOR**: 0xFFFF

---

## 9.0 PERFORMANCE CHARACTERISTICS

### 9.1 Throughput Specifications
- **Maximum Packet Rate**: ~10 packets/second (interface-dependent)
- **Maximum Payload Throughput**: ~2 KB/s (interface-dependent)
- **Link Establishment Time**: <1 second (typical)

### 9.2 Latency Specifications
- **Local Processing Latency**: <10 ms (typical)
- **Interface Transmission Latency**: Interface-dependent
  - WiFi UDP: <50 ms (local network)
  - ESP-NOW: <20 ms
  - Serial: <100 ms (115200 baud)
  - LoRa: 100-500 ms (depending on spreading factor)

### 9.3 Resource Utilization
- **Flash Usage**: ~200-400 KB (depending on enabled features)
- **RAM Usage**: ~50-150 KB (depending on active connections)
- **CPU Usage**: <20% (typical operation)

### 9.4 Reliability Specifications
- **Link Reliability**: >95% (with retransmission)
- **Route Discovery**: >90% (within 3 announce intervals)
- **Packet Delivery**: >85% (network-dependent)

---

## 10.0 MAINTENANCE AND TROUBLESHOOTING

### 10.1 Preventive Maintenance
- **Periodic Checks**: Verify system operation weekly
- **Memory Monitoring**: Check free heap periodically
- **Route Table**: Monitor route table size and staleness
- **Link Status**: Monitor active link count

### 10.2 Troubleshooting Procedures

#### 10.2.1 System Not Booting
1. Verify power supply voltage and current
2. Check serial monitor for error messages
3. Verify flash memory integrity
4. Attempt firmware reflash

#### 10.2.2 WiFi Connection Failure
1. Verify SSID and password configuration
2. Check WiFi signal strength
3. Verify router compatibility (2.4 GHz required)
4. Check for IP address assignment

#### 10.2.3 Packet Loss
1. Verify interface initialization
2. Check signal strength (wireless interfaces)
3. Monitor routing table for valid routes
4. Verify hop count limits

#### 10.2.4 Memory Issues
1. Monitor free heap via serial output
2. Reduce maximum routes/links if needed
3. Reduce payload size if needed
4. Disable unused interfaces

#### 10.2.5 Heltec V2 LoRa Not Initializing
1. Verify Vext is enabled (GPIO21 should be LOW)
2. Check SPI connections: SCK=5, MISO=19, MOSI=27, CS=18
3. Verify antenna is properly connected
4. Check serial output for RadioLib error codes
5. Ensure 100ms delay after enabling Vext before LoRa init

### 10.3 Diagnostic Commands
- **Serial Monitor**: Provides real-time status and debug information
- **Memory Status**: Printed every 15 seconds (default)
- **Routing Table**: Can be printed via debug output
- **Link Status**: Available via debug output

---

## 11.0 APPENDICES

### Appendix A: Build Flag Reference
See `platformio.ini` for complete build flag documentation.

### Appendix B: Configuration Parameters
See `include/Config.h` for all configurable parameters.

### Appendix C: Additional Documentation
- `docs/ARCHITECTURE.md`: Detailed system architecture
- `docs/PACKET_FORMATS.md`: Packet format specifications
- `docs/LINK_LAYER.md`: Link layer implementation details
- `docs/KISS_INTERFACE.md`: KISS interface guide
- `docs/HAM_MODEM.md`: HAM modem interface guide
- `docs/IPFS_INTEGRATION.md`: IPFS integration guide
- `docs/ENHANCED_FEATURES.md`: Enhanced features documentation

### Appendix D: Revision History

| Version | Date | Changes |
|---------|------|---------|
| 2.1 | 2026-02-13 | Added Heltec WiFi LoRa 32 V2 support |
| 2.0 | 2025-12-31 | Initial release |

See `CHANGELOG.md` for complete revision history.

---

**Document Control:**
- **Prepared By**: Akita Engineering
- **Distribution**: Unrestricted
- **Classification**: Unclassified

---

*End of Document*
