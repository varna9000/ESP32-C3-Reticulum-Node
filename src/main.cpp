#include <Arduino.h>
#include "ReticulumNode.h"
#include "Utils.h"

// ============================================================================
// GPIO Control Configuration
// ============================================================================
// Define the pin you want to control (can be relay, LED, etc.)
#define CONTROL_PIN 25          // GPIO25 = Heltec V2 onboard LED
// #define CONTROL_PIN 12       // Or use GPIO12 for external relay
// #define CONTROL_PIN 2        // Or GPIO2, etc.

#define CONTROL_PIN_ACTIVE_HIGH true  // true = HIGH turns ON, false = LOW turns ON

// Global instance of the main node application class
ReticulumNode reticulumNode;

// Track pin state
bool controlPinState = false;

// ============================================================================
// Helper: Set control pin state
// ============================================================================
void setControlPin(bool state) {
    controlPinState = state;
    if (CONTROL_PIN_ACTIVE_HIGH) {
        digitalWrite(CONTROL_PIN, state ? HIGH : LOW);
    } else {
        digitalWrite(CONTROL_PIN, state ? LOW : HIGH);  // Inverted for active-low relays
    }
    DebugSerial.print(">> Control pin ");
    DebugSerial.print(CONTROL_PIN);
    DebugSerial.print(" set to: ");
    DebugSerial.println(state ? "ON" : "OFF");
}

// ============================================================================
// Application Data Handler - Called when messages arrive
// ============================================================================
void myAppDataReceiver(const uint8_t *source_address, const std::vector<uint8_t> &data)
{
    DebugSerial.print("\n<<<< App Layer Received ");
    DebugSerial.print(data.size());
    DebugSerial.print(" bytes from ");
    Utils::printBytes(source_address, RNS_ADDRESS_SIZE, DebugSerial);
    DebugSerial.println();
    
    // Convert data to string for easy parsing
    String message = "";
    for (uint8_t byte : data) {
        message += (char)byte;
    }
    message.trim();  // Remove whitespace
    message.toUpperCase();  // Case-insensitive matching
    
    DebugSerial.print("Message: \"");
    DebugSerial.print(message);
    DebugSerial.println("\"");
    
    // ========================================================================
    // Command parsing - Add your own commands here!
    // ========================================================================
    
    if (message == "ON" || message == "1" || message == "HIGH") {
        setControlPin(true);
    }
    else if (message == "OFF" || message == "0" || message == "LOW") {
        setControlPin(false);
    }
    else if (message == "TOGGLE" || message == "T") {
        setControlPin(!controlPinState);
    }
    else if (message == "STATUS" || message == "?") {
        DebugSerial.print(">> Status: Pin ");
        DebugSerial.print(CONTROL_PIN);
        DebugSerial.print(" is ");
        DebugSerial.println(controlPinState ? "ON" : "OFF");
        // TODO: You could send a response packet back here
    }
    else if (message.startsWith("PWM:")) {
        // Example: "PWM:128" sets PWM duty cycle (0-255)
        int pwmValue = message.substring(4).toInt();
        pwmValue = constrain(pwmValue, 0, 255);
        analogWrite(CONTROL_PIN, pwmValue);
        DebugSerial.print(">> PWM set to: ");
        DebugSerial.println(pwmValue);
    }
    else if (message.startsWith("PIN:")) {
        // Example: "PIN:13:ON" or "PIN:13:OFF" - control any pin
        int firstColon = message.indexOf(':');
        int secondColon = message.indexOf(':', firstColon + 1);
        if (secondColon > firstColon) {
            int pin = message.substring(firstColon + 1, secondColon).toInt();
            String state = message.substring(secondColon + 1);
            pinMode(pin, OUTPUT);
            if (state == "ON" || state == "1" || state == "HIGH") {
                digitalWrite(pin, HIGH);
                DebugSerial.print(">> GPIO"); DebugSerial.print(pin); DebugSerial.println(" = HIGH");
            } else if (state == "OFF" || state == "0" || state == "LOW") {
                digitalWrite(pin, LOW);
                DebugSerial.print(">> GPIO"); DebugSerial.print(pin); DebugSerial.println(" = LOW");
            }
        }
    }
    else {
        DebugSerial.println(">> Unknown command. Valid commands:");
        DebugSerial.println("   ON, OFF, TOGGLE, STATUS");
        DebugSerial.println("   PWM:<0-255>");
        DebugSerial.println("   PIN:<gpio>:<ON|OFF>");
    }
}

void setup()
{
    // Initialize Serial (USB/UART0) for debug output
    DebugSerial.begin(115200);
    delay(100);

    // Initialize control pin
    pinMode(CONTROL_PIN, OUTPUT);
    setControlPin(false);  // Start with pin OFF
    
    // Start KISS serial interface (platform-specific UART)
    KissSerial.begin(KISS_SERIAL_SPEED, SERIAL_8N1, KISS_UART_RX, KISS_UART_TX);

    DebugSerial.println("\n\n===================================");
    DebugSerial.println(" ESP32 Reticulum Gateway - Booting ");
    DebugSerial.println("===================================");
    DebugSerial.print("Control Pin: GPIO");
    DebugSerial.println(CONTROL_PIN);

    // Initialize the Reticulum node subsystems
    reticulumNode.setup();

    // Register the application data handler
    reticulumNode.setAppDataHandler(myAppDataReceiver);

    DebugSerial.println("-----------------------------------");
    DebugSerial.println(" Setup Complete. Entering main loop.");
    DebugSerial.println("-----------------------------------");
}

void loop()
{
    // Run the main node loop function
    reticulumNode.loop();

#if DEMO_TRAFFIC_ENABLED
    // SEND MESSAGE EVERY 10 SECONDS (demo mode)
    static uint32_t last_send = 0;
    if (millis() - last_send >= 10000) {
        last_send = millis();

        // Full 16-byte destination hash for PLAIN destination ["esp32", "node"]
        uint8_t dest_hash[16] = {0xB6, 0x01, 0x0E, 0xA1, 0x1F, 0xDF, 0xC0, 0x4E,
                                 0x01, 0x88, 0x3B, 0xD6, 0x06, 0xC5, 0x42, 0xD7};

        // Prepare message payload
        const char* msg = "Hello from ESP32";
        std::vector<uint8_t> payload((uint8_t*)msg, (uint8_t*)msg + strlen(msg));

        // Serialize packet using official Reticulum wire format
        uint8_t buffer[MAX_PACKET_SIZE];
        size_t packet_len = 0;

        bool success = ReticulumPacket::serialize(
            buffer, packet_len,
            dest_hash,
            RNS_PACKET_DATA,
            RNS_DEST_PLAIN,
            RNS_PROPAGATION_BROADCAST,
            RNS_CONTEXT_NONE,
            0,
            payload
        );

        if (success) {
            DebugSerial.println("\n==== SENDING PACKET ====");
            DebugSerial.print("Packet size: ");
            DebugSerial.println(packet_len);
            DebugSerial.print("Destination hash: ");
            Utils::printBytes(dest_hash, 16, DebugSerial);
            DebugSerial.println();
            DebugSerial.print("Message: ");
            DebugSerial.println(msg);

            // Send via Serial interface with KISS framing
            reticulumNode.getInterfaceManager().sendPacketVia(InterfaceType::SERIAL_PORT, buffer, packet_len, dest_hash);
        } else {
            DebugSerial.println("ERROR: Failed to serialize packet!");
        }
    }
#endif
}
