#include "Mcp2515Bus.h"

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include "Diagnostics.h"

Mcp2515Bus::Mcp2515Bus() : controller_(cfg::pins::ChipSelect, cfg::spi::ClockHz) {
}

uint8_t Mcp2515Bus::readRegisterRaw(uint8_t address) {
    SPI.beginTransaction(SPISettings(cfg::spi::ClockHz, MSBFIRST, SPI_MODE0));
    digitalWrite(cfg::pins::ChipSelect, LOW);
    SPI.transfer(cfg::spi::ReadInstruction);
    SPI.transfer(address);
    const uint8_t value = SPI.transfer(0x00);
    digitalWrite(cfg::pins::ChipSelect, HIGH);
    SPI.endTransaction();
    return value;
}

bool Mcp2515Bus::configure() {
    bool ready = false;
    if (controller_.reset() == MCP2515::ERROR_OK) {
        if (controller_.setBitrate(Bitrate, CrystalFrequency) == MCP2515::ERROR_OK) {
            ready = (controller_.setNormalMode() == MCP2515::ERROR_OK);
        }
    }
    return ready;
}

void Mcp2515Bus::reportFault(uint8_t statusRegister) {
    if (statusRegister == cfg::spi::NoReply) {
        Serial.println(F("0x00: no SPI reply. Check GND, 5V power, and SO->D12."));
    } else if (statusRegister == cfg::spi::StuckHigh) {
        Serial.println(F("0xFF: MISO stuck high. Check wiring / module power."));
    } else {
        Serial.println(F("partial SPI reply. Check for loose jumpers / lower SPI clock."));
    }
}

bool Mcp2515Bus::begin() {
    pinMode(cfg::pins::Interrupt, INPUT_PULLUP);
    pinMode(cfg::pins::ChipSelect, OUTPUT);
    digitalWrite(cfg::pins::ChipSelect, HIGH);
    SPI.begin();

    bool ready = false;
    uint32_t attempt = 0;
    // Retried rather than abandoned so that wiring can be corrected and the
    // link observed to come up without reprogramming the board.
    while (!ready) {
        ++attempt;
        const uint8_t status = readRegisterRaw(cfg::spi::StatusRegister);
        ready = configure();
        if (ready) {
            diag::printf("MCP2515 up (CANSTAT=0x%02X). Bitrate 100k / xtal 8MHz, normal mode.\n",
                        status);
        } else {
            diag::printf("MCP2515 reset FAIL (attempt %lu, CANSTAT raw=0x%02X) - ",
                        static_cast<unsigned long>(attempt), status);
            reportFault(status);
            delay(cfg::spi::RetryDelayMs);
        }
    }
    return ready;
}

void Mcp2515Bus::recover() {
    // A controller in bus-off has switched its transmitter off and will not
    // return by itself. Reinitialising restores it once the bus is healthy
    // again, which is what happens when the cluster is powered back up.
    consecutiveFailures_ = 0;
    if (configure()) {
        Serial.println(F("CAN recovered after a run of failures (bus-off?)"));
    }
}

bool Mcp2515Bus::send(const CanMessage &message) {
    can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = message.identifier();
    frame.can_dlc = message.length();
    memcpy(frame.data, message.data(), message.length());

    const bool sent = (controller_.sendMessage(&frame) == MCP2515::ERROR_OK);
    if (sent) {
        ++statistics_.transmitted;
        consecutiveFailures_ = 0;
    } else {
        ++statistics_.failed;
        ++consecutiveFailures_;
        if (consecutiveFailures_ >= cfg::spi::FailuresBeforeReset) {
            recover();
        }
    }
    return sent;
}

bool Mcp2515Bus::receive(CanMessage &message) {
    can_frame frame;
    const bool got = (controller_.readMessage(&frame) == MCP2515::ERROR_OK);
    if (got) {
        ++statistics_.received;
        message.setIdentifier(static_cast<uint16_t>(frame.can_id));
        message.setLength(frame.can_dlc);
        for (uint8_t i = 0; i < message.length(); ++i) {
            message[i] = frame.data[i];
        }
    }
    return got;
}

ICanBus::Statistics Mcp2515Bus::statistics() const {
    return statistics_;
}
