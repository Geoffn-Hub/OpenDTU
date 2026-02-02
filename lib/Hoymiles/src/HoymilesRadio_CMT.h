// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "HoymilesRadio.h"
#include "commands/CommandAbstract.h"
#include "types.h"
#include <Arduino.h>
#include <cmt2300wrapper.h>
#include <memory>
#include <queue>
#include <vector>

// number of fragments hold in buffer
#define FRAGMENT_BUFFER_SIZE 30

#ifndef HOYMILES_CMT_WORK_FREQ
#define HOYMILES_CMT_WORK_FREQ 865000000
#endif

enum CountryModeId_t {
    MODE_EU,
    MODE_US,
    MODE_BR,
    CountryModeId_Max
};

struct CountryFrequencyDefinition_t {
    FrequencyBand_t Band;
    uint32_t Freq_Min;
    uint32_t Freq_Max;
    uint32_t Freq_Legal_Min;
    uint32_t Freq_Legal_Max;
    uint32_t Freq_Default;
    uint32_t Freq_StartUp;
};

struct CountryFrequencyList_t {
    CountryModeId_t mode;
    CountryFrequencyDefinition_t definition;
};

class HoymilesRadio_CMT : public HoymilesRadio {
public:
    void init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const int8_t pin_gpio2, const int8_t pin_gpio3);
    void loop();
    void setPALevel(const int8_t paLevel);
    void setInverterTargetFrequency(const uint32_t frequency);
    uint32_t getInverterTargetFrequency() const;

    bool isConnected() const;

    uint32_t getMinFrequency() const;
    uint32_t getMaxFrequency() const;
    static constexpr uint32_t getChannelWidth()
    {
        return FH_OFFSET * CMT2300A_ONE_STEP_SIZE;
    }

    CountryModeId_t getCountryMode() const;
    void setCountryMode(const CountryModeId_t mode);

    uint32_t getInvBootFrequency() const;

    void setCaptureMode(const bool enabled);
    bool getCaptureMode() const;

    uint32_t getFrequencyFromChannel(const uint8_t channel) const;
    uint8_t getChannelFromFrequency(const uint32_t frequency) const;

    std::vector<CountryFrequencyList_t> getCountryFrequencyList() const;

private:
    void ARDUINO_ISR_ATTR handleInt1();
    void ARDUINO_ISR_ATTR handleInt2();

    void sendEsbPacket(CommandAbstract& cmd);

    std::unique_ptr<CMT2300A> _radio;

    bool _captureMode = false;

    volatile bool _packetReceived = false;
    volatile bool _packetSent = false;

    bool _gpio2_configured = false;
    bool _gpio3_configured = false;

    std::queue<fragment_t> _rxBuffer;
    TimeoutHelper _txTimeout;

    uint32_t _inverterTargetFrequency = HOYMILES_CMT_WORK_FREQ;

    bool cmtSwitchDtuFreq(const uint32_t to_frequency);

    CountryModeId_t _countryMode;

    // Channel hopping for capture mode
    uint8_t _captureChIdx = 0;
    uint32_t _captureLastHop = 0;
    static constexpr uint32_t CAPTURE_HOP_INTERVAL_MS = 50; // dwell time per channel (50ms × 29ch = ~1.5s sweep)

    // Receive-side frequency hopping for MIT inverters
    // MIT-5000-8T hops response fragments across 3 frequencies spaced 250kHz apart:
    //   frag 1 → base - 250kHz, frag 2 → base, frag 3 → base + 250kHz (repeats)
    // Without hopping, OpenDTU only receives 2 of 6 fragments (those on the base freq).
    bool _rxHopEnabled = false;          // true when waiting for MIT response
    uint8_t _rxHopBaseChannel = 0;       // channel corresponding to _inverterTargetFrequency
    uint8_t _rxHopLastFragId = 0;        // last fragment ID received (1-based)
    uint32_t _rxHopLastFragTime = 0;     // millis() when last fragment was received

    // Map fragment ID (1-based) to channel offset from base: -1, 0, +1 repeating
    static int8_t getHopOffsetForFragment(const uint8_t fragId);
    void rxHopToNextFragment(const uint8_t receivedFragId);
};
