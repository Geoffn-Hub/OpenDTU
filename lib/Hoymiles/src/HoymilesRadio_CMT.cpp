// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023-2025 Thomas Basler and others
 */
#include "HoymilesRadio_CMT.h"
#include "Hoymiles.h"
#include "Utils.h"
#include "crc.h"
#include <FunctionalInterrupt.h>
#include <frozen/map.h>
#include <esp_log.h>

#undef TAG
static const char* TAG = "hoymiles";

constexpr CountryFrequencyDefinition_t make_value(FrequencyBand_t Band, uint32_t Freq_Legal_Min, uint32_t Freq_Legal_Max, uint32_t Freq_Default, uint32_t Freq_StartUp)
{
    // frequency can not be lower than actual initailized base freq + 250000
    uint32_t minFrequency = CMT2300A::getBaseFrequency(Band) + HoymilesRadio_CMT::getChannelWidth();

    // =923500, 0xFF does not work
    uint32_t maxFrequency = CMT2300A::getBaseFrequency(Band) + 0xFE * HoymilesRadio_CMT::getChannelWidth();

    CountryFrequencyDefinition_t v = { Band, minFrequency, maxFrequency, Freq_Legal_Min, Freq_Legal_Max, Freq_Default, Freq_StartUp };
    return v;
}

constexpr frozen::map<CountryModeId_t, CountryFrequencyDefinition_t, 3> countryDefinition = {
    { CountryModeId_t::MODE_EU, make_value(FrequencyBand_t::BAND_860, 863e6, 870e6, 865e6, 868e6) },
    { CountryModeId_t::MODE_US, make_value(FrequencyBand_t::BAND_900, 905e6, 925e6, 918e6, 915e6) },
    { CountryModeId_t::MODE_BR, make_value(FrequencyBand_t::BAND_900, 915e6, 928e6, 918e6, 915e6) },
};

uint32_t HoymilesRadio_CMT::getFrequencyFromChannel(const uint8_t channel) const
{
    return (_radio->getBaseFrequency() + channel * getChannelWidth());
}

uint8_t HoymilesRadio_CMT::getChannelFromFrequency(const uint32_t frequency) const
{
    if ((frequency % getChannelWidth()) != 0) {
        ESP_LOGE(TAG, "%.3f MHz is not divisible by %" PRIu32 " kHz!", frequency / 1000000.0, getChannelWidth());
        return 0xFF; // ERROR
    }
    if (frequency < getMinFrequency() || frequency > getMaxFrequency()) {
        ESP_LOGE(TAG, "%.2f MHz is out of Hoymiles/CMT range! (%.2f MHz - %.2f MHz)",
            frequency / 1000000.0, getMinFrequency() / 1000000.0, getMaxFrequency() / 1000000.0);
        return 0xFF; // ERROR
    }
    if (frequency < countryDefinition.at(_countryMode).Freq_Legal_Min || frequency > countryDefinition.at(_countryMode).Freq_Legal_Max) {
        ESP_LOGE(TAG, "!!! caution: %.2f MHz is out of region legal range! (%" PRIu32 " - %" PRIu32 " MHz)",
            frequency / 1000000.0,
            static_cast<uint32_t>(countryDefinition.at(_countryMode).Freq_Legal_Min / 1e6),
            static_cast<uint32_t>(countryDefinition.at(_countryMode).Freq_Legal_Max / 1e6));
    }

    return (frequency - _radio->getBaseFrequency()) / getChannelWidth(); // frequency to channel
}

std::vector<CountryFrequencyList_t> HoymilesRadio_CMT::getCountryFrequencyList() const
{
    std::vector<CountryFrequencyList_t> v;
    for (const auto& [key, value] : countryDefinition) {
        CountryFrequencyList_t s;
        s.mode = key;
        s.definition.Band = value.Band;
        s.definition.Freq_Default = value.Freq_Default;
        s.definition.Freq_StartUp = value.Freq_StartUp;
        s.definition.Freq_Min = value.Freq_Min;
        s.definition.Freq_Max = value.Freq_Max;
        s.definition.Freq_Legal_Max = value.Freq_Legal_Max;
        s.definition.Freq_Legal_Min = value.Freq_Legal_Min;

        v.push_back(s);
    }
    return v;
}

bool HoymilesRadio_CMT::cmtSwitchDtuFreq(const uint32_t to_frequency)
{
    const uint8_t toChannel = getChannelFromFrequency(to_frequency);
    if (toChannel == 0xFF) {
        return false;
    }

    _radio->setChannel(toChannel);

    return true;
}

void HoymilesRadio_CMT::init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const int8_t pin_gpio2, const int8_t pin_gpio3)
{
    _dtuSerial.u64 = 0;

    _radio.reset(new CMT2300A(pin_sdio, pin_clk, pin_cs, pin_fcs));

    _radio->begin();

    setCountryMode(CountryModeId_t::MODE_EU);
    cmtSwitchDtuFreq(_inverterTargetFrequency); // start dtu at work freqency, for fast Rx if inverter is already on and frequency switched

    if (!_radio->isChipConnected()) {
        ESP_LOGE(TAG, "CMT2300A: Connection error!!");
        return;
    }
    ESP_LOGI(TAG, "CMT2300A: Connection successful");

    if (pin_gpio2 >= 0) {
        attachInterrupt(digitalPinToInterrupt(pin_gpio2), std::bind(&HoymilesRadio_CMT::handleInt1, this), RISING);
        _gpio2_configured = true;
    }

    if (pin_gpio3 >= 0) {
        attachInterrupt(digitalPinToInterrupt(pin_gpio3), std::bind(&HoymilesRadio_CMT::handleInt2, this), RISING);
        _gpio3_configured = true;
    }

    _isInitialized = true;

    // Start in RX mode so passive capture works without needing to TX first
    _radio->startListening();
    ESP_LOGI(TAG, "CMT2300A: RX mode active");
}

void HoymilesRadio_CMT::loop()
{
    if (!_isInitialized) {
        return;
    }

    // Capture mode: hop across all legal EU channels to find traffic
    if (_captureMode && !_busyFlag) {
        const uint32_t now = millis();
        if (now - _captureLastHop >= CAPTURE_HOP_INTERVAL_MS) {
            _captureLastHop = now;

            // Calculate channel range from legal frequency limits
            const uint8_t minCh = getChannelFromFrequency(countryDefinition.at(_countryMode).Freq_Legal_Min);
            const uint8_t maxCh = getChannelFromFrequency(countryDefinition.at(_countryMode).Freq_Legal_Max);

            if (minCh != 0xFF && maxCh != 0xFF) {
                _captureChIdx++;
                if (_captureChIdx > maxCh || _captureChIdx < minCh) {
                    _captureChIdx = minCh;
                }
                // Switch channel while staying in RX — GoStby, change channel, GoRx
                _radio->stopListening();
                _radio->setChannel(_captureChIdx);
                _radio->startListening();

                // Log channel sweep start for diagnostics
                if (_captureChIdx == minCh) {
                    ESP_LOGD(TAG, "CAPTURE: sweep restart ch %" PRIu8 "-%" PRIu8 " (%.2f-%.2f MHz)",
                        minCh, maxCh,
                        getFrequencyFromChannel(minCh) / 1000000.0,
                        getFrequencyFromChannel(maxCh) / 1000000.0);
                }
            }
        }
    }

    // RX hop timeout: if no fragment received within the expected interval,
    // cycle to the next frequency. MIT sends fragments at ~835ms intervals;
    // if we haven't heard anything in 1000ms, we may be on the wrong channel.
    if (_rxHopEnabled && _busyFlag) {
        const uint32_t now = millis();
        if (now - _rxHopLastFragTime >= 1000) {
            _rxHopLastFragTime = now;
            // Cycle through the 3 hop channels
            const uint8_t nextFrag = (_rxHopLastFragId & 0x7F) + 1;
            _rxHopLastFragId = nextFrag; // pretend we received it to advance the hop
            const int8_t offset = getHopOffsetForFragment(nextFrag + 1);
            const uint8_t nextChannel = static_cast<uint8_t>(static_cast<int8_t>(_rxHopBaseChannel) + offset);

            if (nextChannel != _radio->getChannel()) {
                // Use CMT2300A fast frequency hopping: single register write,
                // radio stays in RX mode (no state transitions needed).
                _radio->setChannelFast(nextChannel);
                ESP_LOGD(TAG, "RX HOP: timeout, fast hop to ch %" PRIu8 " (%.2f MHz)",
                    nextChannel, getFrequencyFromChannel(nextChannel) / 1000000.0);
            }
        }
    }

    if (!_gpio3_configured) {
        if (_radio->rxFifoAvailable()) { // read INT2, PKT_OK flag
            _packetReceived = true;
        }
    }

    // Step 1: Drain all available packets from the hardware FIFO into the
    // software ring buffer.
    if (_packetReceived) {
        while (_radio->available()) {
            if (_rxBuffer.size() > FRAGMENT_BUFFER_SIZE) {
                ESP_LOGE(TAG, "CMT2300A: Buffer full");
                _radio->flush_rx();
                break;
            }

            fragment_t f;
            memset(f.fragment, 0xcc, MAX_RF_PAYLOAD_SIZE);
            f.len = std::min<uint8_t>(_radio->getDynamicPayloadSize(), MAX_RF_PAYLOAD_SIZE);
            f.channel = _radio->getChannel();
            f.rssi = _radio->getRssiDBm();
            f.wasReceived = false;
            f.mainCmd = 0x00;
            _radio->read(f.fragment, f.len);
            _rxBuffer.push(f);
        }
        _radio->flush_rx();
        _packetReceived = false;
    }

    // Step 2: Process all buffered packets. Previously only one packet was
    // processed per loop() call, and only when no new packet was arriving.
    // Processing the entire buffer each iteration reduces latency and prevents
    // the software buffer from growing unboundedly during bursts.
    while (!_rxBuffer.empty()) {
        fragment_t f = _rxBuffer.back();
        if (checkFragmentCrc(f)) {

            // --- Capture Mode: log ALL valid frames before filtering ---
            if (_captureMode) {
                // Extract source inverter serial from fragment bytes [1..4]
                uint64_t srcSerial = 0;
                if (f.len > 4) {
                    srcSerial = (static_cast<uint64_t>(f.fragment[1]) << 24)
                        | (static_cast<uint64_t>(f.fragment[2]) << 16)
                        | (static_cast<uint64_t>(f.fragment[3]) << 8)
                        | (static_cast<uint64_t>(f.fragment[4]));
                }

                // Extract destination (DTU) serial from fragment bytes [5..8]
                uint64_t dstSerial = 0;
                if (f.len > 8) {
                    dstSerial = (static_cast<uint64_t>(f.fragment[5]) << 24)
                        | (static_cast<uint64_t>(f.fragment[6]) << 16)
                        | (static_cast<uint64_t>(f.fragment[7]) << 8)
                        | (static_cast<uint64_t>(f.fragment[8]));
                }

                ESP_LOGI(TAG, "CAPTURE %.2f MHz | %" PRId8 " dBm | src=%08" PRIx64 " dst=%08" PRIx64 " len=%u | %s",
                    getFrequencyFromChannel(f.channel) / 1000000.0,
                    f.rssi,
                    srcSerial,
                    dstSerial,
                    f.len,
                    Utils::dumpArray(f.fragment, f.len).c_str());
            }
            // --- End Capture Mode ---

            const serial_u dtuId = convertSerialToRadioId(_dtuSerial);

            // The CMT RF module does not filter foreign packages by itself.
            // Has to be done manually here.
            if (memcmp(&f.fragment[5], &dtuId.b[1], 4) == 0) {

                std::shared_ptr<InverterAbstract> inv = Hoymiles.getInverterByFragment(f);

                if (nullptr != inv) {
                    // Save packet in inverter rx buffer
                    ESP_LOGD(TAG, "RX %.2f MHz --> %s | %" PRId8 " dBm",
                        getFrequencyFromChannel(f.channel) / 1000000.0, Utils::dumpArray(f.fragment, f.len).c_str(), f.rssi);

                    inv->addRxFragment(f.fragment, f.len, f.rssi);

                    // Frequency hop to catch the next fragment from MIT inverters
                    if (_rxHopEnabled && f.len > 9) {
                        rxHopToNextFragment(f.fragment[9]);
                    }
                } else {
                    if (_captureMode) {
                        ESP_LOGI(TAG, "CAPTURE: Unknown inverter (not configured)");
                    } else {
                        ESP_LOGE(TAG, "Inverter Not found!");
                    }
                }
            } else if (_captureMode) {
                ESP_LOGI(TAG, "CAPTURE: Frame not addressed to this DTU (foreign traffic)");
            }

        } else {
            if (_captureMode) {
                ESP_LOGW(TAG, "CAPTURE: CRC failed | len=%u | %s",
                    f.len, Utils::dumpArray(f.fragment, f.len).c_str());
            } else {
                ESP_LOGW(TAG, "Frame kaputt"); // ;-)
            }
        }

        // Remove packet from buffer even if it was corrupted
        _rxBuffer.pop();
    }

    handleReceivedPackage();

    // After command completes (busyFlag cleared), disable RX hopping and
    // return to the base frequency so the next command starts clean
    if (_rxHopEnabled && !_busyFlag) {
        ESP_LOGI(TAG, "RX HOP: command complete, restoring base frequency");
        _rxHopEnabled = false;
        cmtSwitchDtuFreq(_inverterTargetFrequency);
    }
}

void HoymilesRadio_CMT::setPALevel(const int8_t paLevel)
{
    if (!_isInitialized) {
        return;
    }

    if (_radio->setPALevel(paLevel)) {
        ESP_LOGI(TAG, "CMT TX power set to %" PRId8 " dBm", paLevel);
    } else {
        ESP_LOGE(TAG, "CMT TX power %" PRId8 " dBm is not defined! (min: -10 dBm, max: 20 dBm)", paLevel);
    }
}

void HoymilesRadio_CMT::setInverterTargetFrequency(const uint32_t frequency)
{
    _inverterTargetFrequency = frequency;
    if (!_isInitialized) {
        return;
    }
    cmtSwitchDtuFreq(_inverterTargetFrequency);
}

uint32_t HoymilesRadio_CMT::getInverterTargetFrequency() const
{
    return _inverterTargetFrequency;
}

bool HoymilesRadio_CMT::isConnected() const
{
    if (!_isInitialized) {
        return false;
    }
    return _radio->isChipConnected();
}

uint32_t HoymilesRadio_CMT::getMinFrequency() const
{
    return countryDefinition.at(_countryMode).Freq_Min;
}

uint32_t HoymilesRadio_CMT::getMaxFrequency() const
{
    return countryDefinition.at(_countryMode).Freq_Max;
}

CountryModeId_t HoymilesRadio_CMT::getCountryMode() const
{
    return _countryMode;
}

void HoymilesRadio_CMT::setCountryMode(const CountryModeId_t mode)
{
    _countryMode = mode;
    if (!_isInitialized) {
        return;
    }
    _radio->setFrequencyBand(countryDefinition.at(mode).Band);
}

void HoymilesRadio_CMT::setCaptureMode(const bool enabled)
{
    _captureMode = enabled;
    if (enabled) {
        ESP_LOGI(TAG, "CMT2300A: Capture mode ENABLED - channel hopping active, logging all frames");
        ESP_LOGI(TAG, "CMT2300A: Dwell time: %" PRIu32 "ms, gpio3_configured: %s",
            CAPTURE_HOP_INTERVAL_MS, _gpio3_configured ? "yes" : "no");
        _captureChIdx = 0;
        _captureLastHop = 0;
    } else {
        ESP_LOGI(TAG, "CMT2300A: Capture mode DISABLED");
    }
}

bool HoymilesRadio_CMT::getCaptureMode() const
{
    return _captureMode;
}

int8_t HoymilesRadio_CMT::getHopOffsetForFragment(const uint8_t fragId)
{
    // MIT-5000-8T frequency hop pattern (confirmed by capture analysis):
    //   frag 1 → base - 1 channel (-250kHz)
    //   frag 2 → base
    //   frag 3 → base + 1 channel (+250kHz)
    //   frag 4 → base - 1 channel  (repeats)
    //   frag 5 → base
    //   frag 6 (0x86) → base + 1 channel
    // Pattern: offset = ((fragId - 1) % 3) - 1  → gives -1, 0, +1 repeating
    const uint8_t id = (fragId & 0x7F); // strip high bit (0x86 → 6)
    return static_cast<int8_t>((id - 1) % 3) - 1;
}

void HoymilesRadio_CMT::rxHopToNextFragment(const uint8_t receivedFragId)
{
    if (!_rxHopEnabled) {
        return;
    }

    _rxHopLastFragId = receivedFragId;
    _rxHopLastFragTime = millis();

    // Calculate next expected fragment ID
    const uint8_t nextFragId = (receivedFragId & 0x7F) + 1; // 0x86 means we're done, but hop anyway
    const int8_t nextOffset = getHopOffsetForFragment(nextFragId);
    const uint8_t nextChannel = static_cast<uint8_t>(static_cast<int8_t>(_rxHopBaseChannel) + nextOffset);

    const uint8_t currentChannel = _radio->getChannel();
    if (nextChannel != currentChannel) {
        // Use CMT2300A fast frequency hopping: single FH_CHANNEL register
        // write while staying in RX mode. This is the chip's native FH
        // mechanism (see AN197), avoiding the overhead of full state
        // transitions (stopListening→setChannel→startListening).
        _radio->setChannelFast(nextChannel);

        ESP_LOGD(TAG, "RX HOP: frag %" PRIu8 " received → fast hop to ch %" PRIu8 " (%.2f MHz) for next frag %" PRIu8,
            receivedFragId & 0x7F, nextChannel,
            getFrequencyFromChannel(nextChannel) / 1000000.0,
            nextFragId);
    }
}

uint32_t HoymilesRadio_CMT::getInvBootFrequency() const
{
    // Hoymiles boot/init frequency after power up inverter or connection lost for 15 min
    return countryDefinition.at(_countryMode).Freq_StartUp;
}

void ARDUINO_ISR_ATTR HoymilesRadio_CMT::handleInt1()
{
    _packetSent = true;
}

void ARDUINO_ISR_ATTR HoymilesRadio_CMT::handleInt2()
{
    _packetReceived = true;
}

void HoymilesRadio_CMT::sendEsbPacket(CommandAbstract& cmd)
{
    cmd.incrementSendCount();

    cmd.setRouterAddress(DtuSerial().u64);

    _radio->stopListening();

    if (cmd.getDataPayload()[0] == 0x56) { // @todo(tbnobody) Bad hack to identify ChannelChange Command
        cmtSwitchDtuFreq(getInvBootFrequency());
    }

    ESP_LOGD(TAG, "TX %s %.2f MHz --> %s",
        cmd.getCommandName().c_str(), getFrequencyFromChannel(_radio->getChannel()) / 1000000.0, cmd.dumpDataPayload().c_str());

    if (!_radio->write(cmd.getDataPayload(), cmd.getDataSize())) {
        ESP_LOGE(TAG, "TX SPI Timeout");
    }
    cmtSwitchDtuFreq(_inverterTargetFrequency);

    // Enable receive-side frequency hopping for MIT inverters (serial prefix 0x1520)
    // MIT hops response fragments across 3 frequencies: base-250kHz, base, base+250kHz
    const uint16_t prefix = (cmd.getTargetAddress() >> 32) & 0xffff;
    if (prefix == 0x1520 && cmd.getDataPayload()[0] != 0x56) { // not ChannelChange
        _rxHopEnabled = true;
        _rxHopBaseChannel = getChannelFromFrequency(_inverterTargetFrequency);
        _rxHopLastFragId = 0;
        _rxHopLastFragTime = millis();

        // Determine which fragment to expect first
        uint8_t expectedFirstFrag = 1;
        if (cmd.getDataPayload()[0] == 0x15 && cmd.getDataSize() >= 10) {
            // Retransmit request: payload[9] has fragment ID with bit 7 set
            expectedFirstFrag = cmd.getDataPayload()[9] & 0x7F;
        }

        const int8_t offset = getHopOffsetForFragment(expectedFirstFrag);
        const uint8_t firstChannel = static_cast<uint8_t>(static_cast<int8_t>(_rxHopBaseChannel) + offset);
        _radio->setChannel(firstChannel);

        ESP_LOGI(TAG, "RX HOP: MIT detected, expecting frag %" PRIu8 " on ch %" PRIu8 " (%.2f MHz), base ch %" PRIu8,
            expectedFirstFrag, firstChannel,
            getFrequencyFromChannel(firstChannel) / 1000000.0,
            _rxHopBaseChannel);
    } else {
        _rxHopEnabled = false;
    }

    _radio->startListening();
    _busyFlag = true;
    _rxTimeout.set(cmd.getTimeout());
}
