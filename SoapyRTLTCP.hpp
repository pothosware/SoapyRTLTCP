/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Christian Zuckschwerdt <zany@triq.net>

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.h>
#include <SoapySDR/Types.h>
#include "SocketDefs.h"
#include <stdexcept>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

struct SoapyRTLTCP_SocketInit
{
    SoapyRTLTCP_SocketInit(void)
    {
#ifdef _MSC_VER
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }
    ~SoapyRTLTCP_SocketInit(void)
    {
#ifdef _MSC_VER
        WSACleanup();
#endif
    }
};

typedef enum rtltcpRXFormat
{
    RTL_RX_FORMAT_FLOAT32,
    RTL_RX_FORMAT_INT16,
    RTL_RX_FORMAT_INT8,
    RTL_RX_FORMAT_UINT8,
} rtltcpRXFormat;

#define DEFAULT_BUFFER_LENGTH (16 * 32 * 512)
#define DEFAULT_NUM_BUFFERS 15
#define BYTES_PER_SAMPLE 2
#define BUFFER_SIZE (DEFAULT_BUFFER_LENGTH * DEFAULT_NUM_BUFFERS)

enum rtlsdr_tuner
{
    RTLSDR_TUNER_UNKNOWN = 0,
    RTLSDR_TUNER_E4000,
    RTLSDR_TUNER_FC0012,
    RTLSDR_TUNER_FC0013,
    RTLSDR_TUNER_FC2580,
    RTLSDR_TUNER_R820T,
    RTLSDR_TUNER_R828D
};

enum rtltcpCommand
{
    RTLTCP_SET_FREQ             = 0x01,
    RTLTCP_SET_SAMPLE_RATE      = 0x02,
    RTLTCP_SET_GAIN_MODE        = 0x03,
    RTLTCP_SET_GAIN             = 0x04,
    RTLTCP_SET_FREQ_CORRECTION  = 0x05,
    RTLTCP_SET_IF_TUNER_GAIN    = 0x06,
    RTLTCP_SET_TEST_MODE        = 0x07,
    RTLTCP_SET_AGC_MODE         = 0x08,
    RTLTCP_SET_DIRECT_SAMPLING  = 0x09,
    RTLTCP_SET_OFFSET_TUNING    = 0x0a,
    RTLTCP_SET_RTL_XTAL         = 0x0b,
    RTLTCP_SET_TUNER_XTAL       = 0x0c,
    RTLTCP_SET_TUNER_GAIN_BY_ID = 0x0d,
    RTLTCP_SET_BIAS_TEE         = 0x0e,
};

class SoapyRTLTCP : public SoapySDR::Device
{
public:
    SoapyRTLTCP(const SoapySDR::Kwargs &args);

    ~SoapyRTLTCP(void);

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const;

    std::string getHardwareKey(void) const;

    SoapySDR::Kwargs getHardwareInfo(void) const;

    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels(const int) const;

    bool getFullDuplex(const int direction, const size_t channel) const;

    /*******************************************************************
     * Stream API
     ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

    SoapySDR::Stream *setupStream(
            const int direction,
            const std::string &format,
            const std::vector<size_t> &channels = std::vector<size_t>(),
            const SoapySDR::Kwargs &args        = SoapySDR::Kwargs());

    void closeStream(SoapySDR::Stream *stream);

    size_t getStreamMTU(SoapySDR::Stream *stream) const;

    int activateStream(
            SoapySDR::Stream *stream,
            const int flags        = 0,
            const long long timeNs = 0,
            const size_t numElems  = 0);

    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0);

    int readStream(
            SoapySDR::Stream *stream,
            void *const *buffs,
            const size_t numElems,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000);

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;

    void setAntenna(const int direction, const size_t channel, const std::string &name);

    std::string getAntenna(const int direction, const size_t channel) const;

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    bool hasDCOffsetMode(const int direction, const size_t channel) const;

    bool hasFrequencyCorrection(const int direction, const size_t channel) const;

    void setFrequencyCorrection(const int direction, const size_t channel, const double value);

    double getFrequencyCorrection(const int direction, const size_t channel) const;

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const;

    bool hasGainMode(const int direction, const size_t channel) const;

    void setGainMode(const int direction, const size_t channel, const bool automatic);

    bool getGainMode(const int direction, const size_t channel) const;

    void setGain(const int direction, const size_t channel, const double value);

    void setGain(const int direction, const size_t channel, const std::string &name, const double value);

    double getGain(const int direction, const size_t channel, const std::string &name) const;

    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency(
            const int direction,
            const size_t channel,
            const std::string &name,
            const double frequency,
            const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

    double getFrequency(const int direction, const size_t channel, const std::string &name) const;

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;

    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    void setSampleRate(const int direction, const size_t channel, const double rate);

    double getSampleRate(const int direction, const size_t channel) const;

    std::vector<double> listSampleRates(const int direction, const size_t channel) const;

    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;

    void setBandwidth(const int direction, const size_t channel, const double bw);

    double getBandwidth(const int direction, const size_t channel) const;

    std::vector<double> listBandwidths(const int direction, const size_t channel) const;

    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const;

    /*******************************************************************
     * Utility
     ******************************************************************/

    static std::string rtlTunerToString(rtlsdr_tuner tunerType);
    static rtlsdr_tuner rtlStringToTuner(std::string tunerType);
    static std::vector<int> rtlTunerGains(rtlsdr_tuner tunerType);
    static int getE4000Gain(int stage, int gain);

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const;

    void writeSetting(const std::string &key, const std::string &value);

    std::string readSetting(const std::string &key) const;

private:
    SOCKET connectToServer(char const *serverName, char const *defaultPort);

    int recvHeader();

    int sendCommand(rtltcpCommand command, unsigned int param);

    SoapyRTLTCP_SocketInit socket_init;

    // network handle
    SOCKET serverSocket;

    // network recv stream
    std::thread _net_recv_thread;
    void net_recv_operation(void);

    // One big circular buffer
    unsigned char *_buf;
    size_t _buf_size;
    std::atomic<size_t> _buf_head;
    std::atomic<size_t> _buf_tail;
    std::atomic<bool> _buf_active;
    std::atomic<bool> _overflowEvent;

    std::mutex _buf_mutex;
    std::condition_variable _buf_cond;

    // cached settings
    rtltcpRXFormat rxFormat;
    rtlsdr_tuner tunerType;
    int tunerGainCount;
    uint32_t sampleRate, centerFrequency;
    double bandwidth;
    int ppm, directSamplingMode;
    bool iqSwap, gainMode, offsetMode, digitalAGC, biasTee;
    double IFGain[6], tunerGain;
    double gainMin, gainMax;

    std::vector<std::complex<float>> _lut_32f;
    std::vector<std::complex<float>> _lut_swap_32f;
    std::vector<std::complex<int16_t>> _lut_16i;
    std::vector<std::complex<int16_t>> _lut_swap_16i;

public:
    std::string address;
};
