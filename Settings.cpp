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

#include "SoapyRTLTCP.hpp"
#include <SoapySDR/Time.hpp>
#include <algorithm>
#include <cstring>

#define DEFAULT_PORT "1234"

SoapyRTLTCP::SoapyRTLTCP(const SoapySDR::Kwargs &args):
    serverSocket(INVALID_SOCKET),
    rxFormat(RTL_RX_FORMAT_FLOAT32),
    tunerType(RTLSDR_TUNER_R820T),
    sampleRate(2048000),
    centerFrequency(100000000),
    bandwidth(0),
    ppm(0),
    directSamplingMode(0),
    iqSwap(false),
    gainMode(false),
    offsetMode(false),
    digitalAGC(false),
    biasTee(false),
    gainMin(0.0),
    gainMax(0.0)
{
    std::string rtltcp = args.count("rtltcp") ? args.at("rtltcp") : "";

    if (rtltcp.length())
        SoapySDR_logf(SOAPY_SDR_INFO, "RTL-TCP Opening %s...", rtltcp.c_str());

    serverSocket = connectToServer((char const *)rtltcp.c_str(), DEFAULT_PORT);
    if (serverSocket == INVALID_SOCKET)
    {
        if (rtltcp.length())
            SoapySDR_logf(SOAPY_SDR_INFO, "RTL-TCP connect failed");
        throw std::runtime_error(socket_strerror(SOCKET_ERRNO));
    }

    int hdr = recvHeader();
    if (hdr)
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "RTL-TCP header failed");
        throw std::runtime_error(socket_strerror(SOCKET_ERRNO));
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP Tuner type: %s", rtlTunerToString(tunerType).c_str());

    // Extract min/max overall gain range
    if (tunerGainCount > 0)
    {
        std::vector<int> gains = rtlTunerGains(tunerType);
        gainMin = *std::min_element(gains.begin(), gains.end()) / 10.0;
        gainMax = *std::max_element(gains.begin(), gains.end()) / 10.0;
    }

    // Start the network recv thread
    _buf_size        = BUFFER_SIZE;
    _buf             = new unsigned char[_buf_size];
    _buf_head        = 0;
    _buf_tail        = 0;
    _net_recv_thread = std::thread(&SoapyRTLTCP::net_recv_operation, this);
}

SoapyRTLTCP::~SoapyRTLTCP(void)
{
    closesocket(serverSocket); // aborts recv in network thread

    if (_net_recv_thread.joinable())
    {
        _net_recv_thread.join();
    }

    delete[] _buf;
}

/*******************************************************************
 * Network helper
 ******************************************************************/

int SoapyRTLTCP::recvHeader()
{
    char cmd[12] = {0};

    int left = sizeof(cmd);
    while (left > 0)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(serverSocket, &readfds);
        // worst case at 100ms for 12 bytes is 1.2 secs
        struct timeval tv = {0, 100000};

        int ret = select(serverSocket + 1, &readfds, NULL, NULL, &tv);
        if (ret)
        {
            ssize_t received = recv(serverSocket, (char *)&cmd + (sizeof(cmd) - left), left, 0);
            if (received < 0)
            {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "server recv error");
                break;
            }
            left -= received;
        }
    }

    if (cmd[0] != 'R' || cmd[1] != 'T' || cmd[2] != 'L' || cmd[3] != '0')
    {
        return -1;
    }

    tunerType = (rtlsdr_tuner)(((unsigned)cmd[4] << 24) | (cmd[5] << 16) | (cmd[6] << 8) | cmd[7]);
    tunerGainCount = ((unsigned)cmd[8] << 24) | (cmd[9] << 16) | (cmd[10] << 8) | cmd[11];

    return left;
}

int SoapyRTLTCP::sendCommand(rtltcpCommand command, unsigned int param)
{
    unsigned char cmd[5] = {(uint8_t)command, (uint8_t)(param >> 24), (uint8_t)(param >> 16), (uint8_t)(param >> 8), (uint8_t)(param >> 0)};

    int left = sizeof(cmd);
    while (left > 0)
    {
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(serverSocket, &writefds);
        // worst case at 100ms for 5 bytes is 0.5 secs
        struct timeval tv = {0, 100000};

        int ret = select(serverSocket + 1, NULL, &writefds, NULL, &tv);
        if (ret)
        {
            ssize_t sent = send(serverSocket, (char *)&cmd + (sizeof(cmd) - left), left, 0);
            if (sent < 0)
            {
                SoapySDR_logf(SOAPY_SDR_WARNING, "RTL-TCP server send error");
                break;
            }
            left -= sent;
        }
    }

    return left;
}

SOCKET SoapyRTLTCP::connectToServer(char const *serverName, char const *defaultPort)
{
    if (!serverName)
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "RTL-TCP Bad Address (%s)", serverName);
        return INVALID_SOCKET;
    }

    char host[INET6_ADDRSTRLEN] = {0};
    char port[NI_MAXSERV]       = {0};

    // Split IPv4, IPv6, or DNS name with optional port
    if (serverName[0] == '[')
    {
        // IPv6
        char const *endBracket = strchr(serverName, ']');
        if (!endBracket)
        {
            SoapySDR_logf(SOAPY_SDR_WARNING, "RTL-TCP Bad IPv6 Address (%s)", serverName);
            return INVALID_SOCKET;
        }
        size_t len = endBracket - serverName - 1;
        strncpy(host, serverName + 1, len);
        host[len] = '\0';

        if (serverName[len + 2] == ':')
        {
            strcpy(port, &serverName[len + 3]);
        }
    }
    else
    {
        // IPv4 or DNS name
        char const *lastColon = strrchr(serverName, ':');
        if (lastColon)
        {
            size_t len = lastColon - serverName;
            strncpy(host, serverName, len);
            host[len] = '\0';
            strcpy(port, lastColon + 1);
        }
        else
        {
            strcpy(host, serverName);
        }
    }
    // Default port if not specified
    if (!*port)
    {
        strcpy(port, defaultPort);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP Trying address %s port %s", host, port);

    struct addrinfo hints, *res, *res0;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags    = AI_ADDRCONFIG;
    int error         = getaddrinfo(host, port, &hints, &res0);
    if (error)
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "RTL-TCP Address error: %s", gai_strerror(error));
        return INVALID_SOCKET;
    }

    struct sockaddr_storage addr = {};
    unsigned addr_len            = sizeof(addr);
    SOCKET sock = INVALID_SOCKET;
    for (res = res0; res; res = res->ai_next)
    {
        // Try every resolved address
        sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (sock == INVALID_SOCKET)
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP Connection error: %s", socket_strerror(SOCKET_ERRNO));
            continue;
        }

        memset(&addr, 0, sizeof(addr));
        memcpy(&addr, res->ai_addr, res->ai_addrlen);
        addr_len = res->ai_addrlen;

        int err = getnameinfo((struct sockaddr *)&addr, addr_len,
                host, sizeof(host), port, sizeof(port), NI_NUMERICHOST | NI_NUMERICSERV);
        if (err != 0)
        {
            SoapySDR_logf(SOAPY_SDR_WARNING, "RTL-TCP failed to convert address to string (code=%d)", err);
        }

        // Try to connect
        int ret = connect(sock, (struct sockaddr *)&addr, addr_len);
        if (ret == 0)
        {
            break; // success
        }

        if (SOCKET_ERRNO == SOCKET_ECONNREFUSED)
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP Connection to %s port %s Refused", host, port);
            closesocket(serverSocket);
        }
        else
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP Connection to %s port %s Error: %s", host, port, socket_strerror(SOCKET_ERRNO));
        }
        sock = INVALID_SOCKET;
    }
    freeaddrinfo(res0);
    if (sock == INVALID_SOCKET)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP Server not found");
        return INVALID_SOCKET;
    }

    SoapySDR_logf(SOAPY_SDR_INFO, "RTL-TCP Connected to server %s port %s", host, port);
    address = std::string(host) + ":" + std::string(port);
    serverSocket = sock;

    return serverSocket;
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyRTLTCP::getDriverKey(void) const
{
    return "RTLTCP";
}

std::string SoapyRTLTCP::getHardwareKey(void) const
{
    switch (tunerType)
    {
    case RTLSDR_TUNER_UNKNOWN:
        return "UNKNOWN";
    case RTLSDR_TUNER_E4000:
        return "E4000";
    case RTLSDR_TUNER_FC0012:
        return "FC0012";
    case RTLSDR_TUNER_FC0013:
        return "FC0013";
    case RTLSDR_TUNER_FC2580:
        return "FC2580";
    case RTLSDR_TUNER_R820T:
        return "R820T";
    case RTLSDR_TUNER_R828D:
        return "R828D";
    default:
        return "OTHER";
    }
}

SoapySDR::Kwargs SoapyRTLTCP::getHardwareInfo(void) const
{
    // key/value pairs for any useful information
    // this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["origin"] = "https://github.com/pothosware/SoapyRTLTCP";

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyRTLTCP::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

bool SoapyRTLTCP::getFullDuplex(const int /*direction*/, const size_t /*channel*/) const
{
    return false;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyRTLTCP::listAntennas(const int /*direction*/, const size_t /*channel*/) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    return antennas;
}

void SoapyRTLTCP::setAntenna(const int direction, const size_t, const std::string &)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("setAntena failed: RTL-TCP only supports RX");
    }
}

std::string SoapyRTLTCP::getAntenna(const int /*direction*/, const size_t /*channel*/) const
{
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyRTLTCP::hasDCOffsetMode(const int /*direction*/, const size_t /*channel*/) const
{
    return false;
}

bool SoapyRTLTCP::hasFrequencyCorrection(const int /*direction*/, const size_t /*channel*/) const
{
    return true;
}

void SoapyRTLTCP::setFrequencyCorrection(const int /*direction*/, const size_t /*channel*/, const double value)
{
    int r = sendCommand(RTLTCP_SET_FREQ_CORRECTION, int(value));
    if (r != 0)
    {
        throw std::runtime_error("setFrequencyCorrection failed");
    }
    ppm = (int)value;
}

double SoapyRTLTCP::getFrequencyCorrection(const int /*direction*/, const size_t /*channel*/) const
{
    return double(ppm);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyRTLTCP::listGains(const int /*direction*/, const size_t /*channel*/) const
{
    // list available gain elements,
    // the functions below have a "name" parameter
    std::vector<std::string> results;

    if (tunerType == RTLSDR_TUNER_E4000)
    {
        results.push_back("IF1");
        results.push_back("IF2");
        results.push_back("IF3");
        results.push_back("IF4");
        results.push_back("IF5");
        results.push_back("IF6");
    }
    results.push_back("TUNER");

    return results;
}

bool SoapyRTLTCP::hasGainMode(const int /*direction*/, const size_t /*channel*/) const
{
    return true;
}

void SoapyRTLTCP::setGainMode(const int /*direction*/, const size_t /*channel*/, const bool automatic)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting RTL-TCP gain mode: %s", automatic ? "Automatic" : "Manual");
    int r = sendCommand(RTLTCP_SET_GAIN_MODE, gainMode ? 0 : 1);
    if (r != 0)
    {
        throw std::runtime_error("setFrequencyCorrection failed");
    }
    gainMode = automatic;
}

bool SoapyRTLTCP::getGainMode(const int /*direction*/, const size_t /*channel*/) const
{
    return gainMode;
}

void SoapyRTLTCP::setGain(const int /*direction*/, const size_t /*channel*/, const double value)
{
    int r = sendCommand(RTLTCP_SET_GAIN, int(value));
    if (r != 0)
    {
        throw std::runtime_error("setGain failed");
    }
}

void SoapyRTLTCP::setGain(const int /*direction*/, const size_t /*channel*/, const std::string &name, const double value)
{
    if ((name.length() >= 2) && (name.substr(0, 2) == "IF"))
    {
        int stage = 1;
        if (name.length() > 2)
        {
            int stage_in = name.at(2) - '0';
            if ((stage_in < 1) || (stage_in > 6))
            {
                throw std::runtime_error("Invalid IF stage, 1 or 1-6 for E4000");
            }
        }
        if (tunerType == RTLSDR_TUNER_E4000)
        {
            IFGain[stage - 1] = getE4000Gain(stage, (int)value);
        }
        else
        {
            IFGain[stage - 1] = value;
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting RTL-TCP IF Gain for stage %d: %f", stage, IFGain[stage - 1]);

        int r = sendCommand(RTLTCP_SET_IF_TUNER_GAIN, (stage << 16) | int(IFGain[stage - 1]));
        if (r != 0)
        {
            throw std::runtime_error("setGain failed");
        }
    }

    if (name == "TUNER")
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting RTL-TCP Tuner Gain: %f", tunerGain);

        int r = sendCommand(RTLTCP_SET_GAIN, int(value * 10.0));
        if (r != 0)
        {
            throw std::runtime_error("setGain failed");
        }
        tunerGain = value;
    }
}

double SoapyRTLTCP::getGain(const int /*direction*/, const size_t /*channel*/, const std::string &name) const
{
    if ((name.length() >= 2) && (name.substr(0, 2) == "IF"))
    {
        int stage = 1;
        if (name.length() > 2)
        {
            int stage_in = name.at(2) - '0';
            if ((stage_in < 1) || (stage_in > 6))
            {
                throw std::runtime_error("Invalid IF stage, 1 or 1-6 for E4000");
            }
            else
            {
                stage = stage_in;
            }
        }
        if (tunerType == RTLSDR_TUNER_E4000)
        {
            return getE4000Gain(stage, IFGain[stage - 1]);
        }

        return IFGain[stage - 1];
    }

    if (name == "TUNER")
    {
        return tunerGain;
    }

    return 0;
}

SoapySDR::Range SoapyRTLTCP::getGainRange(const int /*direction*/, const size_t /*channel*/, const std::string &name) const
{
    if (tunerType == RTLSDR_TUNER_E4000 && name != "TUNER")
    {
        if (name == "IF1")
        {
            return SoapySDR::Range(-3, 6);
        }
        if (name == "IF2" || name == "IF3")
        {
            return SoapySDR::Range(0, 9);
        }
        if (name == "IF4")
        {
            return SoapySDR::Range(0, 2);
        }
        if (name == "IF5" || name == "IF6")
        {
            return SoapySDR::Range(3, 15);
        }

        return SoapySDR::Range(gainMin, gainMax);
    }
    else
    {
        return SoapySDR::Range(gainMin, gainMax);
    }
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyRTLTCP::setFrequency(
        const int /*direction*/,
        const size_t /*channel*/,
        const std::string &name,
        const double frequency,
        const SoapySDR::Kwargs &/*args*/)
{
    if (name == "RF")
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting center freq: %d", (uint32_t)frequency);
        int r = sendCommand(RTLTCP_SET_FREQ, (uint32_t)frequency);
        if (r != 0)
        {
            throw std::runtime_error("setFrequency failed");
        }
        centerFrequency = (uint32_t)frequency;
    }

    if (name == "CORR")
    {
        int r = sendCommand(RTLTCP_SET_FREQ_CORRECTION, (int)frequency);
        if (r != 0)
        {
            throw std::runtime_error("setFrequencyCorrection failed");
        }
        ppm = (int)frequency;
    }
}

double SoapyRTLTCP::getFrequency(const int /*direction*/, const size_t /*channel*/, const std::string &name) const
{
    if (name == "RF")
    {
        return (double)centerFrequency;
    }

    if (name == "CORR")
    {
        return (double)ppm;
    }

    return 0;
}

std::vector<std::string> SoapyRTLTCP::listFrequencies(const int /*direction*/, const size_t /*channel*/) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    names.push_back("CORR");
    return names;
}

SoapySDR::RangeList SoapyRTLTCP::getFrequencyRange(
        const int /*direction*/,
        const size_t /*channel*/,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        if (tunerType == RTLSDR_TUNER_E4000)
        {
            results.push_back(SoapySDR::Range(52000000, 2200000000));
        }
        else if (tunerType == RTLSDR_TUNER_FC0012)
        {
            results.push_back(SoapySDR::Range(22000000, 1100000000));
        }
        else if (tunerType == RTLSDR_TUNER_FC0013)
        {
            results.push_back(SoapySDR::Range(22000000, 948600000));
        }
        else
        {
            results.push_back(SoapySDR::Range(24000000, 1764000000));
        }
    }
    if (name == "CORR")
    {
        results.push_back(SoapySDR::Range(-1000, 1000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyRTLTCP::getFrequencyArgsInfo(const int /*direction*/, const size_t /*channel*/) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyRTLTCP::setSampleRate(const int /*direction*/, const size_t /*channel*/, const double rate)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %d", (uint32_t)rate);
    int r = sendCommand(RTLTCP_SET_SAMPLE_RATE, (uint32_t)rate);
    if (r != 0)
    {
        throw std::runtime_error("setSampleRate failed");
    }
    sampleRate = (uint32_t)rate;
}

double SoapyRTLTCP::getSampleRate(const int /*direction*/, const size_t /*channel*/) const
{
    return sampleRate;
}

std::vector<double> SoapyRTLTCP::listSampleRates(const int /*direction*/, const size_t /*channel*/) const
{
    std::vector<double> results;

    results.push_back(250000);
    results.push_back(1024000);
    results.push_back(1536000);
    results.push_back(1792000);
    results.push_back(1920000);
    results.push_back(2048000);
    results.push_back(2160000);
    results.push_back(2560000);
    results.push_back(2880000);
    results.push_back(3200000);

    return results;
}

SoapySDR::RangeList SoapyRTLTCP::getSampleRateRange(const int /*direction*/, const size_t /*channel*/) const
{
    SoapySDR::RangeList results;

    results.push_back(SoapySDR::Range(225001, 300000));
    results.push_back(SoapySDR::Range(900001, 3200000));

    return results;
}

void SoapyRTLTCP::setBandwidth(const int /*direction*/, const size_t /*channel*/, const double bw)
{
    int r = 0; // TODO: rtl_tcp has no concept of bandwidth
    if (r != 0)
    {
        throw std::runtime_error("setBandwidth failed");
    }
    bandwidth = bw;
}

double SoapyRTLTCP::getBandwidth(const int /*direction*/, const size_t /*channel*/) const
{
    if (bandwidth == 0) // auto / full bandwidth
        return sampleRate;
    return bandwidth;
}

std::vector<double> SoapyRTLTCP::listBandwidths(const int /*direction*/, const size_t /*channel*/) const
{
    std::vector<double> results;

    return results;
}

SoapySDR::RangeList SoapyRTLTCP::getBandwidthRange(const int /*direction*/, const size_t /*channel*/) const
{
    SoapySDR::RangeList results;

    // stub, not sure what the sensible ranges for different tuners are.
    results.push_back(SoapySDR::Range(0, 8000000));

    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyRTLTCP::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    SoapySDR::ArgInfo directSampArg;

    directSampArg.key = "direct_samp";
    directSampArg.value = "0";
    directSampArg.name = "Direct Sampling";
    directSampArg.description = "RTL-TCP Direct Sampling Mode";
    directSampArg.type = SoapySDR::ArgInfo::STRING;
    directSampArg.options.push_back("0");
    directSampArg.optionNames.push_back("Off");
    directSampArg.options.push_back("1");
    directSampArg.optionNames.push_back("I-ADC");
    directSampArg.options.push_back("2");
    directSampArg.optionNames.push_back("Q-ADC");

    setArgs.push_back(directSampArg);

    SoapySDR::ArgInfo offsetTuneArg;

    offsetTuneArg.key = "offset_tune";
    offsetTuneArg.value = "false";
    offsetTuneArg.name = "Offset Tune";
    offsetTuneArg.description = "RTL-TCP Offset Tuning Mode";
    offsetTuneArg.type = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(offsetTuneArg);

    SoapySDR::ArgInfo iqSwapArg;

    iqSwapArg.key = "iq_swap";
    iqSwapArg.value = "false";
    iqSwapArg.name = "I/Q Swap";
    iqSwapArg.description = "RTL-TCP I/Q Swap Mode";
    iqSwapArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(iqSwapArg);

    SoapySDR::ArgInfo digitalAGCArg;

    digitalAGCArg.key = "digital_agc";
    digitalAGCArg.value = "false";
    digitalAGCArg.name = "Digital AGC";
    digitalAGCArg.description = "RTL-TCP digital AGC Mode";
    digitalAGCArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(digitalAGCArg);

    SoapySDR::ArgInfo biasTeeArg;

    biasTeeArg.key = "biastee";
    biasTeeArg.value = "false";
    biasTeeArg.name = "Bias Tee";
    biasTeeArg.description = "RTL-TCP Blog V.3 Bias-Tee Mode";
    biasTeeArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(biasTeeArg);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "SETARGS?");

    return setArgs;
}

void SoapyRTLTCP::writeSetting(const std::string &key, const std::string &value)
{
    int r = 0;

    if (key == "direct_samp")
    {
        try
        {
            directSamplingMode = std::stoi(value);
        }
        catch (const std::invalid_argument &)
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "RTL-TCP invalid direct sampling mode '%s', [0:Off, 1:I-ADC, 2:Q-ADC]", value.c_str());
            directSamplingMode = 0;
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP direct sampling mode: %d", directSamplingMode);
        r = sendCommand(RTLTCP_SET_DIRECT_SAMPLING, directSamplingMode);
    }
    else if (key == "iq_swap")
    {
        iqSwap = ((value == "true") ? true : false);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP I/Q swap: %s", iqSwap ? "true" : "false");
    }
    else if (key == "offset_tune")
    {
        offsetMode = (value == "true") ? true : false;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP offset_tune mode: %s", offsetMode ? "true" : "false");
        r = sendCommand(RTLTCP_SET_OFFSET_TUNING, offsetMode ? 1 : 0);
    }
    else if (key == "digital_agc")
    {
        digitalAGC = (value == "true") ? true : false;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP digital agc mode: %s", digitalAGC ? "true" : "false");
        r = sendCommand(RTLTCP_SET_AGC_MODE, digitalAGC ? 1 : 0);
    }
    else if (key == "biastee")
    {
        biasTee = (value == "true") ? true : false;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP bias tee mode: %s", biasTee ? "true" : "false");
        r = sendCommand(RTLTCP_SET_BIAS_TEE, biasTee ? 1 : 0);
    }

    if (r != 0)
    {
        throw std::runtime_error("writeSetting failed");
    }
}

std::string SoapyRTLTCP::readSetting(const std::string &key) const
{
    if (key == "direct_samp")
    {
        return std::to_string(directSamplingMode);
    }
    else if (key == "iq_swap")
    {
        return iqSwap ? "true" : "false";
    }
    else if (key == "offset_tune")
    {
        return offsetMode ? "true" : "false";
    }
    else if (key == "digital_agc")
    {
        return digitalAGC ? "true" : "false";
    }
    else if (key == "biastee")
    {
        return biasTee ? "true" : "false";
    }

    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}

std::string SoapyRTLTCP::rtlTunerToString(rtlsdr_tuner tunerType)
{
    std::string deviceTuner;
    switch (tunerType)
    {
    case RTLSDR_TUNER_UNKNOWN:
        deviceTuner = "Unknown";
        break;
    case RTLSDR_TUNER_E4000:
        deviceTuner = "Elonics E4000";
        break;
    case RTLSDR_TUNER_FC0012:
        deviceTuner = "Fitipower FC0012";
        break;
    case RTLSDR_TUNER_FC0013:
        deviceTuner = "Fitipower FC0013";
        break;
    case RTLSDR_TUNER_FC2580:
        deviceTuner = "Fitipower FC2580";
        break;
    case RTLSDR_TUNER_R820T:
        deviceTuner = "Rafael Micro R820T";
        break;
    case RTLSDR_TUNER_R828D:
        deviceTuner = "Rafael Micro R828D";
        break;
    default:
        deviceTuner = "Unknown";
    }
    return deviceTuner;
}

std::vector<int> SoapyRTLTCP::rtlTunerGains(rtlsdr_tuner tunerType)
{
    /* all gain values are expressed in tenths of a dB */
    std::vector<int> e4k_gains{-10, 15, 40, 65, 90, 115, 140, 165, 190, 215,
            240, 290, 340, 420};
    std::vector<int> fc0012_gains{-99, -40, 71, 179, 192};
    std::vector<int> fc0013_gains{-99, -73, -65, -63, -60, -58, -54, 58, 61,
            63, 65, 67, 68, 70, 71, 179, 181, 182,
            184, 186, 188, 191, 197};
    std::vector<int> fc2580_gains{0 /* no gain values */};
    std::vector<int> r82xx_gains{0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
            166, 197, 207, 229, 254, 280, 297, 328,
            338, 364, 372, 386, 402, 421, 434, 439,
            445, 480, 496};
    std::vector<int> unknown_gains{0 /* no gain values */};

    switch (tunerType)
    {
    case RTLSDR_TUNER_E4000:
        return e4k_gains;
    case RTLSDR_TUNER_FC0012:
        return fc0012_gains;
    case RTLSDR_TUNER_FC0013:
        return fc0013_gains;
    case RTLSDR_TUNER_FC2580:
        return fc2580_gains;
    case RTLSDR_TUNER_R820T:
    case RTLSDR_TUNER_R828D:
        return r82xx_gains;
    default:
        return unknown_gains;
	}
}

int SoapyRTLTCP::getE4000Gain(int stage, int gain) {
    static const int8_t if_stage1_gain[] = {
            -3, 6
    };

    static const int8_t if_stage23_gain[] = {
            0, 3, 6, 9
    };

    static const int8_t if_stage4_gain[] = {
            0, 1, 2 //, 2
    };

    static const int8_t if_stage56_gain[] = {
            3, 6, 9, 12, 15 // , 15, 15, 15 // wat?
    };

    const int8_t *if_stage = nullptr;
    int n_gains = 0;

    if (stage == 1) {
        if_stage = if_stage1_gain;
        n_gains = 2;
    } else if (stage == 2 || stage == 3) {
        if_stage = if_stage23_gain;
        n_gains = 4;
    } else if (stage == 4) {
        if_stage = if_stage4_gain;
        n_gains = 3;
    } else if (stage == 5 || stage == 6) {
        if_stage = if_stage56_gain;
        n_gains = 5;
    }

    if (n_gains && if_stage) {
        int gainMin = if_stage[0];
        int gainMax = if_stage[n_gains-1];

        if (gain > gainMax) {
            gain = gainMax;
        }

        if (gain < gainMin) {
            gain = gainMin;
        }

        for (int i = 0; i < n_gains-1; i++) {
            if (gain >= if_stage[i] && gain <= if_stage[i+1]) {
                gain = ((gain-if_stage[i]) < (if_stage[i+1]-gain))?if_stage[i]:if_stage[i+1];
            }
        }
    }

    return gain;
}

rtlsdr_tuner SoapyRTLTCP::rtlStringToTuner(std::string tunerType)
{
    rtlsdr_tuner deviceTuner = RTLSDR_TUNER_UNKNOWN;

    deviceTuner = RTLSDR_TUNER_UNKNOWN;

    if (tunerType == "Elonics E4000")
        deviceTuner = RTLSDR_TUNER_E4000;
    if (tunerType == "Fitipower FC0012")
        deviceTuner = RTLSDR_TUNER_FC0012;
    if (tunerType == "Fitipower FC0013")
        deviceTuner = RTLSDR_TUNER_FC0013;
    if (tunerType == "Fitipower FC2580")
        deviceTuner = RTLSDR_TUNER_FC2580;
    if (tunerType == "Rafael Micro R820T")
        deviceTuner = RTLSDR_TUNER_R820T;
    if (tunerType == "Rafael Micro R828D")
        deviceTuner = RTLSDR_TUNER_R828D;

    return deviceTuner;
}
