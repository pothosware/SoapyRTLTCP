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
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <algorithm> //min
#include <climits>   //SHRT_MAX
#include <cstring>   // memcpy

std::vector<std::string> SoapyRTLTCP::getStreamFormats(const int /*direction*/, const size_t /*channel*/) const
{
    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CU8);
    formats.push_back(SOAPY_SDR_CS8);
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);

    return formats;
}

std::string SoapyRTLTCP::getNativeStreamFormat(const int direction, const size_t /*channel*/, double &fullScale) const
{
    // Check that direction is SOAPY_SDR_RX
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("RTL-TCP is RX only, use SOAPY_SDR_RX");
    }

    fullScale = 128;
    return SOAPY_SDR_CS8;
}

SoapySDR::ArgInfoList SoapyRTLTCP::getStreamArgsInfo(const int direction, const size_t /*channel*/) const
{
    // Check that direction is SOAPY_SDR_RX
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("RTL-TCP is RX only, use SOAPY_SDR_RX");
    }

    SoapySDR::ArgInfoList streamArgs;

    return streamArgs;
}

/*******************************************************************
 * Network Recv Worker
 ******************************************************************/

void SoapyRTLTCP::net_recv_operation(void)
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP net_recv_operation");

    // Uses _buf_tail for a circular buffer.
    // A chunk_size area behind _buf_tail is active, a concurrent
    // read will be inconsistent. Just flags overflow should we
    // overtake _buf_head, can't really do anything else.
    for (;;)
    {
        size_t tail = _buf_tail;
        // Wrap write position if needed
        if (tail == _buf_size)
            tail = 0;

        // Clip to some maximum write size
        size_t buf_left   = _buf_size - tail;
        size_t chunk_size = std::min(buf_left, (size_t)DEFAULT_BUFFER_LENGTH);

        // Check overflow condition: the caller is not reading fast enough
        size_t chunk_end = tail + chunk_size;
        size_t head      = _buf_head;
        if (_buf_active && head > tail && head <= chunk_end)
        {
            _overflowEvent = true;
        }

        // Block on recv
        ssize_t received = recv(serverSocket, _buf + tail, chunk_size, 0);
        if (received <= 0)
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP server recv end");
            break;
        }
        _buf_tail = tail + received;

        // Notify readStream()
        _buf_cond.notify_one();
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP net_recv_operation done");
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyRTLTCP::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs & /*args*/)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("RTL-TCP is RX only, use SOAPY_SDR_RX");
    }

    // Check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    // Check the format
    if (format == SOAPY_SDR_CF32)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
        rxFormat = RTL_RX_FORMAT_FLOAT32;
    }
    else if (format == SOAPY_SDR_CS16)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
        rxFormat = RTL_RX_FORMAT_INT16;
    }
    else if (format == SOAPY_SDR_CS8)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
        rxFormat = RTL_RX_FORMAT_INT8;
    }
    else if (format == SOAPY_SDR_CU8)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CU8.");
        rxFormat = RTL_RX_FORMAT_UINT8;
    }
    else
    {
        throw std::runtime_error(
                "setupStream invalid format '" + format + "' -- Only CU8, CS8, CS16, and CF32 are supported by SoapyRTLTCP module.");
    }

    if (rxFormat != RTL_RX_FORMAT_INT8 && !_lut_32f.size())
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Generating RTL-TCP lookup tables");
        // Create lookup tables
        for (unsigned int i = 0; i <= 0xffff; i++)
        {
#if (__BYTE_ORDER == __LITTLE_ENDIAN)
            float re = ((i & 0xff) - 127.4f) * (1.0f / 128.0f);
            float im = ((i >> 8) - 127.4f) * (1.0f / 128.0f);
#else
            float re = ((i >> 8) - 127.4f) * (1.0f / 128.0f);
            float im = ((i & 0xff) - 127.4f) * (1.0f / 128.0f);
#endif

            std::complex<float> v32f, vs32f;

            v32f.real(re);
            v32f.imag(im);
            _lut_32f.push_back(v32f);

            vs32f.real(v32f.imag());
            vs32f.imag(v32f.real());
            _lut_swap_32f.push_back(vs32f);

            std::complex<int16_t> v16i, vs16i;

            v16i.real(int16_t((float(SHRT_MAX) * re)));
            v16i.imag(int16_t((float(SHRT_MAX) * im)));
            _lut_16i.push_back(v16i);

            vs16i.real(vs16i.imag());
            vs16i.imag(vs16i.real());
            _lut_swap_16i.push_back(vs16i);
        }
    }

    if (tunerType == RTLSDR_TUNER_E4000)
    {
        IFGain[0] = 6;
        IFGain[1] = 9;
        IFGain[2] = 3;
        IFGain[3] = 2;
        IFGain[4] = 3;
        IFGain[5] = 3;
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            IFGain[i] = 0;
        }
    }

    return (SoapySDR::Stream *)this;
}

void SoapyRTLTCP::closeStream(SoapySDR::Stream *stream)
{
    this->deactivateStream(stream, 0, 0);
}

size_t SoapyRTLTCP::getStreamMTU(SoapySDR::Stream * /*stream*/) const
{
    return DEFAULT_BUFFER_LENGTH / BYTES_PER_SAMPLE;
}

int SoapyRTLTCP::activateStream(
        SoapySDR::Stream * /*stream*/,
        const int flags,
        const long long /*timeNs*/,
        const size_t /*numElems*/)
{
    if (flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;

    // Reset buffer
    size_t tail    = _buf_tail;
    _buf_head      = tail;
    _buf_active    = true;
    _overflowEvent = false;

    return 0;
}

int SoapyRTLTCP::deactivateStream(SoapySDR::Stream * /*stream*/, const int flags, const long long /*timeNs*/)
{
    if (flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;

    _buf_active = false;

    return 0;
}

int SoapyRTLTCP::readStream(
        SoapySDR::Stream * /*stream*/,
        void *const *buffs,
        const size_t numElems,
        int &flags,
        long long & /*timeNs*/,
        const long timeoutUs)
{
    // This is the user's buffer for channel 0
    void *buff0 = buffs[0];

    if (_overflowEvent)
    {
        // We automatically skipped one full buffer length ahead
        _overflowEvent = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        flags |= SOAPY_SDR_OVERFLOW;
    }

    // Wait for a buffer to become available
    if (_buf_head == _buf_tail)
    {
        std::unique_lock<std::mutex> lock(_buf_mutex);
        _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]
                { return _buf_head != _buf_tail; });
        if (_buf_head == _buf_tail)
            return SOAPY_SDR_TIMEOUT;
    }

    size_t head = _buf_head;
    size_t tail = _buf_tail;
    if (head >= _buf_size)
        head = 0;
    unsigned char *buf = _buf + head;
    size_t buf_len     = tail > head ? tail - head : _buf_size - head;

    size_t returnedElems = std::min(buf_len / BYTES_PER_SAMPLE, numElems);

    // SoapySDR_logf(SOAPY_SDR_INFO, "RTL-TCP read %zu of %zu _buf_head=%zu, _buf_tail=%zu",
    //         returnedElems, numElems, head, tail);

    // Convert into user's buff0
    if (rxFormat == RTL_RX_FORMAT_UINT8)
    {
        uint8_t *itarget = (uint8_t *)buff0;
        if (iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                itarget[i * 2]     = buf[i * 2 + 1];
                itarget[i * 2 + 1] = buf[i * 2];
            }
        }
        else
        {
            std::memcpy(itarget, buf, returnedElems * BYTES_PER_SAMPLE);
        }
    }
    else if (rxFormat == RTL_RX_FORMAT_FLOAT32)
    {
        float *ftarget = (float *)buff0;
        std::complex<float> tmp;
        if (iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp                = _lut_swap_32f[*((uint16_t *)&buf[2 * i])];
                ftarget[i * 2]     = tmp.real();
                ftarget[i * 2 + 1] = tmp.imag();
            }
        }
        else
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp                = _lut_32f[*((uint16_t *)&buf[2 * i])];
                ftarget[i * 2]     = tmp.real();
                ftarget[i * 2 + 1] = tmp.imag();
            }
        }
    }
    else if (rxFormat == RTL_RX_FORMAT_INT16)
    {
        int16_t *itarget = (int16_t *)buff0;
        std::complex<int16_t> tmp;
        if (iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp                = _lut_swap_16i[*((uint16_t *)&buf[2 * i])];
                itarget[i * 2]     = tmp.real();
                itarget[i * 2 + 1] = tmp.imag();
            }
        }
        else
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp                = _lut_16i[*((uint16_t *)&buf[2 * i])];
                itarget[i * 2]     = tmp.real();
                itarget[i * 2 + 1] = tmp.imag();
            }
        }
    }
    else if (rxFormat == RTL_RX_FORMAT_INT8)
    {
        int8_t *itarget = (int8_t *)buff0;
        if (iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                itarget[i * 2]     = buf[i * 2 + 1] - 128;
                itarget[i * 2 + 1] = buf[i * 2] - 128;
            }
        }
        else
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                itarget[i * 2]     = buf[i * 2] - 128;
                itarget[i * 2 + 1] = buf[i * 2 + 1] - 128;
            }
        }
    }

    // Advance position for next call into readStream
    _buf_head = head + returnedElems * BYTES_PER_SAMPLE;

    // Return number of elements written to buff0
    if (_buf_head != _buf_tail)
        flags |= SOAPY_SDR_MORE_FRAGMENTS;

    return returnedElems;
}
