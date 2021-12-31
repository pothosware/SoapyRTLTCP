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
#include <SoapySDR/Registry.hpp>

static std::vector<SoapySDR::Kwargs> findRTLTCP(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;

    try
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-TCP trying to find server");
        SoapyRTLTCP device(args);

        const auto tuner = device.getHardwareKey();

        SoapySDR::Kwargs devInfo;
        devInfo["label"]  = std::string("rtl_tcp :: " + device.address);
        devInfo["rtltcp"] = device.address;
        devInfo["tuner"]  = tuner;

        results.push_back(devInfo);
    }
    catch (const std::exception &)
    {
        // no rtl_tcp server found
    }

    return results;
}

static SoapySDR::Device *makeRTLTCP(const SoapySDR::Kwargs &args)
{
    return new SoapyRTLTCP(args);
}

static SoapySDR::Registry registerRTLTCP("rtltcp", &findRTLTCP, &makeRTLTCP, SOAPY_SDR_ABI_VERSION);
