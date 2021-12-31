# Soapy SDR module for RTL-TCP

Use an rtl_tcp server transparently as SoapySDR device.
The rtl_tcp protocol lacks many features, like reporting current settings and stream control.
Prefer any other module if possible.

By default, the module will attempt to discover a local rtl_tcp server. Specify the "rtltcp" key to connect at a specific address. The value of the "rtltcp" key should be the rtltcp server's hostname or IP address. If a custom port was selected, the value should be specified as "myServer:portNum":

```
SoapySDRUtil --find="rtltcp=myServer"
```

Note that IPv6 will be used if available and assigned for the interface, if you want IPv4 use e.g.:

```
SoapySDRUtil --find="rtltcp=127.0.0.1"
```

## Dependencies

* SoapySDR - https://github.com/pothosware/SoapySDR/wiki

## Licensing information

The MIT License (MIT)

Copyright (c) 2021 Christian Zuckschwerdt <zany@triq.net>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
