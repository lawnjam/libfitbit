#!/usr/bin/env python
#################################################################
# serial access for ant devices
# By Kyle Machulis <kyle@nonpolynomial.com>, 
#    John Lamb <john@lawnjam.com>
#
# Licensed under the BSD License, as follows
#
# Copyright (c) 2011, Kyle Machulis
# All rights reserved.
#
# Redistribution and use in source and binary forms, 
# with or without modification, are permitted provided 
# that the following conditions are met:
#
#    * Redistributions of source code must retain the 
#      above copyright notice, this list of conditions 
#      and the following disclaimer.
#    * Redistributions in binary form must reproduce the 
#      above copyright notice, this list of conditions and 
#      the following disclaimer in the documentation and/or 
#      other materials provided with the distribution.
#    * Neither the name of the Nonpolynomial Labs nor the names 
#      of its contributors may be used to endorse or promote 
#      products derived from this software without specific 
#      prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################
#

from protocol import ANT, ANTReceiveException
import serial
import operator
import time

class ANTpyserial(ANT):
    

    def __init__(self, chan=0x0, debug=False):
        super(ANTpyserial, self).__init__(chan, debug)
        self._connection = False
        self.timeout = 1000

    def open(self, port = None, baud = None):
        if port is None:
            port = self.PORT
        if baud is None:
            baud = self.BAUD
        self._connection = serial.Serial(port, baud) 
        if self._connection is None:
            return False
        
        return True

    def close(self):
        if self._connection is not None:
            self._connection = None

    def _send(self, command):
        c = command
        self._connection.write(''.join(c))

    def _receive(self, size=4096):
        r = self._connection.read(1)
        n = self._connection.inWaiting()
        r += self._connection.read(n)
        if len(r) == 0:
            return r
        if self._debug:
            self.data_received(''.join(map(chr, bytearray(r))))
        return bytearray(r)
