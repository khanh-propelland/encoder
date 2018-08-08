#!/usr/bin/env python
# -*- coding: utf-8 -*-

# modification of code found here: https://github.com/arielnh56/ace128-rpi/blob/master/ace128.py#L9 without the backpack

import RPi.GPIO as GPIO
import time
import pickle


class Encoder:

    def __init__(self, pinOrder=(8,7,6,5,4,3,2,1), saveFile=None):
        self.reverse = False
        self.saveFile = saveFile
        #Ace128._bus.write_byte(self._i2caddr, 255)  # set all pins up. pulldown for input

        # setting up the pins as channels
        pinOne = GPIO.setup(9, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinTwo = GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinThree = GPIO.setup(0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinFour = GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinFive = GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinSix = GPIO.setup(1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinSeven = GPIO.setup(8, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        pinEight = GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        pins = [pinEight, pinSeven, pinSix, pinFive, pinFour, pinThree, pinTwo, pinOne]

        # create encoder map on the fly - ported from make_encodermap.ino
        # track binary data taken from p1 column on datasheet

        # creating a map that encodes a specific byte to a position
        track = [
            0b11000000, 0b00111111, 0b11110000, 0b00001111,
            0b11100000, 0b00011111, 0b11111111, 0b11111111,
            0b11111111, 0b00000000, 0b11111100, 0b00000011,
            0b10000000, 0b01111000, 0b00000110, 0b00000001,
            ]
        self._map = [255] * 256  # an array of all possible bit combinations
        for pos in range(0, 128):  # imagine rotating the encoder
            index = 0
            mask = 128 >> pos % 8  # which bit in current byte
            for pin in range(0, 8):  # think about each pin
                # which byte in track[] to look at.
                #  Each pin is 16 bits behind the previous
                offset = (pos - (1 - pinOrder[pin]) * 16) % 128 / 8
                if track[offset] & mask:  # is the bit set?
                    index |= 0b00000001 << pin  # set that pin's bit in the index byte

            self._map[index] = pos  # record the position in the map
        if self.saveFile:
            try:
                with open(self.saveFile, 'rb') as handle:
                    saveData = pickle.load(handle)
                    self._mpos = saveData.get('mpos', 0)
                    self._zero = saveData.get('zero', self.rawPos())
                    self._lastpos = self.pos()
            except:
                self._mpos = 0
                self._zero = self.rawPos()
                self._lastpos = 0
        else:
            self._mpos = 0
            self._zero = self.rawPos()
            self._lastpos = 0

    def readPins(self):
        current_index = '0b00000000'
        list_index = list(current_index)
        for i in range(0,8):
            if GPIO.input(pins[i]):
                list_index[i+2] = 1
        string_index = ''.join(list_index)  
        current_index = bin(int(string_index, 2))     

        return current_index

    def rawPos(self):
        
        return self._map[self.readPins()]

    def _raw2pos(self, raw):
        pos = raw - self._zero  # adjust for logical zero
        if self.reverse:
            pos *= -0b00000001  # reverse direction
        if pos > 63:
            pos -= 128
        elif pos < -64:
            pos += 128
        return pos

    def upos(self):
        pos = self.rawPos()  # get raw position
        pos -= self._zero  # adjust for logical zero
        if self.reverse:
            pos *= -1  # reverse direction
        pos &= 127  # clear the 8bit neg bit
        return pos

    def pos(self):
        return self._raw2pos(self.rawPos())

    def mpos(self):
        currentpos = self.pos()
        if self._lastpos - currentpos > 0x40:  # more than half a turn smaller - we rolled up
            self._mpos += 128
            self.__saveData()
        elif currentpos - self._lastpos > 0x40: # more than half a turn bigger - we rolled down
            self._mpos -= 128
            self.__saveData()
        self._lastpos = currentpos
        return self._mpos + currentpos

    def setZero(self, rawPos):
        self._zero = rawPos & 127
        self.__saveData()

    def setZero(self):
        self.setZero(self.rawPos())

    def getZero(self):
        return self._zero

    def setMpos(self, mPos):
        rawpos = self.rawPos()
        self.setZero(rawpos - (mPos & 127))  # mask to 7bit
        self._lastpos = self._raw2pos(rawpos)
        self._mpos = mPos - _lastpos & 0xFF80  # mask higher 9 bits
        self.__saveData()

    def __saveData(self):
        if self.saveFile:
            saveData = {'zero': self._zero, 'mpos': self._mpos}
            try:
                with open(self.saveFile, 'rb') as handle:
                    oldData = pickle.load(handle)
            except:
                oldData = { }

            if oldData != saveData:
                with open(self.saveFile, 'w') as handle:
                    pickle.dump(saveData, handle)
            

if __name__ == "__main__":
    # DEVICE = 0x38  # Device address (A0-A2)
    encoder = Encoder(saveFile="file.txt")

    while True:
        print encoder.rawPos(), encoder.upos(), encoder.pos(), encoder.mpos()