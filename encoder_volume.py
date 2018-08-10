#!/usr/bin/env python
# -*- coding: utf-8 -*-

# modification of code found here: https://github.com/arielnh56/ace128-rpi/blob/master/ace128.py#L9 without the backpack

import RPi.GPIO as GPIO
import time
import threading
import pickle
import os
from threading import Thread, Timer
from time import sleep
import signal
import subprocess
import sys
import pyrebase
import math
from queue import Queue

GPIO.setmode(GPIO.BCM)

# setting up the pins as channels
GPIO.setup(9, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(0, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(8, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pins = [24, 8, 1, 6, 5, 0, 11, 9]

GPIO_BUTTON = 26


# The minimum and maximum volumes, as percentages.

VOLUME_MIN = 0
VOLUME_MAX = 100


config = {
  "apiKey": "AIzaSyDqK4y0KO-JarxVBgwSdvLl8m1IAaVQK8E",
  "authDomain": "framer-test-1-ff664.firebaseapp.com",
  "databaseURL": "https://framer-test-1-ff664.firebaseio.com/",
  "storageBucket": "framer-test-1-ff664.appspot.com"
}

firebase = pyrebase.initialize_app(config)

# Get a reference to the database service
db = firebase.database()

knobValueData = {
    "knobValue": "0",
}

# When the knob is turned, the callback happens in a separate thread. 
# The callback will push onto a queue, and all the actual
# volume-changing will happen in the main thread. We'll use an event to 
# signal to the main thread that there's something in there. 
QUEUE = Queue()
EVENT = threading.Event()
    
    
class Encoder:

    def __init__(self, pins, buttonPin, pinOrder=(8,7,6,5,4,3,2,1), buttonCallback = None, saveFile=None):
        self.reverse = False
        self.saveFile = saveFile
        self.lastGpio = None
       
        self.last_volume = None
     
        # setting up the pins as channels
        self.pins = pins
        
        # setting up the button
        self.gpioButton     = buttonPin
        self.buttonCallback = buttonCallback
        
        GPIO.setup(self.gpioButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.gpioButton, GPIO.FALLING, self.buttonCallback, bouncetime=1000)
        
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
                offset = int((pos - (1 - pinOrder[pin]) * 16) % 128 / 8)
                if track[offset] & mask:  # is the bit set?
                    index |= 0b00000001 << (pinOrder[pin] -1)  # set that pin's bit in the index byte
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

        self.current_index = '0b11111111'
        self.list_index = list(str(self.current_index))
        
        for i in range(0,8):
            if GPIO.input(self.pins[i]) == False:
                self.list_index[i+2] = '0'
        
        self.string_index = ''.join(self.list_index)
        self.current_index = int(self.string_index, 2)     
        return self.current_index

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
        
    def mapToVolume(self):
        rawpos = self.rawPos()
        posSpan = 128
        volSpan = 60
        
        volume =  40 + math.ceil((volSpan * rawpos) / (posSpan))
        #QUEUE.put(volume)
        #EVENT.set()
        self.stored_volume = volume
        #print(volume)
        return (volume, self._callback(volume))
    
    def _callback(self, volume):
       
        if volume != self.last_volume:
            
  
        #self.callback(1, vol = new_volume)
        
            QUEUE.put(volume)
            EVENT.set()
        
        self.last_volume = volume
        return volume
    
            
        
    def buttonCallback(self, channel, vol = None):
        #print(vol, self.stored_volume)
        vol = self.mapToVolume() [0]


        self.buttonCallback(GPIO.input(channel), self.stored_volume)

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

class Volume:
  """
  A wrapper API for interacting with the volume settings on the RPi.
  """
  MIN = VOLUME_MIN
  MAX = VOLUME_MAX

  
  def __init__(self, volume):
    # Set an initial value for last_volume in case we're muted when we start.
    self.last_volume = volume
    self.delta = None
    self._sync()
    self.volume = volume 
    
  def change(self, vol):
    
    self.volume = vol
    v = self.volume
    v = self._constrain(v)
    return self.set_volume(v)
  
  def set_volume(self, v):
    global VOLUME
    self.volume = self._constrain(v)
   
    os.system("mpc volume " + str(v))
    knobValueData["knobValue"] = str(v)
    db.update(knobValueData)
    #superscript.ledVolumeControl(superscript.strip, v)
    
    return self.volume
    
  def toggle(self, vol):
    global VOLUME
    self.volume = vol
    #print(vol)
    if self.is_muted:

      os.system("mpc volume " + str(self.volume))
      #VOLUME = self.normalize(self.volume)
    
      VOLUME = self.volume
      knobValueData["knobValue"] = str(VOLUME)
      db.update(knobValueData)
      self.is_muted = not self.is_muted
    else:
      # We're about to mute ourselves, so we should remember the last volume
      # value we had because we'll want to restore it later.
      self.last_volume = self.volume
      
      VOLUME = 0
      QUEUE.put(VOLUME)
      EVENT.set()
      os.system("mpc volume 0")
      knobValueData["knobValue"] = "0"
      db.update(knobValueData)
      self.is_muted = not self.is_muted
  

  def status(self):
    if self.is_muted:
      return "{}% (muted)".format(self.volume)
    return "{}%".format(self.volume)
  
  # Read the output of `amixer` to get the system volume and mute state.

  def _sync(self, output=None):
    if output is None:
      output = self.amixer("get 'PCM'")
      
    lines = output.readlines()

    last = lines[-1].decode('utf-8')
    
    # The last line of output will have two values in square brackets. The
    # first will be the volume (e.g., "[95%]") and the second will be the
    # mute state ("[off]" or "[on]").
    i1 = last.rindex('[') + 1
    i2 = last.rindex(']')

    self.is_muted = last[i1:i2] == 'off'
    
    i1 = last.index('[') + 1
    i2 = last.index('%')
    # In between these two will be the percentage value.
    pct = last[i1:i2]

    #self.volume = int(pct)
  
  # Ensures the volume value is between our minimum and maximum.
  def _constrain(self, v):
    if v < self.MIN:
      return self.MIN
    if v > self.MAX:
      return self.MAX
    return v
    
  def amixer(self, cmd):
    p = subprocess.Popen("amixer {}".format(cmd), shell=True, stdout=subprocess.PIPE)
    code = p.wait()
  
    return p.stdout


if __name__ == "__main__":
   
    gpioButton = GPIO_BUTTON       
          
    def on_press(vol):
        #print("button pressed", vol)
        vol = encoder.mapToVolume()[0]
        #print("button pressed", vol)
        #vol = QUEUE.get()
        v.toggle(vol)
        EVENT.set()

                 
    def on_exit(pins):
        print("Exiting...")
        sys.exit(0)
      
    encoder = Encoder(pins, buttonPin=GPIO_BUTTON, buttonCallback=on_press, saveFile="file.txt")
    VOLUME = encoder.mapToVolume()
    v = Volume(volume=VOLUME)
    signal.signal(signal.SIGINT, on_exit)

    def consume_queue():        
         
        while not QUEUE.empty():
          vol = QUEUE.get()
          handle_volume(vol)
      
      
    def handle_volume(vol):
        if v.is_muted:
            v.toggle(vol)
        if vol != v.volume:
            v.change(vol)


      
    while True:
        #pass
        #EVENT.wait(100)
        encoder.mapToVolume()
        consume_queue()
        
        #print(encoder.lastGpio)
        #print(encoder.rawPos(), encoder.mapToVolume(), v.volume)
        EVENT.clear()
