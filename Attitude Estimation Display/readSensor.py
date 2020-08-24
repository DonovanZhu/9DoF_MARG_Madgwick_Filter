#!/usr/bin/env python

from threading import Thread
import serial
import time
import struct
import numpy as np

class SerialRead:
    def __init__(self, serialPort='COM5', serialBaud=1000000, dataNumBytes=4, numParams=3):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numParams = numParams
        self.rawData = bytearray(numParams * dataNumBytes)
        self.dataType = None
        self.dataType = 'f'  # 4 byte float
        self.data = np.zeros(numParams)
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            exit()

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self):
        privateData = self.rawData[:]
        for i in range(self.numParams):
            data = privateData[(i * self.dataNumBytes):(self.dataNumBytes + i * self.dataNumBytes)]
            value, = struct.unpack(self.dataType, data)
            self.data[i] = value
        return self.data

    def backgroundThread(self):  # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while self.isRun:
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True
            # print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
