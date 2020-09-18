#!/usr/bin/env python

from datetime import datetime
import serial
import struct
import numpy as np
import copy


class serialPlot:
    def __init__(self, serialPort='COM4', serialBaud=1000000, dataNumBytes=4, numVariables=3):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numVariables = numVariables
        self.rawData = bytearray(numVariables * dataNumBytes)
        self.dataType = None
        self.dataType = 'f'  # 4 byte float
        self.dataBlock = []
        self.data = []
        for i in range(numVariables):
            self.data.append([])

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def getSerialData(self, t):
        self.serialConnection.reset_input_buffer()
        startTime = datetime.now()

        while (datetime.now() - startTime).total_seconds() < t:
            self.serialConnection.readinto(self.rawData)
            self.dataBlock.append(self.rawData[:])

        print("Captured data for %d seconds" % t)
        self.close()
        print("Processing data ...")

        for i in range(len(self.dataBlock)):
            for j in range(self.numVariables):
                byteData = self.dataBlock[i][(j * self.dataNumBytes):((j + 1) * self.dataNumBytes)]
                value, = struct.unpack(self.dataType, byteData)
                self.data[j].append(copy.copy(value))
        print("Exporting data ...")
        csvData = np.flip(np.array(self.data), 1).transpose()
        np.savetxt('magnetometer.csv', csvData, delimiter=',', fmt='%1.7f')
        print("Done")

    def close(self):
        self.serialConnection.close()
        print('Disconnected...')


def main():
    # Input the name of USB port
    portName = 'COM4'
    # portName = '/dev/ttyUSB0'
    baudRate = 1000000
    dataNumBytes = 4  # number of bytes of 1 data point
    numVariables = 3  # number of plots in 1 graph
    s = serialPlot(portName, baudRate, dataNumBytes, numVariables)  # initializes all required variables
    # time.sleep(2)
    s.getSerialData(120)


if __name__ == '__main__':
    main()
