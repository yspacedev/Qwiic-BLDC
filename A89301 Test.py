import os
os.environ["BLINKA_MCP2221"] = "1" #initialize MCP2221 usage
import time
import board
import busio
i2c = board.I2C()

class A89301:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.EepromWrite=False
        #constants:
        self.direction_backwards = 0b0
        self.direction_forwards = 0b1
        #register values that will be used later
        self.ratedVoltageReg = self.readSubReg(20, 7, 0)
        self.senseResistorReg = self.readSubReg(20, 15, 8)
        self.ratedCurrentReg = self.readSubReg(10, 10, 0)
        self.ratedSpeedReg = self.readSubReg(8, 10, 0)
        
        while not self.i2c.try_lock():
            pass
    def writeReg(self, reg, val):
        MSB = val >> 8
        LSB = ((val << 8) & 0xFF00) >> 8
        self.i2c.writeto(self.addr, bytes([reg+64, MSB, LSB]), stop=True) # write to normal register (register is 64 above EEPROM address value)
        
        if self.EepromWrite and not (self.readReg(reg) == val): #don't write EEPROM if the value doesn't change
            self.i2c.writeto(self.addr, bytes([162, 0x00, reg]), stop=True) #set address to erase
            self.i2c.writeto(self.addr, bytes([163, 0x00, 0x00]), stop=True) #set write buffer to 0x0000
            self.i2c.writeto(self.addr, bytes([161, 0x00, 3]), stop=True) #set control to erase and set voltage high
            time.sleep(0.015)
            self.i2c.writeto(self.addr, bytes([162, 0x00, reg]), stop=True) #set address to write
            self.i2c.writeto(self.addr, bytes([163, MSB, LSB]), stop=True) #set write buffer to data to be written
            self.i2c.writeto(self.addr, bytes([161, 0x00, 5]), stop=True) #set control to write and set voltage high
            self.i2c.writeto(self.addr, bytes([reg, MSB, LSB]), stop=True)
            print("write EEPROM")
        
        if True:
            print(f"RAM match: {val==self.readReg(reg+64)}")
            print(f"EEPROM match: {val==self.readReg(reg)}")

    def readReg(self, reg, readRAM=True):
        result=bytearray(2)
        if readRAM:
            reg += 0 #64
        self.i2c.writeto(self.addr, bytes([reg]), stop=True)
        self.i2c.readfrom_into(self.addr, result)
        return int.from_bytes(result, "big")

    def readSubReg(self, reg, endpos, startpos, readRAM1=True):
        val = self.readReg(reg, readRAM=readRAM1)

        val = val >> startpos
        val = val & ((1 << (endpos-startpos+1)) - 1)
        
        return val
    
    def writeSubReg(self, reg, val, endpos, startpos):
        oldval = self.readReg(reg)

        mask = ((1 << (endpos-startpos+1)) - 1) << startpos
        val = val << startpos
        val = val & mask
        oldval = oldval & ~mask
        oldval = oldval | val

        self.writeReg(reg, oldval)


    def setEepromWrite(self, yes):
        self.EepromWrite=yes
    
    def unlock(self):
        self.i2c.unlock()
    
    def relock(self):
        while not self.i2c.try_lock():
            pass
    def writeAllReg(self, regmap):
        for regnum in range(len(regmap)):
            self.writeReg(regnum, regmap[regnum])
    def readAllReg(self):
        regmap=[]
        for reg in range(22):
            regmap.append(self.readReg(reg))
        return regmap
    
    def setSenseResistor(self, mOhms):
        self.senseResistorReg = int(mOhms*3.7) & 0xFF
        self.writeSubReg(20, self.senseResistorReg, 15, 8)
    def setRatedVoltage(self, V):
        self.ratedVoltageReg = int(V*5) & 0xFF
        self.writeSubReg(20, self.ratedVoltageReg, 7, 0)
    def setMotorResistance(self, ohms):
        self.motorResistanceReg = int(ohms*((self.ratedVoltageReg*4.096)/(self.senseResistorReg/125)/(self.ratedVoltageReg/10))) & 0xFF
        self.writeSubReg(9, self.motorResistanceReg, 15, 8)
    def setRatedCurrent(self, mA):
        self.ratedCurrentReg = int(mA*(self.senseResistorReg/125)) & 0b11111111111
        self.writeSubReg(10, self.ratedCurrentReg, 10, 0)
    def setRatedSpeed(self, RPM):
        self.ratedSpeedReg = int((RPM/60)/0.530) & 0b11111111111
        self.writeSubReg(8, self.ratedSpeedReg, 10, 0)
    def setAcceleration(self, acc): #acc in Hz/s
        if acc>12.75:
            ranger = 1
            k=3.2
        else:
            ranger = 0
            k=0.05
        self.accelReg = int(acc/k) & 0xFF
        self.writeSubReg(9, self.accelReg, 7, 0)
        self.writeSubReg(9, ranger, 13, 13)
    def setStartupCurrent(self, mA):
        ratedCurrent = self.ratedCurrentReg/(self.senseResistorReg/125)
        if (mA>0):
            self.startupCurrentReg=int((mA/(0.125*ratedCurrent))-1) & 0b111
        else:
            self.startupCurrentReg=0
        self.writeSubReg(10, self.startupCurrentReg, 15, 13)
    def setMotorInductance(self, val): #this is just a value
        self.writeSubReg(11, 0b0, 5, 5)
        self.writeSubReg(12, (val & 0b11111), 12, 8)
    def setOffThreshold(self, thresh): #0b01 = 6%
        self.writeSubReg(21, thresh, 9, 8)
    def setClosedLoop(self, yes):
        if yes:
            bit=0b1
        else:
            bit=0b0
        self.writeSubReg(8, bit, 11, 11)
    def setEnableCurrentLimit(self, yes):
        if yes:
            bit=0b1
        else:
            bit=0b0
        self.writeSubReg(11, bit, 7, 7)
    def setStartupMode(self, mode):
        self.writeSubReg(11, mode, 11, 10)
    def setIPD(self, A):
        self.IPDcurrentThrdValue = int(A/0.086) & 0b111111
        self.writeSubReg(18, self.IPDcurrentThrdValue, 13, 8)
    def setOpenWindow(self, yes):
        if yes:
            bit=0b1
        else:
            bit=0b0
        self.writeSubReg(12, bit, 15, 15)
    def setDelayStart(self, yes):
        if yes:
            bit=0b1
        else:
            bit=0b0
        self.writeSubReg(13, bit, 13, 13)
    def setOCPenable(self, yes): #I don't know if there are more possible values than just the two
        if yes:
            bit=0b100
        else:
            bit=0b111
        self.writeSubReg(16, bit, 2, 0)
    def setPIDparams(self, P, I):
        P = P & 0xFF
        I = I & 0xFF
        self.writeSubReg(12, P, 7, 0)
        self.writeSubReg(13, I, 7, 0)
    def setLockDisable(self, angleError, Vibration, overSpeed):
        self.writeSubReg(15, angleError, 3, 2)
        self.writeSubReg(22, Vibration, 10, 10)
        self.writeSubReg(12, overSpeed, 13, 13)
    def setRestartAttempts(self, val):
        self.writeSubReg(22, val, 7, 6)
    def setMosfetParams(self, deadtime, deadtimeCompEn, driveGateSlew, mosfetCissComp):
        self.writeSubReg(15, deadtime&0xF, 11, 8)
        self.writeSubReg(22, deadtimeCompEn&0b1, 12, 12)
        self.writeSubReg(18, driveGateSlew&0b11, 15, 14)
        self.writeSubReg(19, mosfetCissComp&0xFF, 15, 8)
    def setFirstCycleSpeed(self, spd):
        self.writeSubReg(16, spd&0b11, 7, 6)
    def setSoftSwitching(self, softOff, softOn, softOffTime):
        self.writeSubReg(15, softOff&0b1, 6, 6)
        self.writeSubReg(15, softOn&0b1, 7, 7)
        self.writeSubReg(22, softOffTime&0b1, 9, 9)
    def setVDSthreshSel(self, setting):
        self.writeSubReg(22, setting&0b1, 15, 15)
    def setBEMFlockFilter(self, val):
        self.writeSubReg(16, val&0b11, 13, 12)
    def setPhaseAdvance(self, deg):
        self.writeSubReg(11, 0b1, 5, 5)
        self.writeSubReg(12, (deg & 0b11111), 12, 8)
        
    def setSpeedDemand(self, demand):
        self.writeSubReg(17, 0b1, 9, 9) #speed mode set to i2c
        self.writeSubReg(17, demand&0b111111111, 8, 0)
    def setSpeedRPM(self, RPM):
        ratedRPM = self.ratedSpeedReg*0.530*60
        demand = (RPM/ratedRPM)*511
        self.setSpeedDemand(demand)
    def setDirection(self, direction):
        self.writeSubReg(8, direction, 14, 14)
    def readSpeed(self): #return RPM
        return self.readReg(120, readRAM=False)*0.530*60
    def readVoltage(self):
        return self.readReg(123, readRAM=False)/5
    def readCurrent(self): #mA
        return self.readReg(121, readRAM=False)/(self.senseResistorReg/125)
    def readOpState(self):
        return self.readSubReg(127, 15, 12, 12, readRAM1=False) #this throws an error

BLDC = A89301(i2c, 0x55)
import json
BLDC.setEepromWrite(False)
BLDC.setMosfetParams(0, 0b1, 0b01, 0x88)
BLDC.setSenseResistor(12.5)
BLDC.setRatedVoltage(12)
BLDC.setMotorResistance(0.065)
BLDC.setRatedCurrent(3000)
BLDC.setLockDisable(0b00, 0b0, 0b0)
BLDC.setBEMFlockFilter(0b00)
BLDC.setSpeed(100)

"""

BLDC.setSenseResistor(10)
BLDC.setRatedVoltage(12)
BLDC.setMotorResistance(0.065)
BLDC.setRatedCurrent(3000)
BLDC.setRatedSpeed(11000)
BLDC.setStartupCurrent(12000)
BLDC.setClosedLoop(True)
BLDC.setStartupMode(0b10)
BLDC.setOpenWindow(False)
BLDC.setDelayStart(True)
BLDC.setOCPenable(True)
BLDC.setEnableCurrentLimit(True)
BLDC.setMotorInductance(32)
BLDC.setAcceleration(100)
BLDC.setOffThreshold(0b01)
BLDC.setRestartAttempts(0b00)
#BLDC.setFirstCycleSpeed(
#BLDC.setSoftSwitching(
BLDC.setLockDisable(0b00, 0b0, 0b0)
BLDC.setBEMFlockFilter(0b00)
BLDC.setPIDparams(2, 0)

#BLDC.setEepromWrite(False)
BLDC.setSpeed(100)
#BLDC.setDirection(BLDC.direction_backwards)

print(BLDC.readVoltage())
print(BLDC.readCurrent())
print(BLDC.readOpState())
"""
#maybe these are good settings?
"""
BLDC.setEepromWrite(False)
BLDC.writeReg(8, 0x6334) # open loop, acceleration range is high

BLDC.writeReg(12, 0x0128) # motor inductance lowered

BLDC.writeReg(16, 0x206C) # disabled acc and dec buffer

# for speed via I2C

BLDC.writeReg(10, 0x4740) # increasing rated current

BLDC.writeReg(17, 0x22FF)

# motor resistance and sense resistor

BLDC.writeReg(9, 0x0E10) # increased startup acceleration

BLDC.writeReg(20, 0x584B)
"""
#motor resistance: 0.4 ohms
#inductance: 0.01mH