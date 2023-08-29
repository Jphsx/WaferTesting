from tkinter import *
import motion
import numpy as np
import TransformMatrix
import keithley
import json
import os
import cv2
import time
import random
from datetime import datetime

class Window(Frame):

    def __init__(self, master=None, motorsPort='COM4', keithleyPort='COM13', emulate=False):
        Frame.__init__(self, master)        
        self.master = master
        self.emulate = emulate

        self.motors = motion.motion(port=motorsPort, emulate=emulate)

        self.kt = keithley.Keithley(port=keithleyPort, emulate=emulate)
        self.kt.reset()
        self.kt.on()

        self.safetyResist = 50000

        self.cam = cv2.VideoCapture(1)
        time.sleep(1)
        
        self.resistanceMeasurement = {
            'Sensor': "defaultSensor",
            'Measurement': "resistance",
            'Notes': "",
            'Run': "",
            'Values': []
        }

        self.stepSizeXY = 1
        self.stepSizeZ = 1
        
        self.sensorsPositions = []
        self.measuredPositions = []

        self.TM = TransformMatrix.TransformMatrix()
        # self.TM.Calibrate(self.sensorsPositions)

        self.stepSizeVarXY=StringVar() 
        self.stepSizeVarXY.set(str(self.stepSizeXY))
        stepsLabelXY = Label(self, text="mm")
        self.stepsEntryXY = Entry(self, textvariable=self.stepSizeVarXY, width=10)

        self.stepSizeVarZ=StringVar() 
        self.stepSizeVarZ.set(str(self.stepSizeZ))
        stepsLabelZ = Label(self, text="mm")
        self.stepsEntryZ = Entry(self, textvariable=self.stepSizeVarZ, width=10)

        self.moveToXVar=StringVar() 
        self.moveToXVar.set("0")
        self.moveToXEntry = Entry(self, textvariable=self.moveToXVar, width=10)
        self.moveToYVar=StringVar() 
        self.moveToYVar.set("0")
        self.moveToYEntry = Entry(self, textvariable=self.moveToYVar, width=10)

        expectedLabel = Label(self, text="Nominal position:")
        self.expectedPositionXVar=StringVar() 
        self.expectedPositionXVar.set("0")
        self.expectedPositionXEntry = Entry(self, textvariable=self.expectedPositionXVar, width=10)
        self.expectedPositionYVar=StringVar() 
        self.expectedPositionYVar.set("0")
        self.expectedPositionYEntry = Entry(self, textvariable=self.expectedPositionYVar, width=10)
        savedLabel = Label(self, text="Saved positions:")

        self.savedPointsVar=StringVar() 
        self.savedPointsVar.set("0")
        self.savedPointsLabel = Label(self, textvariable=self.savedPointsVar, width=10)

        menu = Menu(self.master)
        self.master.config(menu=menu)

        fileMenu = Menu(menu)
        fileMenu.add_command(label="Exit", command=self.exitProgram)
        fileMenu.add_command(label="Clear Calib", command=self.clearCalibrations)
        menu.add_cascade(label="File", menu=fileMenu)

        self.saveCalibrationFile = "defaultCalibration.json"
        self.saveCalibrationFileVar=StringVar() 
        self.saveCalibrationFileVar.set(str(self.saveCalibrationFile))
        saveCalibrationFileLabel = Label(self, text="Save Calibration To:")
        self.saveCalibrationFileEntry = Entry(self, textvariable=self.saveCalibrationFileVar, width=20)
        self.loadCalibrationFile = "defaultCalibration.json"
        self.loadCalibrationFileVar=StringVar() 
        self.loadCalibrationFileVar.set(str(self.loadCalibrationFile))
        loadCalibrationFileLabel = Label(self, text="Load Calibration From:")
        self.loadCalibrationFileEntry = Entry(self, textvariable=self.loadCalibrationFileVar, width=20)

        self.sensor = "Sensor"
        self.sensorVar=StringVar() 
        self.sensorVar.set(str(self.sensor))
        sensorLabel = Label(self, text="Sensor:")
        self.sensorEntry = Entry(self, textvariable=self.sensorVar, width=10)
        self.pad = "Pad"
        self.padVar=StringVar() 
        self.padVar.set(str(self.pad))
        padLabel = Label(self, text="Pad:")
        self.padEntry = Entry(self, textvariable=self.padVar, width=10)
        self.nominalPadPositionXVar=StringVar() 
        self.nominalPadPositionXVar.set("0")
        nominalPadPositionXLabel = Label(self, text="Pad X:")
        self.nominalPadPositionXEntry = Entry(self, textvariable=self.nominalPadPositionXVar, width=5)
        self.nominalPadPositionYVar=StringVar() 
        self.nominalPadPositionYVar.set("0")
        nominalPadPositionYLabel = Label(self, text="Pad Y:")
        self.nominalPadPositionYEntry = Entry(self, textvariable=self.nominalPadPositionYVar, width=5)

        self.notes = "Notes"
        self.notesVar=StringVar() 
        self.notesVar.set(str(self.notes))
        notesLabel = Label(self, text="Notes:")
        self.notesEntry = Entry(self, textvariable=self.notesVar, width=10)

        self.run = 0
        self.runVar=StringVar() 
        self.runVar.set(str(self.run))
        runLabel = Label(self, text="Run:")
        self.runEntry = Entry(self, textvariable=self.runVar, width=5)

        self.loadPointsFile = "defaultPoints.json"
        self.loadPointsFileVar=StringVar() 
        self.loadPointsFileVar.set(str(self.loadPointsFile))
        loadPointsFileLabel = Label(self, text="Load Points:")
        self.loadPointsFileEntry = Entry(self, textvariable=self.loadPointsFileVar, width=18)

        # widget can take all window
        self.pack(fill=BOTH, expand=1)

        self.saveButton = Button(self, text="Save Calibration Point", width=25, height=2, command=self.savePosition)
        readPositionButton  = Button(self, text="Read Position", width=10, height=2, command=self.readPosition)
        self.setHomeButton = Button(self, text="Set Home", width=10, height=2, command=self.setHome)
        self.goHomeButton = Button(self, text="Go Home", width=10, height=2, command=self.goHome)
        self.calibrateButton = Button(self, text="Calibrate", width=10, height=2, command=self.calibrate)
        self.saveCalibrationButton = Button(self, text="Save Calibration", width=15, height=2, command=self.saveCalibration)
        self.saveCalibrationButton['state'] = DISABLED
        self.loadCalibrationButton = Button(self, text="Load Calibration", width=15, height=2, command=self.loadCalibration)
        self.moveToButton = Button(self, text="Move To", width=10, height=2, command=lambda: self.moveToPosition())
        self.moveToButton['state'] = DISABLED

        upButton = Button(self, text="Up", width=10, height=2, command=lambda: self.moveBtn(0,1*float(self.stepsEntryXY.get()),0))
        upRightButton = Button(self, text="U-R", width=10, height=2, command=lambda: self.moveBtn(-1*float(self.stepsEntryXY.get()),1*float(self.stepsEntryXY.get()),0))
        upLeftButton = Button(self, text="U-L", width=10, height=2, command=lambda: self.moveBtn(1*float(self.stepsEntryXY.get()),1*float(self.stepsEntryXY.get()),0))
        downButton = Button(self, text="Down", width=10, height=2, command=lambda: self.moveBtn(0,-1*float(self.stepsEntryXY.get()),0))
        downRightButton = Button(self, text="D-R", width=10, height=2, command=lambda: self.moveBtn(-1*float(self.stepsEntryXY.get()),-1*float(self.stepsEntryXY.get()),0))
        downLeftButton = Button(self, text="D-L", width=10, height=2, command=lambda: self.moveBtn(1*float(self.stepsEntryXY.get()),-1*float(self.stepsEntryXY.get()),0))
        leftButton = Button(self, text="Left", width=10, height=2, command=lambda: self.moveBtn(1*float(self.stepsEntryXY.get()),0,0))
        rightButton = Button(self, text="Right", width=10, height=2, command=lambda: self.moveBtn(-1*float(self.stepsEntryXY.get()),0,0))
        raiseButton = Button(self, text="Raise", width=10, height=2, command=lambda: self.moveBtn(0,0,1*float(self.stepsEntryZ.get())))
        lowerButton = Button(self, text="Lower", width=10, height=2, command=lambda: self.moveBtn(0,0,-1*float(self.stepsEntryZ.get())))

        upFastButton = Button(self, text="10x U", width=10, height=2, command=lambda: self.moveBtn(0,10*float(self.stepsEntryXY.get()),0))
        upRightFastButton = Button(self, text="10x U-R", width=10, height=2, command=lambda: self.moveBtn(-10*float(self.stepsEntryXY.get()),10*float(self.stepsEntryXY.get()),0))
        upLeftFastButton = Button(self, text="10x U-L", width=10, height=2, command=lambda: self.moveBtn(10*float(self.stepsEntryXY.get()),10*float(self.stepsEntryXY.get()),0))
        downFastButton = Button(self, text="10x D", width=10, height=2, command=lambda: self.moveBtn(0,-10*float(self.stepsEntryXY.get()),0))
        downRightFastButton = Button(self, text="10x D-R", width=10, height=2, command=lambda: self.moveBtn(-10*float(self.stepsEntryXY.get()),-10*float(self.stepsEntryXY.get()),0))
        downLeftFastButton = Button(self, text="10x D-L", width=10, height=2, command=lambda: self.moveBtn(10*float(self.stepsEntryXY.get()),-10*float(self.stepsEntryXY.get()),0))
        leftFastButton = Button(self, text="10x L", width=10, height=2, command=lambda: self.moveBtn(10*float(self.stepsEntryXY.get()),0,0))
        rightFastButton = Button(self, text="10x R", width=10, height=2, command=lambda: self.moveBtn(-10*float(self.stepsEntryXY.get()),0,0))
        raiseFastButton = Button(self, text="10X Raise", width=10, height=2, command=lambda: self.moveBtn(0,0,10*float(self.stepsEntryZ.get())))
        lowerFastButton = Button(self, text="10X Lower", width=10, height=2, command=lambda: self.moveBtn(0,0,-10*float(self.stepsEntryZ.get())))

        # keithley buttons
        readResistanceButton = Button(self, text="Read Resistance", width=15, height=2, command=self.readResistance)
        checkContactButton = Button(self, text="Check Contact", width=15, height=2, command=self.checkContact)
        getMinResistanceButton = Button(self, text="Get Min Resistance", width=15, height=2, command=self.getMinResistance)

        self.raiseToContactButton=Button(self, text="Raise To Contact", width=15, height=2, command=self.raiseToContact)
        self.saveResistanceButton=Button(self, text="Save Resistance", width=15, height=2, command=self.saveResistanceManual)
        self.writeResistanceButton=Button(self, text="Write Resistance", width=15, height=2, command=self.writeResistance)

        self.loadPointsButton = Button(self, text="Load Points", width=15, height=2, command=self.loadPoints)
        self.measureAllPointsButton = Button(self, text="Measure Points", width=15, height=2, command=self.measureAllPoints)
        self.measureAllPointsButton['state'] = DISABLED
        self.testAllPointsButton = Button(self, text="Test Points", width=15, height=2, command=lambda: self.measureAllPoints(test=True))
        self.testAllPointsButton['state'] = DISABLED

        # buttons for setting safety limits
        self.setUpperLimitX = Button(self, text="Set Right Limit", width=15, height=2, command=lambda: self.setSafetyLimit('x',False))
        self.setLowerLimitX = Button(self, text="Set Left Limit", width=15, height=2, command=lambda: self.setSafetyLimit('x',True))
        self.setUpperLimitY = Button(self, text="Set Up Limit", width=15, height=2, command=lambda: self.setSafetyLimit('y',True))
        self.setLowerLimitY = Button(self, text="Set Down Limit", width=15, height=2, command=lambda: self.setSafetyLimit('y',False))
        self.setUpperLimitZ = Button(self, text="Set Raise Limit", width=15, height=2, command=lambda: self.setSafetyLimit('z',True))
        self.setLowerLimitZ = Button(self, text="Set Lower Limit", width=15, height=2, command=lambda: self.setSafetyLimit('z',False))

        # buttons for resetting safety limits
        self.resetUpperLimitX = Button(self, text="Reset Right Limit", width=15, height=2, command=lambda: self.resetSafetyLimit('x',False))
        self.resetLowerLimitX = Button(self, text="Reset Left Limit", width=15, height=2, command=lambda: self.resetSafetyLimit('x',True))
        self.resetUpperLimitY = Button(self, text="Reset Up Limit", width=15, height=2, command=lambda: self.resetSafetyLimit('y',True))
        self.resetLowerLimitY = Button(self, text="Reset Down Limit", width=15, height=2, command=lambda: self.resetSafetyLimit('y',False))
        self.resetUpperLimitZ = Button(self, text="Reset Raise Limit", width=15, height=2, command=lambda: self.resetSafetyLimit('z',True))
        self.resetLowerLimitZ = Button(self, text="Reset Lower Limit", width=15, height=2, command=lambda: self.resetSafetyLimit('z',False))

        # button for needle stress test
        self.NeedleTest = Button(self, text="Needle Test", width=15, height=2, command=self.NeedleTest)

        # place buttons
        self.saveButton.place(x=300, y=500)
        stepsLabelXY.place(x=200,y=100)
        self.stepsEntryXY.place(x=200,y=125)
        stepsLabelZ.place(x=60,y=275)
        self.stepsEntryZ.place(x=60,y=300)
        self.setHomeButton.place(x=200,y=400)
        self.goHomeButton.place(x=200,y=450)
        self.calibrateButton.place(x=10,y=450)

        saveCalibrationFileLabel.place(x=10,y=500)
        self.saveCalibrationFileEntry.place(x=10,y=525)
        self.saveCalibrationButton.place(x=10,y=550)
        loadCalibrationFileLabel.place(x=150,y=500)
        self.loadCalibrationFileEntry.place(x=150,y=525)
        self.loadCalibrationButton.place(x=150,y=550)

        expectedLabel.place(x=300 ,y=400)
        self.expectedPositionXEntry.place(x=300 ,y=450)
        self.expectedPositionYEntry.place(x=375 ,y=450)
        self.moveToXEntry.place(x=300 ,y=300)
        self.moveToYEntry.place(x=400 ,y=300)
        self.moveToButton.place(x=200,y=300)
        readPositionButton.place(x=100,y=450)
        savedLabel.place(x=300 ,y=550)
        self.savedPointsLabel.place(x=400 ,y=550)

        upButton.place(x=200, y=60)
        upRightButton.place(x=300, y=60)
        upLeftButton.place(x=100, y=60)
        downButton.place(x=200, y=160)
        downRightButton.place(x=300, y=160)
        downLeftButton.place(x=100, y=160)
        leftButton.place(x=100, y=110)
        rightButton.place(x=300, y=110)
        upFastButton.place(x=200, y=10)
        upRightFastButton.place(x=400, y=10)
        upLeftFastButton.place(x=10, y=10)
        downFastButton.place(x=200, y=210)
        downRightFastButton.place(x=400, y=210)
        downLeftFastButton.place(x=10, y=210)
        leftFastButton.place(x=10, y=110)
        rightFastButton.place(x=400, y=110)
        raiseFastButton.place(x=10, y=350)
        lowerFastButton.place(x=10, y=400)
        raiseButton.place(x=100, y=350)
        lowerButton.place(x=100, y=400)

##        self.setUpperLimitX.place(x=500,y=5)
##        self.setLowerLimitX.place(x=500,y=55)
##        self.setUpperLimitY.place(x=500,y=105)
##        self.setLowerLimitY.place(x=500,y=155)
##        self.setUpperLimitZ.place(x=500,y=205)
##        self.setLowerLimitZ.place(x=500,y=255)
##
##        self.resetUpperLimitX.place(x=500,y=305)
##        self.resetLowerLimitX.place(x=500,y=355)
##        self.resetUpperLimitY.place(x=500,y=405)
##        self.resetLowerLimitY.place(x=500,y=455)
##        self.resetUpperLimitZ.place(x=500,y=505)
##        self.resetLowerLimitZ.place(x=500,y=555)

        readResistanceButton.place(x=575,y=110)
        checkContactButton.place(x=575,y=60)
        getMinResistanceButton.place(x=800,y=60)
        loadPointsFileLabel.place(x=575,y=150)
        self.loadPointsFileEntry.place(x=575,y=175)
        self.loadPointsButton.place(x=575,y=200)
        self.testAllPointsButton.place(x=575,y=250)
        self.measureAllPointsButton.place(x=575,y=300)
        self.raiseToContactButton.place(x=575,y=10)

        sensorLabel.place(x=625,y=400)
        self.sensorEntry.place(x=625,y=425)
        padLabel.place(x=625,y=450)
        self.padEntry.place(x=625,y=475)
        self.saveResistanceButton.place(x=575,y=500)
        self.writeResistanceButton.place(x=575,y=550)
        nominalPadPositionXLabel.place(x=575,y=400)
        self.nominalPadPositionXEntry.place(x=575,y=425)
        nominalPadPositionYLabel.place(x=575,y=450)
        self.nominalPadPositionYEntry.place(x=575,y=475)
        notesLabel.place(x=575,y=350)
        self.notesEntry.place(x=575,y=375)
        runLabel.place(x=650,y=350)
        self.runEntry.place(x=650,y=375)

        self.NeedleTest.place(x=700,y=110)

    def exitProgram(self):
        exit()
    
    def clearCalibrations(self):
        print("Clearing Calibrations")
        self.sensorsPositions = []
        self.measuredPositions = []
        self.saveButton['state'] = ACTIVE
        self.setHomeButton['state'] = ACTIVE
        self.calibrateButton['state'] = ACTIVE
        self.moveToButton['state'] = DISABLED
        self.savedPointsVar.set("0")
        self.TM.Reset()

    def saveCalibration(self):
        try:
            to_unicode = unicode
        except NameError:
            to_unicode = str
        calibration = {
            'sensorsPositions': self.sensorsPositions,
            'measuredPositions': self.measuredPositions
        }
        filename = str(self.saveCalibrationFileEntry.get())
        with open(filename,'w',encoding='utf8') as f:
           dump = json.dumps(calibration,indent=4,sort_keys=True,separators=(',',': '),ensure_ascii=False)
           f.write(to_unicode(dump))
        print("Saved Calibration to: "+filename)
        
    def loadCalibration(self):
        self.setHome()
        filename = str(self.loadCalibrationFileEntry.get())
        with open(filename) as f:
            calibration=json.load(f)
        self.sensorsPositions = calibration.get('sensorsPositions')
        self.measuredPositions = calibration.get('measuredPositions')
        print("Loaded Calibration from: "+filename)
        self.calibrate()

    def savePosition(self):
        print(f'Saving position at {len(self.sensorsPositions)}')
        if len(self.sensorsPositions) == 0:
            self.setHome()
            self.sensorsPositions.append([0,0])
        else:
            expX = -1.*float(self.expectedPositionXEntry.get())
            expY = -1.*float(self.expectedPositionYEntry.get())
            self.sensorsPositions.append([expX,expY])
        self.savedPointsVar.set(len(self.sensorsPositions))
        lastPosition = self.motors.getPosition()
        self.moveToXVar.set(str(-1.*lastPosition[0]))
        self.moveToYVar.set(str(lastPosition[1]))
        self.measuredPositions.append(list([np.round(lastPosition[0],decimals=3),np.round(lastPosition[1],decimals=3),np.round(lastPosition[2],decimals=3)]))

    def moveToPosition(self,x=-10000,y=-10000, zRetraction=2):
        self.motors.moveFor('z',-1*zRetraction)
        positionX = 0
        positionY = 0
        if x == -10000:
            positionX = -1.*float(self.moveToXEntry.get())
        else:
            positionX = x
        if y == -10000:
            positionY = float(self.moveToYEntry.get())
        else:
            positionY = y
        if str(self.calibrateButton['state']) == "disabled":
            positionY = -1.*positionY
        print(f"Position non corrected: {positionX}, {positionY}")
        padPos = self.TM.Transform(np.array([positionX,positionY]))
        print(f"Position corrected: {padPos[0]}, {padPos[1]}")
        self.motors.moveTo('x',padPos[0])
        self.motors.moveTo('y',padPos[1])
        self.motors.moveFor('z',zRetraction)

    def setHome(self):
        self.motors.setHome()
        self.readPosition()
        self.moveToButton['state'] = ACTIVE
        self.motors.setSafetyLimit(motor='z',min=None,max=2.0)

    def goHome(self):
        self.motors.moveFor('z',-5)
        self.motors.moveTo('x')
        self.motors.moveTo('y')
        self.motors.moveTo('z')
        self.readPosition()
        self.moveToButton['state'] = ACTIVE

    def readPosition(self):
        pos = self.motors.getPosition()
        self.moveToXVar.set(str(pos[0]))
        self.moveToYVar.set(str(pos[1]))

    def moveBtn(self, x, y, z):
        if x is not 0:
            self.motors.moveFor('x',x)
        if y is not 0:
            self.motors.moveFor('y',y)
        if z is not 0:
##            if z > 0:
##                if self.checkContact() is True:
##                    print("Not going to move as chuck is already in contact...")
##                else:
##                    print("Raising Chuck...")
##                    self.motors.moveFor('z',z)
##                    self.checkContact()
##            else:
##                self.motors.moveFor('z',z)
            self.motors.moveFor('z',z)

    def calibrate(self):
        print("Calibrating ")
        print(self.sensorsPositions)
        print(self.measuredPositions)
        self.TM.Calibrate(self.sensorsPositions, self.measuredPositions, weighted=True)   
        self.saveButton['state'] = DISABLED
        self.setHomeButton['state'] = DISABLED
        self.calibrateButton['state'] = DISABLED
        self.saveCalibrationButton['state'] = ACTIVE

    def setSafetyLimit(self,motor,upper):
        if upper is True:
            if motor is 'z':
                self.motors.setSafetyLimit(motor=motor,min=None,max=np.round(float(self.motors.getPosition(motor))+0.05,decimals=3))
            else:
                self.motors.setSafetyLimit(motor=motor,min=None,max=np.round(self.motors.getPosition(motor),decimals=3))
        else:
            self.motors.setSafetyLimit(motor=motor,min=np.round(self.motors.getPosition(motor),decimals=3),max=None)

    def resetSafetyLimit(self,motor,upper):
        if upper is True:
            self.motors.resetSafetyLimit(motor=motor,min=1,max=None)
        else:
            self.motors.resetSafetyLimit(motor=motor,min=None,max=1)

    def readResistance(self, debug=True, points=10, powerLimit=1e-2, currentLimit=10e-3):
        print("Measuring Resistance")
        time.sleep(0.5)
        resist = 0
        resistErr = 0
        if self.emulate:
            print("In emulation Mode")
            print("Will not check resistance")
            return resist, resistErr
        results = self.kt.measresist(debug=debug,points=points,powerLimit=powerLimit,currentLimit=currentLimit)
        resist = results['R']
        resistErr = results['Rerr']
        results['Z'] = self.motors.getPosition('z')
        print(f"{results['R']:.3f} +- {results['Rerr']:.3f}")
        return results

    def saveResistance(self,sensor="defaultSensor", padNumber=-1, notes="", run=0, x=-1, y=-1, results=None):
        if results is None:
            results = self.readResistance(debug=True)
        if results['R'] == 1e6:
            print("Not Good Contact! Will not save...")
            return False
        now = datetime.now()
        date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        pad = {
            'Pad': padNumber,
            'PositionX': x,
            'PositionY': y,
            'PositionZ': results['Z'],
            'Resistance': results['R'],
            'ResistanceError': results['Rerr'],
            'CurrentPoints': results['I'],
            'VoltagePoints': results['V'],
            'Time': date_time,
            'Notes': str(self.notesEntry.get()),
            'Run': run
        }
        print("Saving Resitance for pad: "+str(padNumber)+" on sensor: "+sensor)
        self.resistanceMeasurement['Values'].append( pad )
        self.kt.keithley.beep(frequency=2000,duration=1)

    def saveResistanceManual(self):
        sensor = str(self.sensorEntry.get())
        padNumber = int(self.padEntry.get())
        notes = str(self.notesEntry.get())
        run = str(self.runEntry.get())
        x = float(self.nominalPadPositionXEntry.get())
        y = float(self.nominalPadPositionYEntry.get())
        self.saveResistance(sensor,padNumber,notes,run,x,y,self.readResistance(debug=True))

    def writeResistance(self):
        try:
            to_unicode = unicode
        except NameError:
            to_unicode = str
        sensor = str(self.sensorEntry.get())
        run = str(self.runEntry.get())
        filename = sensor+"_Run"+run+"_"+"resistance.json"
        self.resistanceMeasurement["Notes"] =  str(self.notesEntry.get())
        if os.path.exists(filename):
            os.remove(filename)
        with open(filename,'a',encoding='utf8') as f:
           dump = json.dumps(self.resistanceMeasurement,indent=4,sort_keys=True,separators=(',',": "),ensure_ascii=False)
           f.write(to_unicode(dump))
        print("Writing Resitance for run "+run+" of sensor: "+sensor+" to file: "+filename)

    def loadPoints(self):
        filename = str(self.loadPointsFileEntry.get())
        with open(filename) as f:
            points=json.load(f)
        self.resistanceMeasurement.update(points)
        print("Loaded Points from: "+filename)
        self.measureAllPointsButton['state'] = ACTIVE
        self.testAllPointsButton['state'] = ACTIVE

    def measureAllPoints(self,test=False):
        sensor = str(self.sensorEntry.get())
        run = str(self.runEntry.get())
        lastContactZ = 0
        total_attempts = 1 # note should be 3 for "real" testing
        if test:
            self.motors.moveFor('z',-0.5)
        else:
            if not os.path.exists("img/"+sensor+"_Run"+run):
                os.makedirs("img/"+sensor+"_Run"+run)

        for p in self.resistanceMeasurement['Pads']:
            print("Measuring Pad: "+str(p.get('Pad')))
            x_given = -1*p.get('PositionX') #Note flip here
            y_given = p.get('PositionY')

            attempt = 0

            self.moveToPosition(x=x_given,y=y_given)
            if p.get('Pad') == 12345:
                lastContactZ = lastContactZ -0.1
                self.writeResistance()
                print("Waiting for needles to be setup for next run...")
                input("Press Enter twice to continue...")
                continue
            if test:
                continue

            if lastContactZ is 0:
                self.motors.moveTo('z',0.0)
                if self.raiseToSoftContact(step=0.2,maxMovement=1.8):
                    lastContactZ = self.motors.getPosition('z')
                    print(f"Setting {lastContactZ:.3f} as lastContactZ")
                    self.raiseToContact(step=0.1,maxMovement=1.)
            else:
                self.motors.moveTo('z',(lastContactZ-0.2))
                if self.raiseToSoftContact(step=0.2,maxMovement=0.8):
                    lastContactZ = self.motors.getPosition('z')
                    print(f"Setting {lastContactZ:.3f} as lastContactZ")
                    self.raiseToContact(step=0.1,maxMovement=0.5)

            while attempt <= total_attempts:
                if attempt > 0:
                    self.motors.moveFor('z',0.05)
                attempt += 1
                frame = None
                while frame is None:
                    pad = p.get('Pad')
                    time.sleep(0.25)
                    ret, frame = self.cam.read()
                    cv2.imwrite(f"img/"+sensor+"_Run"+run+"/Pad"+str(pad)+"_"+str(attempt)+".png", frame)
                results = self.readResistance(debug=True)
                notes = str(self.notesEntry.get())
                self.saveResistance(sensor,p.get('Pad'),notes,run,x_given,y_given,results) # save it in any case, we take care in data analysis
            self.motors.moveTo('z',-0.1)

        self.goHome()
        self.cam.release()
        if not test:
            self.writeResistance()
        
    def checkContact(self):
        print("Checking for contact with resistance threshold of: "+str(self.safetyResist)+" Ohms")
        results = self.readResistance(debug=True,points=5)
        resist = results['R']
        resistErr = results['Rerr']
        if self.emulate:
            print("In emulation mode")
            return False
        if 0 < resist < self.safetyResist and resistErr < 0.001*resist:
            print("Probe Needle is touching")
            self.kt.keithley.beep(frequency=2700,duration=0.5)
            self.kt.keithley.beep(frequency=2700,duration=0.5)
            return True
        else:
            print("Probe Needle is NOT touching")
            return False

    def checkSoftContact(self):
        print("Checking for soft contact with resistance threshold of: "+str(self.safetyResist)+" Ohms")
        results = self.readResistance(debug=True,points=4)
        resist = results['R']
        resistErr = results['Rerr']
        if self.emulate:
            print("In emulation mode")
            return False
        if resist != 1e6 and resistErr != 1e6:
            print("Probe Needle is touching")
            self.kt.keithley.beep(frequency=2700,duration=0.5)
            self.kt.keithley.beep(frequency=2700,duration=0.5)
            return True
        else:
            print("Probe Needle is NOT touching")
            return False

    def raiseToContact(self, step=0.05, maxMovement=1):
        print("Raising Until Made Contact")
        totMovement = 0.0
        firstContact = True
        contact = False
        while totMovement < maxMovement:
            contact = self.checkContact()
            if contact is False:
                self.motors.moveFor('z',step)
                totMovement += step
                firstContact = True
            elif firstContact:
                firstContact = False
            else:
                break
        if contact:
            print("Made Contact")
            self.kt.keithley.beep(frequency=2700,duration=1)
            self.kt.keithley.beep(frequency=2700,duration=1)
        else:
            print("Could not make contact in: "+str(maxMovement/step)+" attempts")
            self.kt.keithley.beep(frequency=1000,duration=1)
        return contact
    
    def raiseToSoftContact(self, step=0.05, maxMovement=1):
        print("Raising Until Made Soft Contact")
        totMovement = 0.0
        firstContact = True
        contact = False
        while totMovement < maxMovement:
            contact = self.checkSoftContact()
            if contact is False:
                self.motors.moveFor('z',step)
                totMovement += step
                firstContact = True
            elif firstContact:
                firstContact = False
            else:
                break
        if contact:
            print("Made Soft Contact")
            self.kt.keithley.beep(frequency=1500,duration=1)
            time.sleep(0.25)
            self.kt.keithley.beep(frequency=1500,duration=1)
        else:
            print("Could not make soft contact in: "+str(maxMovement/step)+" attempts")
            self.kt.keithley.beep(frequency=1000,duration=1)
        return contact

    def getMinResistance(self, step=0.03, maxMovement=0.35):
        self.raiseToContact(step=0.05,maxMovement=0.1)
        print("Checking if we are at maximum z...")
        results = []
        resistances = []
        result = self.readResistance(debug=True)
        results.append(result)
        resist = result['R']
        resistances.append(resist)
        resistErr = result['Rerr']
        ##Additional Check that we have good contact
        if abs(resistErr) > resist or resist > self.safetyResist:
            print("Not Good Contact! Check that you have good contact (low resistance) first...")
            return result
        totMovement = 0.0
        ##Scan z by 'step' mm up to a max of 'maxMovement' mm
        while totMovement < maxMovement:
            self.motors.moveFor('z',step)
            totMovement += step
            result = self.readResistance(debug=True)
            results.append(result)  ## always add the result, then we see how it is
            resist = result['R']
            resistErr = result['Rerr']
            resistances.append(resist)
            if resist > 1.2*results[-2]['R']:
                ##Assuming that this is because the resistance started to go back up
                break
            if abs(resistErr) > 0.25*resist:
                continue
##                self.motors.moveFor('z',-totMovement)   #do we need it?
##                print("Not Good Contact! Check that you have good contact (low resistance) first...")
##                result = results[0]
##                result['Z'] = self.motors.getPosition('z')
##                result['R'] = 1e6
##                return result

            if(len(results) > 2):
                if abs(results[-1]['R'] - results[-2]['R']) <= 0.1*resist and abs(results[-1]['R'] - results[-3]['R']) <= 0.1*resist:
                    break
        #Now that we have our resistances we figure out which is the smallest and assume that to be the "true" resistance
        finalresist = min(resistances)
        finalresult = results[resistances.index(min(resistances))]
        print("Found Minimum: "+str(finalresult['R'])+" +- "+str(finalresult['Rerr']))
        self.motors.moveFor('z',-totMovement)
        return finalresult

    def getMinResistance2(self, down=-0.03, up=0.05):
        print("Checking again for good contact...")
        self.raiseToContact(step=0.05,maxMovement=0.1)
        self.motors.moveFor('z',up)
        self.motors.moveFor('z',down)
        result = self.readResistance(debug=True)
        self.motors.moveFor('z',-1.0*(up+down)) #go back to original spot
        return result

    def NeedleTest(self):
        print("Testing Needles")
        sensor = str(self.sensorEntry.get())
        cam = cv2.VideoCapture(1)
        time.sleep(10)
        attempts = 200
        for attempt in range(0, attempts):
            self.motors.moveFor('z',-3)
            self.motors.moveFor('z',2.75)
            self.raiseToContact(step=0.05, maxMovement=0.5)
            self.motors.moveFor('z',0.05)
            self.saveResistance(padNumber=attempt, notes="Needle Stress Test", run=0, x=0, y=0,results=self.readResistance(debug=True,points=10,powerLimit=1,currentLimit=0.1))
            self.motors.moveFor('z',-0.05)
            time.sleep(300)
            #frame = None
            #while frame is None:
                #time.sleep(0.2)
                #ret, frame = cam.read()
                #cv2.imwrite(f"img/{sensor}/test{attempt}.png", frame)
        cam.release()
        self.writeResistance()
            

root = Tk()
app = Window(root)
root.wm_title("Tkinter button")
root.geometry("800x600")
root.mainloop()

