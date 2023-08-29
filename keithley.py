from pymeasure.instruments.keithley import Keithley2400
from pymeasure import * 
import numpy as np
from scipy.optimize import curve_fit

from time import sleep

class Keithley:
    """Class to control the Keithley 2410"""

    def __init__(self, port='ASRL/dev/ttyUSB0::INSTR', emulate=False, vmax=10, accuracy=1):
        self.port = port
        self.vmax = vmax
        self.emulate = emulate
        self.accuracy = accuracy
        self.mode = None
        self.keithley = Keithley2400(port)
        if not self.emulate:
            self.keithley.write("*RST")
            print(self.keithley.ask("*IDN?"))

        else:
            print("Emulation mode")
        
        print("Keithley Initialized")

    def meascurr(self, units=1e-6, debug=False):
        if self.mode != "mearcurr":
            self.keithley.write(":SENS:CURR:RANGE:AUTO 1")
            self.keithley.write('voltage:nplc %2f'%(self.accuracy))
            self.mode = "mearcurr"
            self.on()
        data = self.keithley.ask(":READ?")
        if debug:
            print(data)
        string = data.split(',')
        current = float(string[1])/units
        return current
    
    def measresist(self, powerLimit=1e-2, points=10, currentLimit=10e-3, delay=0.1, debug=False):
        if self.mode != "measvolt":
            self.keithley.apply_current()
            self.keithley.source_current_range = currentLimit
            self.keithley.compliance_voltage = self.vmax
            self.keithley.measure_voltage(nplc=self.accuracy, voltage=self.vmax, auto_range=True)
            self.mode = "measvolt"
            self.keithley.source_current = 0
        self.on()

        results = {}
        currentPoints = []
        voltagePoints = []
        i = 0.0
        currentIncrement = 1e-6

        while True:
            self.keithley.source_current = i
            sleep(delay) 
            v = self.keithley.voltage
            power = np.abs(i*v)

            if len(voltagePoints)==0 and v > 1:
                print("ERROR: not touching! retrying... ")
                sleep(0.2) 
                v = self.keithley.voltage
                if v > 1:
                    print("ERROR: not touching! Still nothing ")
                    results['I'] = currentPoints
                    results['V'] = voltagePoints
                    results['R'] = 1e6
                    results['Rerr'] = 1e6
                    return results

            
            if power >= powerLimit or len(voltagePoints)>=points or i>=currentLimit:
                break
                
            if 0 < power < 1e-3*powerLimit:
                currentIncrement = 3*currentIncrement
                if debug:
                     print(f'Increment set to {currentIncrement}')
            else:
                currentPoints.append(i)
                if v<0:
                    v=0.0
                voltagePoints.append(v)
                if debug:
                    print(f"Adding: {i*1e3} mA \t{v*1e3} mV \t {power*1e3} mW")

            i = np.around(i+currentIncrement, decimals=6)
            
        results['I'] = currentPoints
        results['V'] = voltagePoints
        r = 1e6
        err = 1e6
        line = lambda x,a,b: a*x + b
        try:
            popt, pcov = curve_fit(line, currentPoints, voltagePoints)
            r = popt[0]
            err = pcov[0,0]
            results['R'] = r
            results['Rerr'] = err
        except:
            print("Could not fit IV curve")
            print(err)
            results['R'] = 1e6
            results['Rerr'] = 1e6
        if debug:
            print(r, " +- ", err )
        self.off()

        return results
    
    def precisemeas(self, repeat=10, units=1e-6, debug=False, skip=50):
        for i in range(skip):
            self.meascurr(units,debug)
            sleep(0.1)
        meas = np.zeros(repeat)
        for i in range(repeat):
            meas[i] = self.meascurr(units,debug)
            sleep(0.2)
        return np.average(meas), np.std(meas)


    def reset(self): 
        return self.keithley.write("*RST")

    def on(self):
        self.keithley.enable_source()

    def off(self):
        self.keithley.disable_source()

    # def __del__(self):
    #    self.keithley.write(":OUTP OFF")     # turn off
    #    self.keithley.write("SYSTEM:KEY 23") # go to local control
    #    self.keithley.close()
