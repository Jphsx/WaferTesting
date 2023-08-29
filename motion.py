from __future__ import print_function 
import serial
from time import sleep
import logging
import numpy as np

__logLevel__ = logging.WARNING


class motion:
    """
        Class to control the motion of the chuck at KU
        authors:    nicola.minafra@cern.ch
                    pacejohn@ku.edu
                    crogan@ku.edu
    """
    def __init__(self, port='COM4', timeout=0.1, log=None, emulate=False):
        if log is None:
            log=self.__class__.__name__+'Log.log'
        hdlr = logging.FileHandler(log)
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger = logging.getLogger()
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(__logLevel__)
        # logging.handlers.RotatingFileHandler.doRollover(hdlr)
        self.commandIndex = 0
        self.emulate = emulate
        if not self.emulate:
            self.ser = serial.Serial(port=port, baudrate=9600, bytesize=8, parity='N', timeout=timeout)
            self.ser.write(b'E')  # enable online mode
            self.ser.write(b'C,R,Q')  # clear current program
            self.ser.readline()
        self.motors = ['X', 'Y', 'Z']
        self.scale = {'X': 628.3, 'Y' : 630.3, 'Z': 163.2}
        self.timeout = timeout
        self.logger.info(f'{port} initialized')

        self.maxLimit = {'X': None, 'Y' : None, 'Z': None}
        self.minLimit = {'X': None, 'Y' : None, 'Z': None}

    def __del__(self):
        if not self.emulate:
            self.ser.write(b'Q,R')      # quit online mode
            self.ser.close()
        self.logger.info(f'{self.ser.port} closed')

    # delay until current program is done   
    def __wait(self):
        self.logger.info(f'Waiting...')
        if self.emulate:
            return
            
        #wait for controller to be ready
        readStr=str(self.ser.readline(),'ISO-8859-1')
        i = 0
        
        while('^' in readStr):
            sleep(self.timeout)
##            self.ser.write(b'C,R')
            readStr = str(self.ser.readline(),'ISO-8859-1')
            i += 1
        self.logger.info(f'Waited for {i*self.timeout} seconds')
        return
    
    def __move(self, motor, distance=0, position=0):
        motor = motor.upper()
        if motor not in self.motors:
            self.logger.error(f'Unknown motor: {motor}')
            return

        if self.emulate:
            return 0

        startPosition = self.getPosition(motor)
        if distance != 0:
            position = startPosition+distance
        
        if (self.minLimit[motor] is not None and position<self.minLimit[motor]) or (self.maxLimit[motor] is not None and position>self.maxLimit[motor]):
            print(f'ERROR! position {position} outside safety limits for motor {motor}: {self.minLimit[motor]}')
            return self.getPosition(motor)
        
        position = int(np.round(position * self.scale[motor]))

        self.ser.write(b'E,C,R')  # clear current program
        command_str = f'IA{self.motors.index(motor)+1}M{position},R'
        self.logger.info(f'move {motor} to {position}: {command_str}')
        self.ser.write(str.encode(command_str)) # byte-ify move_string and send it
        self.ser.write(b'R')
        self.__wait()
        self.ser.write(b'Q')  # disable online mode

        finalPosition = self.getPosition(motor)
        distanceRead = finalPosition - startPosition

        if distance != 0:
            return distanceRead
        else:
            return finalPosition

    def __limit(self):
        """
            Returns True if a motor just hit a limit switch
        """
        if self.emulate:
            return False
        self.ser.write(b'O1')
        self.logger.debug(self.ser.readline()==b'O')
        return (self.ser.readline()==b'O')

    def sendToLimit(self, motor=None, positive=False):    # DANGEROUS!!!
        """
            Moves motor util the positive/negative switch is hit
        """
        if motor is None:
            results = []
            for motor in self.motors:
                self.logger.info(f'Sending {motor} to limit')
                results.append(self.sendToLimit(motor))
                self.__wait()
            return results
        
        motor = motor.upper()
        if motor not in self.motors:
            self.logger.error(f'Unknown motor: {motor}')
            return
        if positive:
            polarity='0'
        else:
            polarity='-0'
        
        if self.emulate:
            return 
        
        self.ser.write(b'C,E')  # clear current program
        command_str = f'I{self.motors.index(motor)+1}M{polarity},R'
        self.ser.write(str.encode(command_str))

        self.__wait()
        self.getPosition(motor)
        self.ser.write(b'Q')

    




    def moveTo(self, motor=None, position=0, coordinates=['0','0','0']):
        """
			Sends to chick to the wanted position
			Returns the final position
            Usage:
            moveTo() #goes to [0,0,0] (home)
            moveTo('x') #goes to 0 for x, other directions unchanged
            moveTo('x', 10) #goes to 10 (mm from home) for x, other directions unchanged
            moveTo(coordinates=[10,20,50]) # goes to coordinates (in mm)
        """

        if motor is not None:
            return self.__move(motor, position=position)
        else:
            if len(coordinates) != 3:
                self.logger.error(f'wrong set of coordinates: {coordinates}')
                return
            results = []
            for i in range(len(coordinates)):
                results.append(self.moveTo(self.motors[i],coordinates[i]))
                self.__wait()               
            return results

    def moveFor(self, motor, distance):
        """
                Moves the chuck for the wanted distance (positive or negative)
                Returns the distance traveled
                WARNING: putting distance to 0 means travelling until the positive switch is hit
        """
        self.__move(motor, distance=distance)
        return

    def goHome(self, motor=None):
        """
                Sends to chuck to home position: [0,0,0]
        """
        if motor is None:
            for motor in self.motors:
                self.goHome(motor)
        elif motor not in self.motors:
            self.logger.error(f'Unknown motor: {motor}')
            return
        else:
            self.__move(motor)
        return self.getPosition(motor)
    
    def setHome(self):
        """
                Sets current position as coordinate origin (home)
        """
        if not self.emulate:
            self.ser.write(b'N')
        self.logger.info(f'motor positions: {self.getPosition()}')
        return
        
    def getPosition(self, motor=None):
        """d
                Returns current position of the chuck for a given motor (if specified) or for all
        """
        if motor is None:
            m = []
            for motor in self.motors:
                m.append(self.getPosition(motor))
            return m

        if self.emulate:
            print("Emulation mode ON")
            return [-1,-1,-1]

        motor = motor.upper()
        if motor not in self.motors:
            self.logger.error(f'Unknown motor: {motor}')
            return
        # self.__wait()                   # wait for motors to finish moving

        preiousPosition = 0.
        pos = 1.

        while pos != preiousPosition:
            preiousPosition = pos
            ##print('CHECK: ' + str(check))
            self.ser.readline()
            self.ser.write(b'C,R')
            self.ser.readline()
            self.ser.write(str.encode(motor))   # query for position
            readStr = self.ser.readline()
            try:
                pos_str = str(readStr, 'ISO-8859-1')
            except:
                print(readStr)
            while len(pos_str)<3:
                self.ser.write(str.encode(motor))
                pos_str = str(self.ser.readline(), 'ISO-8859-1')

            try:
                pos = float(pos_str[1:9]) / self.scale[motor]
            except Exception as e:
                print(e)
                pos = float(pos_str[2:9]) / self.scale[motor]
                print(f'Using this conversion: {pos_str} -> {float(pos_str[2:9])}')

            self.ser.write(b'Q')
            self.logger.info(f'{motor} position: {pos_str}')

        return pos        
        
    def stop(self):
        if not self.emulate:
            self.ser.write(b'D')
        
    def setSafetyLimit(self, motor, min=None, max=None):
        motor = motor.upper()
        if motor not in self.motors:
            self.logger.error(f'Unknown motor: {motor}')
            return
        if min is not None:
            self.minLimit[motor] = min
            print(f'min set to {min} for {motor}')
        if max is not None:
            self.maxLimit[motor] = max
            print(f'max set to {max} for {motor}')
            
    def resetSafetyLimit(self, motor, min=None, max=None):
        motor = motor.upper()
        if motor not in self.motors:
            self.logger.error(f'Unknown motor: {motor}')
            return
        if min is None:
            self.minLimit[motor] = None
            print(f'reset min for {motor}')
        if max is None:
            self.maxLimit[motor] = None
            print(f'reset max for {motor}')
            


            

if __name__ == '__main__':
    m = motion()
    print(m.getPosition())
    
       
    
        





