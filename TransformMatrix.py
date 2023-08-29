import numpy as np


class TransformMatrix:
  def __init__(self):
    self.calibrationDone = False
    self.transfMatrix2D = np.eye(2)
    self.transfMatrix3D = np.eye(3)
  
  def Reset(self):
    self.transfMatrix2D = np.eye(2)
    self.transfMatrix3D = np.eye(3)

  def Transform(self, position):
    if len(position)!=2:
      print("Only 2 coordinates needed!")
      return 
    sensorPositioninPlane = self.transfMatrix2D.T@position
    #back to 3 dimensions
    sensorPositioninPlane = np.array([sensorPositioninPlane[0], sensorPositioninPlane[1], 0])
    return self.transfMatrix3D.T@sensorPositioninPlane

  def Calibrate(self, calibrationList, measuredList, weighted=False):
    if np.all(measuredList[0] == [0,0,0] and calibrationList[0] == [0,0] and len(calibrationList) == len(measuredList) and len(calibrationList)>2):
      matrices2D = []
      matricesForZ = []
      weights = []

      # k,n,m are the index of 3 points (k is origin)
      k = 0
      for n in range(1,len(calibrationList)):
        for m in range(1,len(calibrationList)):
            if n == m:
              continue
            print(f'n: {n}\tm: {m}')
            print(measuredList[k],measuredList[m],measuredList[n])
            #Find unit vector normal to the plane of the chuck
            normPlane = np.cross(measuredList[m],measuredList[n])
            if abs(np.linalg.norm(normPlane)) == abs(normPlane[2]):
              # all points have same Z, no need to use 3 dimensions
                MPlane = np.eye(3)
                print("all points have same Z, no need to use 3 dimensions")
            else:
                #Find matrix to rotate the chuck plane to be horizontal (matrix that overlaps normPlane to Z axis)
                if normPlane[2]<0:
                  continue
                normPlane = normPlane / np.linalg.norm(normPlane)
                Zaxis = np.array([0,0,1])
                v = np.cross(normPlane, Zaxis)
                c = np.dot(normPlane, Zaxis)
                s = np.linalg.norm(v)
                kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                MPlane = np.eye(3) + kmat + kmat.dot(kmat) / (1 + c)

            # Now we can play in 2 dimensions (in the plane of the sensor with z=0)
            CinPlane = [(MPlane@measuredList[k])[:2],(MPlane@measuredList[n])[:2],(MPlane@measuredList[m])[:2]]
            SinPlane = [calibrationList[k], calibrationList[n], calibrationList[m]]
            #Find matrix to trasform S in C
            try:
                rotMatrix = np.linalg.solve(SinPlane[1:],CinPlane[1:])
                print("rotMatrix:")
                print(rotMatrix)
                matrices2D.append(rotMatrix)
                matricesForZ.append(MPlane)
                weights.append(calibrationList[n][0]**2 + calibrationList[n][1]**2 + calibrationList[m][0]**2 + calibrationList[m][1]**2)
            except:
                pass
        

      #average of all matrices:
      if len(matrices2D)==0:
        print("Calibration not possible")
        self.calibrationDone = False
      elif len(matrices2D)==1:
        self.transfMatrix2D = matrices2D[0]
        self.transfMatrix3D = matricesForZ[0]
        self.calibrationDone = True
      elif weighted:
          self.transfMatrix2D = np.average(matrices2D, weights=weights, axis=0)
          self.transfMatrix3D = np.average(matricesForZ, weights=weights, axis=0)
          self.calibrationDone = True
      else:
          self.transfMatrix2D = np.average(matrices2D, axis=0)
          self.transfMatrix3D = np.average(matricesForZ, axis=0)
          self.calibrationDone = True

      
      print(self.transfMatrix2D)
      print(self.transfMatrix3D)
      print("Finished Calibration")
    else:
      print('First point should be origin and calib points should be 2 dimensional')



if __name__ == '__main__':    
  # p = [[0,0],[1,1],[2,2],[1,0],[0,1],[2,0],[0,2]]
  # m2 = [[0,0,0],[1,1,.1],[2,2,.2],[1,0,0],[0,1,.1],[2,0,0],[0,2,0.2]]
  p = [[0, 0], [40.0, 0.0], [0.0, 40.0]]
  m2 =[[0.0, 0.0, 0], [39.7, -0.2, 0], [0.4, 39.7, 0]]
  mat = TransformMatrix()
  mat.Calibrate(p,m2)
  print('\n\n')
  print(mat.Transform([0,2]))
