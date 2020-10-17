
# This is a simple example of using Kalman filter for fusing and filtering 2 sensor readings.
# Temperature and Humudity data is captured using two DHT11 sensors and read by the program.
# Kalman filter has two steps, prediction and update.
# Then filtering and fusing works in following steps:
#       1. Initialze the Filtered value based on value from one sensor
#       2. For each new data point received:
#               a. perform update using sensor 1 value
#               b. perform update using sensor 2 value
#               c. predict the new filtered value
# So basically what we do here is picking up data points from two distributions for sensor 1 and sensor 2 data
# And perform filtering considering the statistical nature of each sensor value
# In other words, filter automatically detects the noisiness of the sensors and predict a good estimate based on the confidence level of sensors


import serial
import time

#This is used to open the serial port and start communication
def openSerial(comport):
    s = serial.Serial(comport, baudrate=115200, timeout = 0, write_timeout = 0)
    s.flushInput()
    print(s.name)
    time.sleep(0.5)
    return s

#Close the com port at the end
def closeSerial(s):
        s.close()

#Following are the system variables for temperature and humidity
X = [0,0]       # State
P = [1,1]       # Covariance (Since we are dealing with 1D data, covariance is 1)
Q = [0.9,0.7]   # System noise (This can be changed to fine tune the prediction value)
F = [1,1]       # Transition matrix (Again 1, because of 1D)
H = [1,1]       # Observation matrix (We observed the parameter directly)

#Initialze the kalman filter to sensor 1 value
def init_kalman(X, y):
    X = y
    P = 1
    print("In Kalman Init")
    return [X, P]

#Prediction step of the Kalman filter
def prediction(X, P, Q, F):
    X = F*X
    P = F*P*F + Q
    return [X, P]

#Updation step of the kalman filter
def update(X, P, y, H):
    Inn = y - H*X
    S = H*P*H + y
    K = P*H/S

    X = X + K*Inn
    P = P - K*H*P
    return [X, P]


start = True    # Used to initilize the Filters at the first time step

#This is run for each sensor readings we receive
def readSerial(s):
        global start,X,P,Q,F,H
        try:
                if s.inWaiting() > 12:

                        #Extract the sensor values
                        ard = s.readline().decode('utf-8')
                        [t1, t2, h1, h2] = ard.split(',')
                        [t1, t2, h1, h2] = [float(t1.strip()), float(t2.strip()), float(h1.strip()), float(h2.strip())]

                        # And if the data is not blank
                        if not ard == '':
                                # temperature fusion & humidity fusion
                                #At start initialize the kalman filter
                                if start == True:
                                        [X[0], P[0]] = init_kalman(X[0], t1) # initialize the state using the 1st sensor
                                        [X[1], P[1]] = init_kalman(X[1], h1) # initialize the state using the 1st sensor
                                        print("Start")
                                        start = False

                                #There after perform predict and update steps
                                else:
                                        print("Predict")
                                        [X[0], P[0]] = prediction(X[0], P[0], Q[0], F[0])
                                        [X[1], P[1]] = prediction(X[1], P[1], Q[1], F[1])

                                        print("update")
                                        [X[0], P[0]] = update(X[0], P[0], t1, H[0])
                                        [X[0], P[0]] = update(X[0], P[0], t2, H[0])
                                        [X[1], P[1]] = update(X[1], P[1], h1, H[1])
                                        [X[1], P[1]] = update(X[1], P[1], h2, H[1])


                                # Uncomment any one line of this plot the respective values
                                print("(",X[0],",",X[1],")")            #Check Filtered values
                                #print("(",t1,",",t2,",",X[0],")")      #Just check temperature variation
                                #print("(",h1,",",h2,",",X[1],")")      #Just check humiditye variation

        except:
                print("Error")

s = openSerial('COM3')

while(True):
    readSerial(s)