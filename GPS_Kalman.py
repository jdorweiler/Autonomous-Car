'''
Reads in serial data from wireless transmitter data stream.  Parces the GPS, Velocity, Heading, and loop time.
The GPS position and velocity is sent through a Kalman filter (code adapted from the Kalman lecture CS373 by Prof. Thurn, Udacity.com) to update the cars position.  Calculates the azimuth heading and distance to a GPS waypoint
'''
import serial
from math import *
import csv
import datetime
import time
import numpy as np
import matplotlib.pyplot as plt

class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)

########################################

def filter(x, P):
    for n in range(len(measurements)):
        
        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
        
        # measurement update
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P
    
    #print 'x= '
    #x.show()
    #print 'P= '
    #P.show()
    
    return x.value[0][0], x.value[1][0]
########################################

def bearing(lat1, lon1, lat2, lon2):
    # convert decimal degrees to radians 
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    # haversine formula 
    dlon = lon2 - lon1
    dlat = lat2 - lat1 
    a = atan2(sin(dlon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))
    angle = ((a*(180./pi))+360.)%360
    return angle

def distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lat1, lon1, lat2, lon2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + (cos(lat1) * cos(lat2) * sin(dlon/2)**2)
    c = 2*atan2(float(sqrt(a)),float(sqrt(1-a)))
    km = 6371 * c
    return km 
 

u = matrix([[0.], [0.], [0.], [0.]]) # external motion

P =  matrix([[1000, 0., 0., 0.],
             [0., 1000, 0., 0.],
             [0., 0., 1000000, 0.],
             [0., 0., 0., 1000000]])



H =  matrix([[1., 0., 0., 0.],
             [0., 1., 0., 0.]])

R =  matrix([[0.2, 0.],
             [0., 0.1]])

I =  matrix([[1., 0., 0., 0.],
             [0., 1., 0., 0.],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]])

initial_x = 0

initial_y = 0

waypoints = [40.50574,-74.432558]

ser = serial.Serial('/dev/ttyUSB0', 57600, timeout = 30)

while True:
        content = ser.readline()
        t = [int(i) for i in content.split(',')]
        print ' *****************'
        
        print t
        lat = t[0]
        lat = (lat/1000000)+(((lat/10000)%100)/60.)+((((lat/10000.)%1)*100)/3600)
        lon = t[1]
        lon = -((lon/1000000)+(((lon/10000)%100)/60.)+((((lon/10000.)%1)*100)/3600))
        dt = t[2]
        heading = t[4]
        velx = (t[3]/2)*0.514444444 #assuming x and y have same velocity, m/s
        vely = velx
        
        F =  matrix([[1., 0., dt, 0.],[0., 1., 0., dt],[0., 0., 1., 0.],[0., 0., 0., 1.]])
        
        measurements = [[lat,lon]]

        print 'Measured position [lat,lon] ', measurements
        print 'Velocity (x,y)              ','(',velx,',',vely,')'
        
        x = matrix([[initial_x], [initial_y], [velx], [vely]])
        
        (initial_x, initial_y) = filter(x, P)
        
        print 'Updated position [lat,lon]  ', initial_x, ',', initial_y
        

        angle = bearing(lat,lon,waypoints[0],waypoints[1])

        print 'Current bearing             ', heading
        print 'Bearing to next waypoint    ' ,angle
        dist = distance(lat,lon,waypoints[0],waypoints[1])
        print 'Distance to waypoint        ', dist
        print ' *****************'
        #ser.open()
        #ser.write("angle")
        

        





