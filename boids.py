import math
from random import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.distance import squareform,pdist,cdist
import sys,argparse



width,height = 640,480
class Boids:
    def __init__(self,N):
        self.pos = [width/2,height/2] + 10 *np. random.rand(2 *N ).reshape(N,2)
        angles = 2 * math.pi * np.random.rand(N) 
        self.vel = np.array(list(zip(np.sin(angles),np.cos(angles))))
        self.maxVel = 60.0
        self.maxRuleValue = 0.7
        self.N = N
    
    def tick(self,frameNum,pts,beak):
        #distance matrix
        self.distMatrix = squareform(pdist(self.pos))
        #rules
        self.vel = self.applyRules()
        self.limit(self.vel,self.maxVel)
        self.pos += self.vel
        self.Boundary()
        #UPDATE data
        pts.set_data(self.pos.reshape(2*self.N)[::2],self.pos.reshape(2 * self.N)[1::2])

        vec = self.pos + 15 * self.vel/self.maxVel
        beak.set_data(vec.reshape(2*self.N)[::2],vec.reshape(2 * self.N)[1::2])

    def limitVec(self,vec,maxVal):
        mag = np.linalg.norm(vec)
        if mag > maxVal:
            vec[0],vec[1] = vec[0] * maxVal/mag,vec[1] * maxVal/mag

    def limit(self,X,maxVal):
        for Vec in X:
            self.limitVec(Vec,maxVal)



    def Boundary(self):
        deltaR = 2.0
        #STAYING WITHIN SCREEN
        for coord in self.pos:
            if coord[0] > width + deltaR:
                coord[0] = -deltaR
            if coord[0] < -deltaR:
                coord[0] = width + deltaR
            if coord[1] > height + deltaR:
                coord[1] = -deltaR
            if coord[1] < -deltaR:
                coord[1] = height + deltaR

    def applyRules(self):
        #separation
        D = self.distMatrix < 25.0
        vel = self.pos * D.sum(axis = 1).reshape(self.N,1) - D.dot(self.pos)
        self.limit(vel,self.maxRuleValue)
        #Alignment
        D = self.distMatrix < 50.0
        vel2 = D.dot(self.vel)
        self.limit(vel2,self.maxRuleValue)
        vel += vel2

        #cohesion
        vel3 = D.dot(self.pos) - self.pos
        self.limit(vel3,self.maxRuleValue)
        vel += vel3
        return vel

    def buttonPress(self,event):
        #left click to add
        if event.button == 1:
            self.pos = np.concatenate((self.pos,np.array([[event.xdata,event.ydata]])),axis=0)
            angles = 2 * math.pi * np.random.rand(1)
            v = np.array(list(zip(np.sin(angles),np.cos(angles))))
            self.vel = np.concatenate((self.vel,v),axis=0)
            self.N +=1
        
        elif event.button == 2:
            #scatter
            self.vel += 0.9 * (self.pos - np.array([[event.xdata,event.ydata]]))


def tick(frameNum,pts,beak,boids):
    boids.tick(frameNum,pts,beak)
    return pts,beak

def main():

    parser = argparse.ArgumentParser(description="Boidzz...")
    parser.add_argument('--num-boids',dest='N',required= False)
    args = parser.parse_args()

    N = 100
    if args.N:
        N = int(args.N)
    
    boids = Boids(N)
    #Plotting the fig
    fig = plt.figure()
    ax = plt.axes(xlim = (0,width),ylim  = (0,height))

    #Body and beak
    pts, = ax.plot([],[],markersize = 10,c = 'b',marker = 'o',ls ='None')
    beak, = ax.plot([],[],markersize = 4, c = 'r',marker = 'o',ls = 'None')
    
    #Animation
    anim = animation.FuncAnimation(fig,tick,fargs=(pts,beak,boids),interval = 50)
    cid = fig.canvas.mpl_connect('button_press_event',boids.buttonPress)
    plt.show()


main()