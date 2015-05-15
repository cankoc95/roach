import random
import pygame
from pygame.locals import *
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from numpy import sin, cos
import sys, os
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

randomize = 0

verticies = (
    (5.5, -2, -0.1),
    (5.5, 2, -0.1),
    (-5.5, 2, -0.1),
    (-5.5, -2, -0.1),
    (5.5, -2, 0.1),
    (5.5, 2, 0.1),
    (-5.5, -2, 0.1),
    (-5.5, 2, 0.1)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )

surfaces = (
    (0,1,2,3),
    (4,5,7,6),
    (0,3,6,4),
    (1,2,7,5),
    (0,1,5,4),
    (2,3,6,7)
    )


def ReflectorUpdate():
    global reflector
    global l
    global w
    l = 5
    w = 1.5
    # +x is away from viewer parallel to board
    # +y is away from viewer perpendicular to board
    # +z is up
    #starts closest to viewer (bottom right when looking top down on robot) and defines corners clockwise
    reflector = np.array([[-l+dx,-w+dy,0],
        [-l+dx,w+dy,0],
        [l+dx,w+dy,0],
        [l+dx,-w+dy,0]])
    Ry = np.array([[cos(yaw),-sin(yaw),0],
        [sin(yaw),cos(yaw),0],
        [0,0,1]])
    Rp = np.array([[cos(pitch),0,sin(pitch)],
        [0,1,0],
        [-sin(pitch),0,cos(pitch)]])
    Rr = np.array([[1,0,0],
        [0,cos(roll),-sin(roll)],
        [0,sin(roll),cos(roll)]])
    reflector = reflector.dot(Rr.T)
    reflector = reflector.dot(Rp.T)
    reflector = reflector.dot(Ry.T)
    reflector = reflector + np.array([[0,0,z],[0,0,z],[0,0,z],[0,0,z]])
    '''
    reflector = (
        (-l-dy, -w+dx, z), #closest to viewer
        (-l-dy, w+dx, z),
        (l-dy, w+dx, z), #farthest from viewer
        (l-dy, -w+dx, z)
        )
    
    reflector = (
        (-l+dy-l*np.sin(yaw), -w-dx+w*np.cos(yaw), z),
        (-l+dy, w-dx, z),
        (l+dy, w-dx, z),
        (l+dy, -w-dx, z)
        )
    '''

def Randomize():
    global dx
    global dy
    global z
    global roll
    global pitch
    global yaw
    scale = 1
    offset = 0.2
    dx = 0#random.random()
    dy = 0#random.random()
    z = .4 #random.random()*scale+offset
    roll = 0
    pitch = 0
    yaw = 0
    
def Update():
    global dx
    global dy
    global z
    global roll
    global pitch
    global yaw
    
    print("x:%.3f y:%.3f z:%.3f roll:%.3f pitch:%.3f yaw:%.3f"%(shared.xyzrpy[0],shared.xyzrpy[1],shared.xyzrpy[2],shared.xyzrpy[3],shared.xyzrpy[4],shared.xyzrpy[5]))
    scale = 1 #.1 cm/mm
    dx = shared.xyzrpy[0]*scale
    dy = shared.xyzrpy[1]*scale
    z = shared.xyzrpy[2]*scale - 1
    roll = shared.xyzrpy[3]*scale
    pitch = shared.xyzrpy[4]*scale
    yaw = shared.xyzrpy[5]*scale

#arrays for moving average
index = 0
count = 10
dx_saved = np.zeros(count)
dy_saved = np.zeros(count)
z_saved = np.zeros(count)
roll_saved = np.zeros(count)
pitch_saved = np.zeros(count)
yaw_saved = np.zeros(count)

def MovingAverage():
    global dx
    global dy
    global z
    global roll
    global pitch
    global yaw
    global dx_saved
    global dy_saved
    global z_saved
    global roll_saved
    global pitch_saved
    global yaw_saved
    global index
    if index >= count:
        index = 0
    dx_saved[index] = dx
    dy_saved[index] = dy
    z_saved[index] = z
    roll_saved[index] = roll
    pitch_saved[index] = pitch
    yaw_saved[index] = yaw

    index = index + 1
    dx = np.mean(dx_saved)
    dy = np.mean(dy_saved)
    z = np.mean(z_saved)
    roll = np.mean(roll_saved)
    pitch = np.mean(pitch_saved)
    yaw = np.mean(yaw_saved)

def ForceCalc():
    x_disp = reflector[:,0]
    y_disp = reflector[:,1]
    z_disp = reflector[:,2]-2.936
    #print z_disp
    kz = .285 #N/mm
    Fz = sum(z_disp)*kz
    print("Z Force (N): %.3f" % Fz)
    x_force = ((z_disp[2]+z_disp[3])-(z_disp[0]+z_disp[1]))*100.0/2.0/((z_disp[0]+z_disp[1])+(z_disp[2]+z_disp[3]))
    print("Location in x (cm): %.3f" % (x_force/10.0))

def DrawBoard():
    glBegin(GL_QUADS)
    glColor3fv((0,1,0))
    for surface in surfaces:
        for vertex in surface:
            glVertex3fv(verticies[vertex])
    glEnd()
    
    glBegin(GL_LINES)
    glColor3fv((0,0,0))
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()
    
def DrawReflectors():
    glBegin(GL_LINES)
    glColor3fv((0,0,0))
    glVertex3fv(reflector[0])
    glVertex3fv(reflector[1])
    glVertex3fv(reflector[0])
    glVertex3fv(reflector[3])
    glVertex3fv(reflector[2])
    glVertex3fv(reflector[1])
    glVertex3fv(reflector[2])
    glVertex3fv(reflector[3])
    glEnd()
    
    glBegin(GL_QUADS)
    glColor3fv((1,1,1))
    for vertex in reflector:
        glVertex3fv(vertex)
    glEnd()

def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0,0, -15)

    glRotatef(60, -1, 0, 0.7)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    glTranslatef(-0.5,0,0)
                if event.key == pygame.K_RIGHT:
                    glTranslatef(0.5,0,0)

                if event.key == pygame.K_UP:
                    glTranslatef(0,1,0)
                if event.key == pygame.K_DOWN:
                    glTranslatef(0,-1,0)

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    #glTranslatef(0,0,1.0)
                    glRotatef(-10,0,0,1)

                if event.button == 5:
                    #glTranslatef(0,0,-1.0)
                    glRotatef(10,0,0,1)
        
        if randomize:
            Randomize()
        else:
            Update()
            MovingAverage()
        ReflectorUpdate()
        #ForceCalc()
        #glRotatef(1, 3, 1, 1)
        #glRotatef(1, 0, 0, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        DrawBoard()
        DrawReflectors()
        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == '__main__':
    randomize = 1
    main()
