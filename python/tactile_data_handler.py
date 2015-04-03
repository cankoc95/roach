#!/usr/bin/env python
"""
author: jgoldberg

"""
import numpy as np
import shared_multi as shared
import time
from struct import pack,unpack

singlepixel = 'A'
fullFrame = 'B'
pollpixel = 'C'
streamFrame = 'E'
skinSize = 'G'
streaming = 'S'

bpack = 0

def handlePacket(src_addr, data):
    global bpack
    global mins
    global maxes

    packet_type = data[0]
    payload_length = ord(data[1])

    if packet_type == singlepixel:
        temp = map(ord,data)
        print "For pixel ["+str(temp[2])+","+str(temp[3])+"] value = "+str(temp[4]+temp[5]*256)

    if packet_type == skinSize:
        for r in shared.ROBOTS:
            if r.DEST_ADDR_int == src_addr:
                r.rows = ord(data[2])
                r.cols = ord(data[3])
                print "Shell dimensions received: ROWS =", r.rows, "COLUMNS =", r.cols
                mins = np.ones(r.rows*r.cols) * 200
                maxes = np.ones(r.rows*r.cols) * 4024

    if packet_type == pollpixel:
        temp = map(ord,data)
        print "Pixel ["+str(temp[2])+","+str(temp[3])+"]:"
        for i in range((payload_length-2)/2):
            print str(i)+"\t"+str(temp[i*2+4]+temp[i*2+5]*256)

    if packet_type == 'Y':
        temp = map(ord,data)
        print "val =",temp[2]+temp[3]*256

    if packet_type == 'F':
        forces = unpack('<6f', pack('24c',*data[2:]))
        print "\nforces:",forces

    if packet_type == 'L':
        f = unpack('<f', data[2]+data[3]+data[4]+data[5])[0]
        for r in shared.ROBOTS:
            if r.DEST_ADDR_int == src_addr:
                print "sent",r.f,"rec",f
                print "delta =",r.f-f

    if packet_type == streaming:
        if ord(data[2]):
            print "SkinProc streaming has been turned ON"
        else:
            print "SkinProc streaming has been turned OFF"

    if packet_type == fullFrame or packet_type == streamFrame:
        #print "received B packet", bpack
        bpack = bpack + 1

        for r in shared.ROBOTS:
            if r.DEST_ADDR_int == src_addr:
                if r.rows == 0 or r.cols == 0:
                    print "ERROR: Row and column size hasn't been set"
                    return
                ROWS = r.rows
                COLS = r.cols
                N = r.N
        temp = map(ord, data)
        temp = np.uint8(temp)
        timeStamp = temp[-4]+temp[-3]*256+temp[-2]*256*256+temp[-1]*256*256*256
        print "time =",timeStamp/1000000.0,"frequency =",(1000000.0/(timeStamp-shared.prevStamp))
        shared.prevStamp = timeStamp
        temp = temp[:-4]
        frame = temp[2:-1:2] + (temp[3::2]*256)

        print frame

        A = np.array([frame[0],np.power(frame[0],2),np.power(frame[0],3),
            frame[1],np.power(frame[1],2),np.power(frame[1],3),
            frame[2],np.power(frame[2],2),np.power(frame[2],3),
            frame[3],np.power(frame[3],2),np.power(frame[3],3),
            frame[4],np.power(frame[4],2),np.power(frame[4],3),
            frame[5],np.power(frame[5],2),np.power(frame[5],3),
            frame[6],np.power(frame[6],2),np.power(frame[6],3),
            frame[7],np.power(frame[7],2),np.power(frame[7],3)])
        F = A.dot(N)
        print("Fx:%.4f Fy:%.4f Fz:%.4f Froll:%.4f Fpitch:%.4f Fyaw:%.4f"%(F[0],F[1],F[2],F[3],F[4],F[5]))

        return

        # normalization
        newframe = np.zeros(ROWS*COLS)
        for i in range(ROWS*COLS):
            if frame[i] < mins[i]:
                mins[i] = frame[i]
            elif frame[i] > maxes[i]:
                maxes[i] = frame[i]
            newframe[i] = (frame[i] - mins[i]) / (maxes[i] - mins[i])
        
        print("    %4.f      :    :      %4.f    " % (frame[11],frame[6]))
        print("%4.f    %4.f  :    :  %4.f    %4.f" % (frame[9],frame[13],frame[4],frame[0]))
        print("    %4.f      :    :      %4.f    " % (frame[15],frame[2]))
        print
        print("    %.2f      :    :      %.2f    " % (newframe[11],newframe[6]))
        print("%.2f    %.2f  :    :  %.2f    %.2f" % (newframe[9],newframe[13],newframe[4],newframe[0]))
        print("    %.2f      :    :      %.2f    " % (newframe[15],newframe[2]))
        
        shared.zvals = [newframe[0],newframe[2],newframe[4],newframe[6],newframe[9],newframe[11],newframe[13],newframe[15]]
        
        np.set_printoptions(precision=3,suppress=True)
        
        '''
        dist1 = 1/((frame[0]+794.39)/7326.6)
        dist2 = 1/((frame[2]+989.47)/8617)
        dist3 = 1/((frame[4]+793.08)/7328.4)
        dist4 = 1/((frame[6]+1074.3)/9582.8)
        print
        print("%.3f" % dist1)
        print("%.3f" % dist2)
        print("%.3f" % dist3)
        print("%.3f" % dist4)

        A = np.array([[8.9127,-4.4563,0,-4.4563],[0,1.5954,-3.1908,1.5954],[0,0.5,0,0.5]])
        x = np.array([dist1,dist2,dist3,dist4])
        np.set_printoptions(precision=3,suppress=True)
        xyz0 = A.dot(x)
        print xyz0

        dist5 = 1/((frame[9]+945.28)/8536.8)
        dist6 = 1/((frame[11]+1118.8)/9611.3)
        dist7 = 1/((frame[13]+881.82)/8049)
        dist8 = 1/((frame[15]+892.76)/8547.4)
        print
        print("%.3f" % dist5)
        print("%.3f" % dist6)
        print("%.3f" % dist7)
        print("%.3f" % dist8)

        A = np.array([[-8.9127,4.4563,0,4.4563],[0,-1.5954,3.1908,-1.5954],[0,0.5,0,0.5]]) #using same cal values as left
        x = np.array([dist5,dist6,dist7,dist8])
        xyz1 = A.dot(x)
        print xyz1

        shared.xyzvals = [xyz0[0],xyz0[1],xyz0[2],xyz1[0],xyz1[1],xyz1[2]]
        '''

        #FOR ENTIRE ARRAY AND 6-DOF
        '''
        dist1 = 1/((frame[0]+515.18)/5876.8)
        dist2 = 1/((frame[2]+500.3)/6171.6)
        dist3 = 1/((frame[4]+425.45)/5967.9)
        dist4 = 1/((frame[6]+590.64)/7028.5)
        dist5 = 1/((frame[9]+366.96)/5398.3)
        dist6 = 1/((frame[11]+569.35)/6501.6)
        dist7 = 1/((frame[13]+449.9)/6144)
        dist8 = 1/((frame[15]+448.44)/6253.5)
        '''

        dist1 = 1.0/((frame[0]+594.68)/7276.3)
        dist2 = 1.0/((frame[2]+868.71)/9058.7)
        dist3 = 1.0/((frame[4]+1000.2)/9529.5)
        dist4 = 1.0/((frame[6]+941.52)/10029.0)
        dist5 = 1.0/((frame[9]+1038.9)/9763.0)
        dist6 = 1.0/((frame[11]+1078.5)/9985.2)
        dist7 = 1.0/((frame[13]+774.43)/8176.5)
        dist8 = 1.0/((frame[15]+1062.4)/10272.0)
        #print
        #print("    %.3f     :    :     %.3f    " % (dist6,dist4))
        #print("%.3f    %.3f:    :%.3f    %.3f" % (dist5,dist7,dist3,dist1))
        #print("    %.3f     :    :     %.3f    " % (dist8,dist2))

        #print dist1,dist2,dist3,dist4,dist5,dist6,dist7,dist8
        A = np.array([[8.9127,-4.4563,0,-4.4563],[0,1.5954,-3.1908,1.5954],[0,0.5,0,0.5]])
        x = np.array([dist1,dist2,dist3,dist4])
        xyz0 = A.dot(x)
        #print
        #print xyz0
        A = np.array([[-8.9127,4.4563,0,4.4563],[0,-1.5954,3.1908,-1.5954],[0,0.5,0,0.5]]) #using same cal values as left
        x = np.array([dist5,dist6,dist7,dist8])
        xyz1 = A.dot(x)
        #print xyz1
        shared.xyzvals = [xyz0[0],xyz0[1],xyz0[2],xyz1[0],xyz1[1],xyz1[2]]

        '''
        A = np.array([[-0.5,0.30357,0,0.30357,0.5,-0.30357,0,-0.30357],
            [0,-0.19643,0.5,-0.19643,0,0.19643,-0.5,0.19643],
            [0,-0.16667,-0.16667,-0.16667,0,-0.16667,-0.16667,-0.16667],
            [0,-0.33333,0,0.33333,0,0.33333,0,-0.33333],
            [0,0.071429,0,0.071429,0,-0.071429,0,-0.071429],
            [-0.11765,0.039216,0.039216,0.039216,-0.11765,0.039216,0.039216,0.039216]
            ])'''
        
        l1 = 8.5
        l2 = 7.0
        l3 = 5.5
        l4 = 1.5
        yscale = .1122
        xscale = .3134

        A = np.array([[0,yscale,1,0,-l1/2,l1/2*yscale], #for printed piece
            [0,0,1,-l4/2,-l2/2,0],
            [xscale,0,1,0,-l3/2,0],
            [0,0,1,l4/2,-l2/2,0],
            [0,-yscale,1,0,l1/2,l1/2*yscale],
            [0,0,1,l4/2,l2/2,0],
            [-xscale,0,1,0,l3/2,0],
            [0,0,1,-l4/2,l2/2,0]])
        A = np.linalg.pinv(A)

        x = np.array([dist1,dist2,dist3,dist4,dist5,dist6,dist7,dist8])
        xyzrpy = A.dot(x)
        #print("x:%.3f y:%.3f z:%.3f roll:%.3f pitch:%.3f yaw:%.3f"%(xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5]))
        

        shared.xyzrpy = [xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5]]

        #record all data
        for r in shared.ROBOTS:
            if r.DEST_ADDR_int == src_addr and r.RECORDSHELL:
                timenow = '%.6f' % time.time()
                dump_data = np.array([frame[0],frame[2],frame[4],frame[6],frame[9],frame[11],frame[13],frame[15],xyzrpy[0],xyzrpy[1],xyzrpy[2],xyzrpy[3],xyzrpy[4],xyzrpy[5]])
                #myCsvRow = timenow
                myCsvRow = str(timeStamp)
                for i in range(len(dump_data)):
                    myCsvRow = myCsvRow + "," + str(dump_data[i])
                myCsvRow = myCsvRow + "\n"
                #print myCsvRow
                fd = open("tactile_dump.csv","a")
                #np.savetxt(fd , dump_data, '%f',delimiter = ',')
                fd.write(myCsvRow)
                fd.close()


