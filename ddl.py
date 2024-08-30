#!/usr/bin/python3

import time
import threading
import serial
import socket
import netifaces
import math
import numpy as np
import random


b1 = 0
b2 = 0


def get_broadcast_addresses():
    broadcast_addresses = []
    # Get all network interfaces
    for interface in netifaces.interfaces():
        # Get addresses associated with each interface
        addresses = netifaces.ifaddresses(interface)
        # Check for IPv4 addresses
        if netifaces.AF_INET in addresses:
            for link in addresses[netifaces.AF_INET]:
                # Get the broadcast address and add it to the list
                if 'broadcast' in link:
                    broadcast_addresses.append(link['broadcast'])
    return broadcast_addresses



def button_thread():
    global b1
    global b2

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    last1 = time.time()
    last2 = time.time()

    while True:
        try:
            line = ser.readline()
            t = time.time()
            pressed_button = line[0]

            if (pressed_button == 49) and ((t - last1)>0.1):
                b1 = 1
                last1 = t
            if (pressed_button == 50) and ((t - last2)>0.1):
                b2 = 1
                last2 = t

        except:
            pass




def fill_arr(arr,start,length,color):
    if length > 0:
        NUMLED = arr.shape[0]

        start = start * NUMLED
        length = length * NUMLED
        stop = start + length

        start = min(start,NUMLED)
        stop = min(stop,NUMLED)

        iStart = math.floor(start)
        fractStart = start - iStart

        iStop = math.ceil(stop)
        fractStop = iStop - stop

        arr[iStart:iStop,:] += color
        arr[iStart,:] -= fractStart * color    
        arr[iStop-1,:] -= fractStop * color

    return arr



def main_thread():
    while True:
        try:
                
            global b1
            global b2

            UNI1 = 302
            UNI2 = 0
            UNI3 = 1

            A1 = 1
            A2 = 150

            NUMLED = 26

            lukas1 = 0
            lukas2 = 0

            t0 = 0.75
            t1 = 1

            c1 = np.array([255,0,0])
            c2 = np.array([0,0,255])

            strobe_hart = np.array([255,180,255,0])
            strobe_off = np.array([0,0,255,0])

            #broadcast_ips = get_broadcast_addresses() 
            #BCIP = broadcast_ips[0]
            port=6454

            header = "Art-Net\x00"
            op_code = (0x5000).to_bytes(2, byteorder='little')
            protocol_version = (14).to_bytes(2, byteorder='big')
            sequence = (0).to_bytes(1, byteorder='big')
            physical = (0).to_bytes(1, byteorder='big')
            universe1 = (UNI1).to_bytes(2, byteorder='little')
            universe2 = (UNI2).to_bytes(2, byteorder='little')
            universe3 = (UNI3).to_bytes(2, byteorder='little')
            length = (512).to_bytes(2, byteorder='big')

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


            dmx_data1 = bytearray(512)
            dmx_data1_ptr = memoryview(dmx_data1)

            dmx_data2 = bytearray(512)
            dmx_data2_ptr = memoryview(dmx_data2)

            dmx_data3 = bytearray(512)
            dmx_data3_ptr = memoryview(dmx_data3)

            arr = np.zeros((NUMLED,3))

            strobe  = np.zeros(4)
            ambient = np.zeros((8,24,3))


            while True:
                time.sleep(0.01)

                if b1 == 1:
                    b1 = 0
                    lukas1 = lukas1+(1.2-lukas1)*0.15
                if b2 == 1:
                    b2 = 0
                    lukas2 = lukas2+(1.2-lukas2)*0.15

                lukas1 = max(0,lukas1-0.005)
                lukas2 = max(0,lukas2-0.005)


                #lukas1 = 0.5
                #lukas2 = 0.4

                arr = np.zeros((NUMLED,3))
                strobe  = np.zeros(4)
                #ambient = np.zeros((8,24,3))

                #arr[int(t1*NUMLED),:] += np.array([50,50,50])
                #arr[int(NUMLED-1),:] += np.array([255,255,255])

                height = lukas1+lukas2

                strobe[1] = 220
                strobe[2] = 255
                strobe[3] = 0
                ambient *= 0.9


                if height == 0:
                    x = int(random.random()*8)
                    y = int(random.random()*24)

                    ambient[x,y] += c1

                    x = int(random.random()*8)
                    y = int(random.random()*24)

                    ambient[x,y] += c2
                else:
                    c = min(2*lukas1/height,1) * c1 + min(2*lukas2/height,1) * c2
                    bright = min(0.1 + height,1)
                    fill_arr(arr,0,height,c)
                                
                
                    if height > t0:
                        for i in range(8):
                            ambient[i] = fill_arr(ambient[i],0,(height-t0)/(1-t0),c)
                        #ambient[:,:] = c            
                        #strobe[3] = 250

                    if height > t1:
                        strobe[0] = 255
                    else:
                        strobe[0] = 0
                        


                ambient = ambient.clip(0,255)
                arr = arr.clip(0,255)



                arr = arr.reshape((2,int(NUMLED/2),3),order='F')
                down = arr[0]
                up = arr[1]
                down = np.flip(down,axis=0)
                arr = np.concatenate((down,up))

        



                a = bytearray(arr.reshape((NUMLED*3,1)).astype(np.uint8).tobytes())
                dmx_data1_ptr[0:NUMLED*3] = a
                dmx_data1_ptr[(NUMLED-1)*3:(NUMLED-1)*3+NUMLED*3] = a


                a = bytearray(strobe.astype(np.uint8).tobytes())

                dmx_data2_ptr[(A1-1):(A1-1)+4] = a
                dmx_data2_ptr[(A2-1):(A2-1)+4] = a

                dmx_data3_ptr[(A1-1):(A1-1)+4] = a
                dmx_data3_ptr[(A2-1):(A2-1)+4] = a

        
                ambient2=np.copy(ambient)
                left,right = np.split(ambient2,2,axis=0)

                l1, l2 = np.split(left,2,axis=1)

                l1 = np.flip(l1,axis=1)
                l2 = np.flip(l2,axis=1)

                l1[0] = np.flip(l1[0],axis=0)
                l1[2] = np.flip(l1[2],axis=0)
                l2[0] = np.flip(l2[0],axis=0)
                l2[2] = np.flip(l2[2],axis=0)

                r1, r2 = np.split(right,2,axis=1)

                r1 = np.flip(r1,axis=0)
                r2 = np.flip(r2,axis=0)

                r1[0] = np.flip(r1[0],axis=0)
                r1[2] = np.flip(r1[2],axis=0)
                r2[0] = np.flip(r2[0],axis=0)
                r2[2] = np.flip(r2[2],axis=0)


                a = bytearray(l1.astype(np.uint8).tobytes())
                dmx_data2_ptr[(A1-1)+4:(A1-1)+148] = a

                a = bytearray(l2.astype(np.uint8).tobytes())
                dmx_data2_ptr[(A2-1)+4:(A2-1)+148] = a

                a = bytearray(r1.astype(np.uint8).tobytes())
                dmx_data3_ptr[(A1-1)+4:(A1-1)+148] = a

                a = bytearray(r2.astype(np.uint8).tobytes())
                dmx_data3_ptr[(A2-1)+4:(A2-1)+148] = a
                
                

                packet  = bytearray(header.encode('utf-8')) + op_code + protocol_version + sequence + physical + universe1 + length + dmx_data1
                packet2 = bytearray(header.encode('utf-8')) + op_code + protocol_version + sequence + physical + universe2 + length + dmx_data2
                packet3 = bytearray(header.encode('utf-8')) + op_code + protocol_version + sequence + physical + universe3 + length + dmx_data3
                sock.sendto(packet, ('192.168.8.104', port))
                sock.sendto(packet2, ('192.168.8.214', port))
                sock.sendto(packet3, ('192.168.8.214', port))

                
            


            sock.close()
        except:
            time.sleep(1)


threading.Thread(target=button_thread).start()
threading.Thread(target=main_thread).start()
