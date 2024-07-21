#!/usr/bin/python3

import time
import threading
import serial
import socket
import netifaces
import math
import numpy as np


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
    global b1
    global b2

    UNI1 = 0
    UNI2 = 1

    NUMLED = 25

    lukas1 = 0
    lukas2 = 0

    t1 = 0.66

    c1 = np.array([255,0,0])
    c2 = np.array([0,0,255])

    broadcast_ips = get_broadcast_addresses() 
    BCIP = broadcast_ips[0]
    port=6454

    header = "Art-Net\x00"
    op_code = (0x5000).to_bytes(2, byteorder='little')
    protocol_version = (14).to_bytes(2, byteorder='big')
    sequence = (0).to_bytes(1, byteorder='big')
    physical = (0).to_bytes(1, byteorder='big')
    universe1 = (UNI1).to_bytes(2, byteorder='little')
    universe2 = (UNI2).to_bytes(2, byteorder='little')
    length = (512).to_bytes(2, byteorder='big')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


    dmx_data1 = bytearray(512)
    dmx_data1_ptr = memoryview(dmx_data1)

    dmx_data2 = bytearray(512)
    dmx_data2_ptr = memoryview(dmx_data2)

    arr = np.zeros((NUMLED,3))

    strobe  = np.zeros(4)
    ambient = np.zeros((4,12,3))


    while True:
        time.sleep(0.01)

        if b1 == 1:
            b1 = 0
            lukas1 = lukas1+(1.5-lukas1)*0.15     
        if b2 == 1:
            b2 = 0
            lukas2 = lukas2+(1.5-lukas2)*0.15   

        lukas1 = max(0,lukas1-0.01)
        lukas2 = max(0,lukas2-0.01)

        arr = np.zeros((NUMLED,3))

        arr[int(t1*NUMLED),:] += np.array([50,50,50])
        arr[int(NUMLED-1),:] += np.array([255,255,255])

        height = lukas1+lukas2

        if height > 0:
            c = min(2*lukas1/height,1) * c1 + min(2*lukas2/height,1) * c2
            bright = min(0.1 + height,1)
            fill_arr(arr,0,height,c*bright)


        arr = arr.clip(0,255)
        arr = np.flip(arr,0)

        a = bytearray(arr.reshape((NUMLED*3,1)).astype(np.uint8).tobytes())
        dmx_data1_ptr[0:NUMLED*3] = a

        #a = bytearray(arr2.astype(np.uint8).tobytes())
        #dmx_data1_ptr[0:NUMLED*3] = a

        packet  = bytearray(header.encode('utf-8')) + op_code + protocol_version + sequence + physical + universe1 + length + dmx_data1
        #packet2 = bytearray(header.encode('utf-8')) + op_code + protocol_version + sequence + physical + universe2 + length + dmx_data2
        sock.sendto(packet, (BCIP, port))
        #sock.sendto(packet2, (BCIP, port))


    sock.close()


threading.Thread(target=button_thread).start()
threading.Thread(target=main_thread).start()
