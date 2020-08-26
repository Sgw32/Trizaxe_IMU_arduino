import argparse
import struct
import numpy as np
import datetime
import binascii

parser = argparse.ArgumentParser(description='Process IMU Trizaxe telemetry.')
parser.add_argument('file')
args = parser.parse_args()

started = 0
cnt=0;
records = 0

b1 = 0;
b2 = 0;
b3 = 0;
b4 = 0;

timestamp = 0;
acc = [0,0,0];
gyro = [0,0,0];
temp = 0;
pitch = 0;
roll = 0;
speed = 0;
coarse = 0;
tempr = 0;
checksum = 0;

filename = datetime.datetime.now().strftime("%d%m%Y%HH%MM%SS") + ".log"
fw = open(filename, "a") 

def output(data,file):
    """
    Вывод в фаил log строки с элементами полученного на вход массива
    """
    file.write(data)

def toint(LowByte, HighByte):
    """
    Перевод старшего и младшего байта в int
    """
    #return int.from_bytes([HighByte, LowByte], byteorder='big', signed=True)
    barray = bytearray([HighByte, LowByte])
    n = int(binascii.hexlify(barray), 16)
    if(n & 0x8000):
        return -0x10000+n
    return n;

def touint(LowByte, HighByte):
    """
    Перевод старшего и младшего байта в int
    """
    #return int.from_bytes([HighByte, LowByte], byteorder='big', signed=True)
    barray = bytearray([HighByte, LowByte])
    n = int(binascii.hexlify(barray), 16)
    return n;

with open(args.file, "rb") as f:
    byte = f.read(1)
    while byte:
        # Do stuff with byte.
        curb = int.from_bytes(byte, byteorder='big')
        if not started: 
            if curb==0xAB:
                #a = input()
                started = 1
                cnt=0
                checksum = 0;
        if started:     
            
            if cnt==1:
               b1=curb
            if cnt==2:
               b2=curb
            if cnt==3:
               b3=curb
            if cnt==4:
               b4=curb
               timestamp = int.from_bytes([b4, b3, b2, b1], byteorder='big')
               #print(timestamp)
            if cnt==5:   
               b1 = curb 
            if cnt==6:
               b2 = curb
               acc[0] = toint(b1,b2)
               #print(acc[0])
            if cnt==7:
               b1=curb  
            if cnt==8:
               b2 = curb
               acc[1] = toint(b1,b2)
               #print(acc[1])
            if cnt==9:
               b1 = curb
            if cnt==10:
               b2 = curb
               acc[2] = toint(b1,b2)
               #print(acc[2])
            if cnt==11:   
               b1 = curb 
            if cnt==12:
               b2 = curb
               gyro[0] = toint(b1,b2)
            if cnt==13:
               b1=curb  
            if cnt==14:
               b2 = curb
               gyro[1] = toint(b1,b2)
            if cnt==15:
               b1 = curb
            if cnt==16:
               b2 = curb
               gyro[2] = toint(b1,b2)
            if cnt==17:
               b1 = curb
            if cnt==18:
               b2 = curb
               pitch = toint(b1,b2)
            if cnt==19:
               b1 = curb
            if cnt==20:
               b2 = curb
               roll = toint(b1,b2)
            if cnt==21:
               b1 = curb
            if cnt==22:
               b2 = curb
               speed = toint(b1,b2)
            if cnt==23:
               b1 = curb
            if cnt==24:
               b2 = curb
               coarse = toint(b1,b2)
            if cnt==25:
               b1 = curb
            if cnt==26:
               b2 = curb
               tempr = toint(b1,b2)
               
            cnt += 1;
            checksum ^= curb;
            #print('checksum ', checksum)
            if (cnt==28):
                if (checksum==0):
                    output(str(timestamp)+' ',fw)
                    output(str(acc[2])+' ',fw)
                    output(str(acc[1])+' ',fw)
                    output(str(acc[0])+' ',fw)
                    output(str(gyro[2])+' ',fw)
                    output(str(gyro[1])+' ',fw)
                    output(str(gyro[0])+' ',fw)
                    output(str(pitch)+' ',fw)
                    output(str(roll)+' ',fw)
                    output(str(speed/100.0)+' ',fw)
                    output(str(coarse/10.0)+' ',fw)
                    output(str(tempr)+' ',fw)
                    output('\n',fw)
                    records += 1    
                    started = 0
                else:
                    started = 0
        byte = f.read(1)
print(timestamp)
print(records)
print("Test ok");
        
