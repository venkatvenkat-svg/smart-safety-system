#!/usr/bin/python
# -*- coding: utf-8 -*-
# Importing modules
import spidev # To communicate with SPI devices
from time import sleep  # To add delay
import RPi.GPIO as GPIO # To use GPIO pins
# Start SPI connection
import os
import time 
from time import sleep
from datetime import datetime
import sys
import serial
import smtplib
from email.MIMEMultipart import MIMEMultipart
from email.MIMEBase import MIMEBase
from email.MIMEText import MIMEText
from email import Encoders
import os
import time
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BCM)
output=0
output2=0
b=27
switch=22
p=21
sw1=20
cpf=26
cpr=19
ve=13
ve1=6
GPIO.setup(switch, GPIO.IN)
GPIO.setup(sw1, GPIO.IN)
GPIO.setup(p, GPIO.IN)
GPIO.setup(b, GPIO.OUT)
GPIO.setup(cpf, GPIO.OUT)
GPIO.setup(cpr, GPIO.OUT)
GPIO.setup(ve, GPIO.OUT)
GPIO.setup(ve1, GPIO.OUT)
gmail_user = "ppradeeprajabe@gmail.com"
gmail_pwd = "pradeepfz"
# Enable Serial Communication
port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
GPIO.output(b,False)
GPIO.output(cpf,False)
GPIO.output(cpr,False)
GPIO.output(ve,True)
GPIO.output(ve1,True)

def mode1():
    print("   MODE1 ")
    ir=GPIO.input(p)
    print ("pir state=", ir)
    if ir==1 and bb=='0':     
        print("ir ok")
        bb='1'   
        sleep(1) 
	else:
        print("ir notdet")    
    if bb=='1':
        if ir==1 and count<10:
            print("irrrrrrrr")
            count=0
        else:
            count+=1  
            if count >10:
                GPIO.output(b,True)
                msg('4')
                
def msg(a):
    # Transmitting AT Commands to the Modem
    # '\r\n' indicates the Enter key
    port.write('AT'+'\r\n')
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
    port.write('ATE0'+'\r\n')      # Disable the Echo
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
    port.write('AT+CMGF=1'+'\r\n')  # Select Message format as Text mode 
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
    port.write('AT+CNMI=2,1,0,0,0'+'\r\n')   # New SMS Message Indications
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
    # Sending a message to a particular Number
     
    port.write('AT+CMGS="9491168559"'+'\r\n')
    rcv = port.read(10)
    print (rcv)
    time.sleep(1)
    if a=='1':
        port.write('Fall Detected'+'\r\n')  # Message
        rcv = port.read(10)
        print (rcv)
    elif a=='2':
        port.write('heartrate Abnormal'+'\r\n')  # Message
        rcv = port.read(10)
        print (rcv)
    elif a=='3':
        port.write('Respiratory Abnormal'+'\r\n')  # Message
        rcv = port.read(10)
        print (rcv)
    elif a=='4':
        port.write('Emergency alert'+'\r\n')  # Message
        rcv = port.read(10)
        print (rcv)
    port.write("\x1A") # Enable to send SMS
    for i in range(10):
        rcv = port.read(10)
        print (rcv)
    time.sleep(4)
    port.write('ATD9491168559;'+'\r\n')
    time.sleep(1)
l1=0
c=0
count=0
spi = spidev.SpiDev() # Created an object
spi.open(0,0)   
# Initializing LED pin as OUTPUT pin
#GPIO.setmode(GPIO.BOARD) 
GPIO.setmode(GPIO.BCM)
# Creating a PWM channel at 100Hz frequency
#pwm = GPIO.PWM(led_pin, 100)
#pwm.start(0) 
# Read MCP3008 data
print("          GETTING THE SAMPLE DATASET FROM THE USER ")
sleep(1)
i=0
c=1
m=0
hb=0
bp=0
output=0
output2=0
def analogInput(channel):  
    spi.max_speed_hz = 1350000
    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data
#sleep(6)
#GPIO.output(led_pin, GPIO.HIGH) #
data = serial.Serial('/dev/ttyS0', 9600, timeout = 1) # ttyACM1 for Arduino board
readOut1 = 0   #chars waiting from laser range finder
aa='0'
bb='0'
GPIO.output(b,False)
GPIO.output(ve,False)
while True:
    sleep(2)
    mode=GPIO.input(switch)
    m=GPIO.input(sw1)
    print ("sw1=", m)    
    if mode==0:
        print("NORAMAL_MODE")  

        print("Fall Detection")
        
        out = analogInput(0) # Reading from CH0
       
        print("lll",out)
        #pwm.ChangeDutyCycle(output)
        #sleep(1)
        if (out<190):
            
            print(" Abnormal  MEMS Detected")
            print("                           fall Detected")
            GPIO.output(b,True)
            GPIO.output(ve,False)
            sleep(1)
            GPIO.output(b,False)
            sleep(7)
            GPIO.output(ve,True)                   
            msg('1')
        else:
            print("  Mems Normal")
            GPIO.output(b,False)
            GPIO.output(ve,True)      
        #data.write(str.encode('sensornewgsm.php?client=iot2k19096&s1=THEFT_DETECTION&s2=TRIOS&s3=ASHOK_PILLAR&s4=CHENNAI&s5=NA&sms=YES&msg=THEFT#'.format('raja')))
        
        output = analogInput(1) # Reading from CH1
        print("                         Heart Rate")
        print(output)       
        if(output < 420 ):
            if((aa=='1') and (count<20)):
                print(hb)
                count+=1
            else:
                GPIO.output(cpf,True)
                GPIO.output(cpr,True)            
                print("No Heartbeat found ")
            while(i<5):
                 i+=1
                 GPIO.output(cpf,True)
                 GPIO.output(cpr,False)
                 sleep(0.1)
                 GPIO.output(cpf,False)
                 GPIO.output(cpr,True)
                 cc='1'
                 aa='0'
        elif((output>350) and (output<480)):
            output=output/130
            print("                   heart rate                      ")
            aa='1'
            hb=73+output
            print("                     ")
            if ((hb>69) and (hb<85)):
                 print("heart_Rate_Normal")
                 GPIO.output(cpf,True)
                 GPIO.output(cpr,True)
                 GPIO.output(b,False)
                 i=0
        else:
            msg('2')
            print("heart_Rate_ABNormal")
            GPIO.output(b,False)
            i+=1
            GPIO.output(cpf,True)
            GPIO.output(cpr,False)
            sleep(0.5)            
            GPIO.output(cpf,False)
            GPIO.output(cpr,True)
            sleep(0.5)
            GPIO.output(cpf,True)
            GPIO.output(cpr,False)
            sleep(0.5)            
            GPIO.output(cpf,False)
            GPIO.output(cpr,True)
            sleep(0.5)
            GPIO.output(cpf,True)
            GPIO.output(cpr,False)
            sleep(0.5)            
            GPIO.output(cpf,False)
            GPIO.output(cpr,True)
            sleep(0.5)
            GPIO.output(cpf,True)
            GPIO.output(cpr,False)
            sleep(0.5)            
            GPIO.output(cpf,True)
            GPIO.output(cpr,True)
            cc='1'       
        output2 = analogInput(2) # Reading from CH1
        print("RESPIRATORY:",output2)
        print(output2)      
        if (output2>800):
            GPIO.output(ve1,True)            
            #GPIO.output(b,True)
            print("                  RESPIRATORY ABNORMAL")
            msg('3')            
            GPIO.output(b,True)   
        else:
            jj=0
            GPIO.output(b,False)
 #           GPIO.output(ve1,True)    
            print("                   RESPIRATORY NORMAL")
        sleep(1)    
        #value = analogInput(2)
        #volts = (value * 3.3) / 1024
        #temperature = volts / (10.0 / 1000)
        #temperature=temperature+8
        #print (temperature)
        #print ("%4d/1023 => %5.3f V => %4.1f °C" % (value, volts,(temperature))
        #data.write(str.encode('sensornewgsm.php?client=iot2k19096&s1=HEART_RATE:_{}&s2=BP:{}&s3=TEMPERATURE:_{}&s4=NA&s5=NA&sms=NO&msg=ABNORMAL#'.format(str(hb),str(bp),str(temperature))))
#   mode0()  
    if mode==1:
        sleep(1) 
        print("   MODE1 ")
        ir=GPIO.input(p)
        print ("pir state=", ir)
        if ir==0 and bb=='0':          
            print("ir ok")
            bb='1'   
            sleep(1) 
        else:
            print("ir notdet")  
            if(ir==0) and (bb=='1'):
                bb='0'        
        if bb=='1':
            if ir==0 and count<10 :              
                print("irrrrrrrr")
                count=0            
            else:
                count+=1  
                print(count)
                if count >10:
                    GPIO.output(b,True)                    
                    if count >15 and m==0:
                       msg('4')
                        GPIO.output(b,True)
                    #else:
                        #GPIO.output(b,False)
                    elif count >15 and m==1:
                        count=0
                        GPIO.output(b,False)
        print("MODE2")    
            #mode1()      
        print (" sw state=", mode)
