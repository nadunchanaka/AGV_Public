#!/usr/bin/env python

#Jetson


from glob import glob
from turtle import st
import rospy
import serial
import time
from geometry_msgs.msg import *
from std_msgs.msg import *
from pymodbus.client.sync import ModbusSerialClient
import crc16

client = ModbusSerialClient(method='rtu', port = '/dev/ttyUSB0', stopbits = 1, parity = 'N', baudrate = 9600, strict=False)
ser = serial.Serial("/dev/ttyUSB0", 9600)

right_modBus_mode             = '01063A980002853C' #Fn_000 = 2
right_speedRun_mode           = '01063A9B0002753C' #Fn_003 = 2
right_enable_hex              = '01063AA80001C532' #Fn_010 = 1
right_workingAllowd_enable    = '010627200001B374' #Fn_013 = 1

left_modBus_mode             = '02063A980002850F' #Fn_000 = 2
left_speedRun_mode           = '02063A9B0002750F' #Fn_003 = 2
left_enable_hex              = '02063AA80001C501' #Fn_010 = 1
left_workingAllowd_enable    = '020627230001B347' #Fn_013 = 1


def initialize():
    if ser.is_open:
        # print("port open success")

        byte_array1 = bytearray.fromhex(right_modBus_mode)
        ser.write(byte_array1)
        time.sleep(0.1)

        byte_array2 = bytearray.fromhex(right_speedRun_mode)
        ser.write(byte_array2)
        time.sleep(0.1)

        byte_array3 = bytearray.fromhex(right_enable_hex)
        ser.write(byte_array3)
        time.sleep(0.1)

        byte_array4 = bytearray.fromhex(right_workingAllowd_enable)
        ser.write(byte_array4)
        time.sleep(0.1)
        #-------------------------------------------------------------
        byte_array1 = bytearray.fromhex(left_modBus_mode)
        ser.write(byte_array1)
        time.sleep(0.1)

        byte_array2 = bytearray.fromhex(left_speedRun_mode)
        ser.write(byte_array2)
        time.sleep(0.1)

        byte_array3 = bytearray.fromhex(left_enable_hex)
        ser.write(byte_array3)
        time.sleep(0.1)

        byte_array4 = bytearray.fromhex(left_workingAllowd_enable)
        ser.write(byte_array4)
        time.sleep(0.1)  

    else:
        print("port open failed")

def hex2complement(number):

    global firstHex, secondHex
    numberOut = []

    bits = 16
    if number < 0:
        hexNum = hex((1 << bits) + number)[2:]
    else:
        hexNum = hex(number)[2:]

    # print(hexNum)
    # hexadecimal_result = format(number, "03X") #convert int to Hex

    for i in hexNum.zfill(4): #devide 2 hex value
        numberOut.append(i)

    firstHexStr = str(numberOut[0]) + str(numberOut[1]) # first hex value
    secondHexStr = str(numberOut[2]) + str(numberOut[3]) #second hex value

    an_integer1 = int(firstHexStr, 16) #conver to int
    an_integer2 = int(secondHexStr, 16) #conver to int

    firstHex = an_integer1 # for CRC calculator
    secondHex = an_integer2 # for CRC calculator

    # print("RPM Hex = " + firstHexStr, secondHexStr)
    
    return firstHexStr, secondHexStr, firstHex, secondHex

def crcCalculation(address, function_code, start_at_reg, num_of_reg):
    global crc_High, crc_Low
    hi_Append = []
    lo_Append = []

    loNum = []
    hiNum =[]

    read_device = address + function_code + start_at_reg + num_of_reg
    crc = crc16.calc(read_device)
    crc_hi = crc/256
    crc_lo = crc & 0xFF

    crc_Low = str(hex(crc_lo)[2:])
    crc_High = str(hex(crc_hi)[2:])

    for i in crc_Low.zfill(2): #devide 2 hex value
        loNum.append(i)

    for i in crc_High.zfill(2): #devide 2 hex value
        hiNum.append(i)

    crc_Low = str(loNum[0]) + str(loNum[1])
    crc_High = str(hiNum[0]) + str(hiNum[1])

    # print ("CRC = " + crc_Low, crc_High)
    
    return crc_Low, crc_High

def wheel_Drive(leftSpeed, rightSpeed):
    global previousLeftData
    global previousRightData
    try:
        leftSpeed = int(leftSpeed)
        rightSpeed = int(rightSpeed)
        

        leftFirstHexStr, leftSecondHexStr, leftFirstHex, leftSecondHex = hex2complement(leftSpeed)
        leftAddress = chr(0x02)
        leftFunction_code = chr(0x06)
        leftStart_at_reg = chr(0x4E) + chr(0x20)
        leftNum_of_reg = chr(leftFirstHex) + chr(leftSecondHex)
        crc_Low, crc_High = crcCalculation(leftAddress, leftFunction_code, leftStart_at_reg, leftNum_of_reg)
        leftMotor = '02064E20' + str(leftFirstHexStr) + str(leftSecondHexStr) + str(crc_Low) + str(crc_High) #Add all hex code together Left Side

        rightFirstHexStr, rightSecondHexStr, rightFirstHex, rightSecondHex = hex2complement(rightSpeed)
        rightAddress = chr(0x01)
        rightFunction_code = chr(0x06)
        rightStart_at_reg = chr(0x4E) + chr(0x20)
        rightNum_of_reg = chr(rightFirstHex) + chr(rightSecondHex)
        rightCrc_Low, rightCrc_High = crcCalculation(rightAddress, rightFunction_code, rightStart_at_reg, rightNum_of_reg)
        rightMotor = '01064E20' + str(rightFirstHexStr) + str(rightSecondHexStr) + str(rightCrc_Low) + str(rightCrc_High) #Add all hex code together Right side
        
        # print(leftMotor.upper(), rightMotor.upper())
        # print("")

        if len(rightMotor) & len(leftMotor) == 16:
            if ser.is_open:
                
                client.close()
                time.sleep(0.1)
                byte_array1 = bytearray.fromhex(leftMotor)
                ser.write(byte_array1)
                time.sleep(0.1)

                byte_array = bytearray.fromhex(rightMotor)
                ser.write(byte_array)
                time.sleep(0.1)
                ser.close
            else:
                print("port open failed")
        else:
            print("Register Hex Code Not Complete <Right Side>")

        if len(rightMotor) & len(leftMotor) == 16:
            if ser.is_open:
                
                client.close()
                time.sleep(0.1)
                byte_array1 = bytearray.fromhex(leftMotor)
                ser.write(byte_array1)
                time.sleep(0.1)

                byte_array = bytearray.fromhex(rightMotor)
                ser.write(byte_array)
                time.sleep(0.1)
                ser.close
            else:
                print("port open failed")
        else:
            print("Register Hex Code Not Complete <Right Side>")
        
    except:
        print("Wheel Drive Error")
        mainFunction()


RightDirection = ""
LeftDirection = ""

def kinematic(data):
    global RightWheelKinInt
    global LeftWheelKinInt
    global RightDirection
    global LeftDirection

    WheelBase = 0.31 #in meter
    WheelRadius = 0.15 
    
    vel_x = data.linear.x 
    omega = data.angular.z

    RightWheelKin = ((2 * vel_x) + (omega * WheelBase)) / (2 * WheelRadius) #kinematic equation
    LeftWheelKin = ((2 * vel_x) - (omega * WheelBase)) / (2 * WheelRadius)

    RightWheelKinInt = int(RightWheelKin) 
    LeftWheelKinInt = int(LeftWheelKin)

    RightWheelKinInt = RightWheelKinInt * 100
    LeftWheelKinInt = LeftWheelKinInt * 100
#-------------------------------------------------
    if RightWheelKinInt == 0:
        RightDirection = "Stop"
    elif RightWheelKinInt > 0:
        RightDirection = "Forwerd"
    else:
        RightDirection = "Reverce"
#-------------------------------------------------
    if LeftWheelKinInt == 0:
        LeftDirection = "Stop"
    if LeftWheelKinInt > 0:
        LeftDirection = "Forwerd"
    else:
        LeftDirection = "Reverce"
#-------------------------------------------------

    # print(LeftDirection, RightDirection)

    # print("Kinematic RPM = " + str(LeftWheelKinInt), str(RightWheelKinInt))
    wheel_Drive(LeftWheelKinInt, RightWheelKinInt)
    # readData(1)

Right_PreviosVal = 0
Right_CurrentVal = 0
Right_TotalStichCount = 0
Right_StichCountDeference = 0

Left_PreviosVal = 0
Left_CurrentVal = 0
Left_TotalStichCount = 0
Left_StichCountDeference = 0

def readData(event):
    try:
        if client.connect():

            global Right_PreviosVal
            global Right_CurrentVal
            global Right_TotalStichCount
            global Right_StichCountDeference

            RightHlist = []
            RightData = client.read_input_registers(address=29, count=2, unit = 1)

            for i in RightData.registers:
                RightHlist.append(i)
            client.close()
            
            RightCountPer_qur = RightHlist[0]
            RightQuarter_counter = RightHlist[1]
            RightTotalQur_counter = 65536

            RightTotalCountAll = (RightCountPer_qur + (RightTotalQur_counter * RightQuarter_counter))
            RightTotalCountInt = RightTotalCountAll / 1000
            RightTotalCount = int(RightTotalCountInt)

            Right_CurrentVal = RightTotalCount
            
            Right_StichCountDeference = Right_CurrentVal - Right_PreviosVal
            
            if RightDirection == "Reverce":
                if Right_StichCountDeference > 1000:
                    Right_TotalStichCount = (RightTotalCount - 4294967)
                else:
                    Right_TotalStichCount += Right_StichCountDeference

            elif RightDirection == "Forwerd":
                if Right_StichCountDeference > 1000:
                    Right_TotalStichCount = (RightTotalCount + 4294967)
                else:
                    Right_TotalStichCount += Right_StichCountDeference
            
            Right_PreviosVal = Right_CurrentVal
                
            # print(Right_TotalStichCount, Right_StichCountDeference)

#---------------------------------------------------------------------------
            global Left_PreviosVal
            global Left_CurrentVal
            global Left_TotalStichCount
            global Left_StichCountDeference

            LeftHlist = []
            LeftData = client.read_input_registers(address=29, count=2, unit = 2)

            for i in LeftData.registers:
                LeftHlist.append(i)
            client.close()
            
            LeftCountPer_qur = LeftHlist[0]
            LeftQuarter_counter = LeftHlist[1]
            LeftTotalQur_counter = 65536

            LeftTotalCountAll = (LeftCountPer_qur + (LeftTotalQur_counter * LeftQuarter_counter))
            LeftTotalCountInt = LeftTotalCountAll / 1000
            LeftTotalCount = int(LeftTotalCountInt)

            Left_CurrentVal = LeftTotalCount
            
            Left_StichCountDeference = Left_CurrentVal - Left_PreviosVal

            if LeftDirection == "Reverce":
                if Left_StichCountDeference > 1000:
                    Left_TotalStichCount = (LeftTotalCount - 4294967)
                else:
                    Left_TotalStichCount += Left_StichCountDeference

            elif LeftDirection == "Forwerd":
                if Left_StichCountDeference > 1000:
                    Left_TotalStichCount = (LeftTotalCount + 4294967)
                else:
                    Left_TotalStichCount += Left_StichCountDeference
            
            Left_PreviosVal = Left_CurrentVal
                
            print(Left_TotalStichCount, Right_TotalStichCount)

            # print(LeftTotalCount, LeftDirection)
            # print(LeftDirection)

        else:
            print("No serial connection")
    except:
        print("Errooooooooo")
        readData(0)

def mainFunction():
    rospy.init_node('drive_wheel', anonymous=True)

    while not rospy.is_shutdown():
        rospy.Subscriber('/cmd_vel', Twist, kinematic) #Subscribe /cmd_val topic
        timer = rospy.Timer(rospy.Duration(0.1), readData)
        rospy.spin()
        timer.shutdown()

if __name__ == '__main__': 
    try:
        initialize()
        mainFunction()

        
    except rospy.ROSInterruptException:
        pass


