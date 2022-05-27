#!/usr/bin/env python

#jetson

import rospy
from pymodbus.client.sync import ModbusSerialClient
import numpy as np


client = ModbusSerialClient(method='rtu', port = '/dev/ttyUSB0', stopbits = 1, parity = 'N', baudrate = 9600)

# def communicate():

# right_modBus_mode             = '01063A980002853C' #Fn_000 = 2
# right_speedRun_mode           = '01063A9B0002753C' #Fn_003 = 2
# right_enable_hex              = '01063AA80001C532' #Fn_010 = 1
# right_workingAllowd_enable    = '010627200001B374' #Fn_013 = 1

# right_run_hex                 = '01064E2003E89F96' #01 06 4E 20
# right_run_hex                 = '01064E200BB8986A' # 3000
# right_run_hex                 = '01064E2000015EE8' # 1
# right_run_hex                 = '01064E20FC18DE22' # -1000
# right_run_hex                 = '01064E20FFFF9E98' # -1
# right_run_hex                 = '01064E20000A1F2F' # 10

# right_stop_hex                = '01062720000082B4' #01 06 4E 20
# right_stop_hex                = '01064E2000009F28' #01 06 4E 20

# left_modBus_mode             = '02063A980002850F' #Fn_000 = 2
# left_speedRun_mode           = '02063A9B0002750F' #Fn_003 = 2
# left_enable_hex              = '02063AA80001C501' #Fn_010 = 1
# left_workingAllowd_enable    = '020627230001B347' #Fn_013 = 1
    

if __name__ == '__main__': 
    try:
        

        while not rospy.is_shutdown():
            
            # print("aaaaaaaaaaaaaaa")
            connection = client.connect()
            # print(connection)
            hlist = []
            data = client.read_input_registers(address=29, count=2, unit = 1)
            # print(data.registers)

            for i in data.registers:
                # print("i:", i)
                hlist.append(i)
                # print(hlist)
                client.close()
            
            print(hlist)
            
        
    except rospy.ROSInterruptException:
        pass