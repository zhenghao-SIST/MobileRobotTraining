#!/usr/bin/env python
# Author: Zhenghao Li
# Email: lizhenghao@shanghaitech.edu.cn
# Institute: SIST
# Created: 2026-01-14
# Last Modified: 2026-01-14
# Description: AQMD6008NS-TBE Driver

from pymodbus.client import ModbusSerialClient as ModbusClient
import time
import threading
import struct
import sys

class MotorState():
    def __init__(self, id):
        self.id = id
        self.speed = 0.0
        self.current = 0.0

    def set_speed(self, speed):
        self.speed = speed

    def __repr__(self):
        return f"MotorState({hex(self.id)} speed={self.speed}RPM, current={self.current})"

    def __str__(self):
        return self.__repr__()

class Driver():
    def __init__(self, serialPort):
        self.thread_rd_run = 0
        self.client = ModbusClient(serialPort, parity='E', baudrate=115200, timeout=1)
        self.motors = (MotorState(0x01), MotorState(0x02))
        
        #连接到 Modbus 从站
        if not self.client.connect():
            print(f"Can't open serial port！")
            sys.exit(-1)

        time.sleep(0.5)
        self.init_device(0x01)
        self.init_device(0x02)
        time.sleep(0.5)
        self.thread_rd_run = 1
        thread_rd = threading.Thread(target=self.read_from_modbus, daemon=False)
        thread_rd.start()


    def get_speed(self):
        return self.motors[0].speed, self.motors[1].speed

    def init_device(self, ID):
        self.client.write_register(0x0080, 0x03, device_id = ID) #Set Speed Mode
        self.client.write_register(0x0069, 500, device_id = ID)  #Set Encoder

    def set_speed(self, speed, id):
        speed %= 65535
        self.client.write_register(address=0x40, value=speed, device_id=id)

    def read_from_modbus(self):
        while self.thread_rd_run:
            ### Speed
            try:
                spdRes0 = self.client.read_holding_registers(0x001e, count=2, device_id = self.motors[0].id)
                data = struct.pack('>HH', spdRes0.registers[0], spdRes0.registers[1])
                self.motors[0].set_speed(struct.unpack('>i', data)[0])

                spdRes1 = self.client.read_holding_registers(0x001e, count=2, device_id = self.motors[1].id)
                data = struct.pack('>HH', spdRes1.registers[0], spdRes1.registers[1])
                self.motors[1].set_speed(struct.unpack('>i', data)[0])

                #Current 
                #curRe = self.client.read_holding_registers(0x0011, count=1, device_id=self.motors[0].id)

                time.sleep(0.08)
            except Exception as e:
                print("Read Error Occur")
                print(e)
                sys.exit(-1)

if __name__ == '__main__':
    dr = Driver('COM4')
    thread_rd = threading.Thread(target=dr.read_from_modbus, daemon=False)
    thread_rd.start()
    dr.set_speed(-1200, 0x01)
    time.sleep(2)
    dr.set_speed(0, 0x01)
    dr.thread_rd_run = False
