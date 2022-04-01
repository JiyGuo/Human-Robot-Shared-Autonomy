#!/usr/bin/python
# -*- coding:utf-8 -*-
from __future__ import print_function
import time
from io_interface import *

if __name__ == "__main__":
    print("testing io-interface")
    get_states()
    print("listener has been activated")
    set_states()
    print("service-server has been started")
    digit_pin = 0
    while(True):
        if digit_pin > 7:
            digit_pin = 0
        # 设置tool电压 0-24v
        set_tool_voltage(0)
        # 设置digital_out (端口号pin, 所设置的值value(True or False))
        set_digital_out(digit_pin, True)
        # 设置analog_out (端口号pin, 所设置的值value(0-1的模拟量))
        set_analog_out(0, 0.75)

        #　读取当前相关端口的值
        print(Analog_Out_States[0])
        print(Digital_Out_States[digit_pin])
        time.sleep(1)

        set_tool_voltage(24)
        set_digital_out(digit_pin, False)
        set_analog_out(0, 0.25)

        print(Analog_Out_States[0])
        print(Digital_Out_States[digit_pin])
        time.sleep(1)

        digit_pin +=1

