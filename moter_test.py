import serial
import numpy as np



serial_device = serial.Serial("COM5", 115200,timeout=0.5)

class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


    def __init__(self, serial_device,moterid):
        """
        define MotorControl object 定义电机控制对象
        :param serial_device: serial object 串口对象
        """
        self.moterid=moterid
        self.serial_ = serial_device
        if self.serial_.is_open:  # open the serial port
            print("Serial port is open")
            serial_device.close()
        self.serial_.open()

    def _send_data(self, motor_id, data):
        self.send_data_frame[13] = motor_id & 0xff # id low 8 bits
        self.send_data_frame[14] = (motor_id >> 8)& 0xff  #id high 8 bits


        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def read_moter_state(self):
        self.data=np.array([0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self._send_data(self.moterid,self.data)
        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\x9a')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")

    def speed_control(self, speedControl, iqControl):
       
        data = np.zeros(8, np.uint8)
        data[0] = 0xA2  
        data[1] = 0x00  

        data[2:4] = np.array([iqControl & 0xFF, (iqControl >> 8) & 0xFF], np.uint8)

        data[4] = speedControl & 0xFF
        data[5] = (speedControl >> 8) & 0xFF
        data[6] = (speedControl >> 16) & 0xFF
        data[7] = (speedControl >> 24) & 0xFF

        self._send_data(self.moterid, data)
        print(f"Sent speedControl={speedControl}, iqControl={iqControl} to motor 0x{self.moterid:X}")

        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\xa2')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")


    def torque_control(self, iqControl):
        
        data = np.zeros(8, np.uint8)
        data[0] = 0xA1  
        data[1] = 0x00
        data[2] = 0x00
        data[3] = 0x00

        # iqControl 16-bit, little endian
        data[4] = iqControl & 0xFF
        data[5] = (iqControl >> 8) & 0xFF

        data[6] = 0x00
        data[7] = 0x00

        self._send_data(self.moterid, data)
        print(f"Sent torque control iq={iqControl} to motor 0x{self.moterid:X}")    

        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\xa1')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")

    def enable(self):
        data = np.zeros(8, np.uint8)
        data[0] = 0x88 
        data[1:7] = 0x00  
        self._send_data(self.moterid, data)
        print(f" 0x{self.moterid:X} enable")

        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\x88')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")

    def disable(self):
        data = np.zeros(8, np.uint8)
        data[0] = 0x80 
        data[1:7] = 0x00  
        self._send_data(self.moterid, data)
        print(f" 0x{self.moterid:X} disable")

        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\x80')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")
    
    def reset(self):
        data = np.zeros(8, np.uint8)
        data[0] = 0x9B 
        data[1:7] = 0x00  
        self._send_data(self.moterid, data)
        print(f" 0x{self.moterid:X} reset")

        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\x9b')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")
        
    def stop(self):
        data = np.zeros(8, np.uint8)
        data[0] = 0x81 
        data[1:7] = 0x00  
        self._send_data(self.moterid, data)
        print(f" 0x{self.moterid:X} stop")

        received=serial_device.readall()
        print("received:",received)
        idx = received.find(b'\x81')
        if idx != -1 and len(received) >= idx + 8:
                data = received[idx:idx+8]
                command = data[0]
                temperature = int.from_bytes(data[1:2], byteorder='little', signed=True)
                voltage = int.from_bytes(data[2:4], byteorder='little', signed=False) * 0.01
                current = int.from_bytes(data[4:6], byteorder='little', signed=False) * 0.01
                motor_state = data[6]
                error_state = data[7]

                print(f"command: 0x{command:02X}")
                print(f"temperature: {temperature} °C")
                print(f"voltage: {voltage:.2f} V")
                print(f"current: {current:.2f} A")
                print(f"motor_state: 0x{motor_state:02X}")
                print(f"error_state: 0x{error_state:02X}")
        else:
                print("No valid DATA found")




MotorControl1=MotorControl(serial_device,0x141)
                  

if __name__ == '__main__':
    while True:
        print("readstate/enable/disable/reset/stop/speed/torque/exit")
        cmd = input("cmd:")
        if cmd=="readstate":
                MotorControl1.read_moter_state()
        elif cmd=="enable":
                MotorControl1.enable()
        elif cmd=="disable":
                MotorControl1.disable()
        elif cmd=="reset":
                MotorControl1.reset()
        elif cmd=="stop":
                MotorControl1.stop()
        elif cmd=="speed":
                speed=int(input("speed:"))
                MotorControl1.speed_control(speed*100, iqControl=500) #0.01dps/LSB -> 1dps/LSB
        elif cmd=="torque":
                torque=int(input("torque:"))
                MotorControl1.torque_control(iqControl=torque)
        elif cmd=="exit":
            break

serial_device.close()







# MotorControl1.read_moter_state()
# MotorControl1.enable()
# MotorControl1.disable()
# MotorControl1.reset()
# MotorControl1.stop()
# MotorControl1.speed_control(speedControl=0, iqControl=500)
# MotorControl1.torque_control(15)

# 主机发送该命令以控制电机的速度， 同时带有力矩限制。控制值 speedControl 为 int32_t 类型，对应
# 实际转速为 0.01dps/LSB；控制值 iqControl 为 int16_t 类型，数值范围-2048~ 2048，对应 MF 电机实际转矩
# 电流范围-16.5A~16.5A，对应 MG 电机实际转矩电流范围-33A~33A，母线电流和电机的实际扭矩因不同电
# 机而异。
