import serial
import time
from packets_and_framing.tiny_phy import tiny_phy
from attiny.furnace_ctrl import pid # can remove this later

class FurCli:

    def __init__(self, device: str):
        ser = serial.Serial(device)
        ser.baudrate = 250_000 // 8
        ser.stopbits = 2
        self.tp = tiny_phy.TinyPhy(ser)


    def get_status(self):
        self.tp.send_packet(b'\x65')
        packet = self.tp.read_packet()[1]

        if len(packet) == 10:
            tc_temp = (packet[0] + (packet[1] << 8)) / 10
            therm_temp = (packet[2] + (packet[3] << 8)) / 10
            sp_temp = (packet[4] + (packet[5] << 8)) / 10
            max_sp = (packet[6] + (packet[7] << 8)) / 10
            ctrl = packet[8] + (packet[9] << 8)

            print(f"Furnace: {tc_temp} C")
            print(f"Cold Junction: {therm_temp} C")
            print(f"Setpoint {sp_temp} C")
            print(f"Duty Cycle {round(ctrl / 0xff * 100, 1)} %")
            print(f"ctrl raw {ctrl}")

        else:
            print(f"oops packet is {len(packet)} bytes long")
            print(packet)

        with open("test_file_2.csv", 'a') as fp:
            fp.write(f"{round(time.time())},{tc_temp},{sp_temp},{ctrl}\n")

        return tc_temp

    def get_setpoint(self):
        self.tp.send_packet(b'\x65')
        packet = self.tp.read_packet()[1]

        if len(packet) == 10:
            tc_temp = (packet[0] + (packet[1] << 8)) / 10
            therm_temp = (packet[2] + (packet[3] << 8)) / 10
            sp_temp = (packet[4] + (packet[5] << 8)) / 10
            max_sp = (packet[6] + (packet[7] << 8)) / 10

        return sp_temp

    def set_point(self, setpoint: int):
        if setpoint > 0xffff:
            print("warning setpoint too high will clip")
            setpoint &= 0xffff

        packet = b'\x5e' + setpoint.to_bytes(2, 'big')
        self.tp.send_packet(packet)


    def monitor_furnace(self):
        while(True):
            self.get_status()
            time.sleep(1)


    def run_profile(self, desired_temp: int, ramp_rate: float, updates_per_min: int = 60):
        setpoint_temp = self.get_setpoint()
        ramp_step = ramp_rate / updates_per_min
        
        while(abs(setpoint_temp - desired_temp) > (ramp_step * 2)):
            if (desired_temp > setpoint_temp):
                setpoint_temp += ramp_step
            else:
                setpoint_temp -= ramp_step
            self.set_point(int(setpoint_temp * 10))
            time.sleep(0.01)
            self.get_status()
            time.sleep(60 / updates_per_min) 
        self.set_point(int(desired_temp * 10))


    def control_loop(self, setpoint):
        integ = 54

        while(True):
            current_temp = self.get_status()
            error = setpoint - current_temp
            [ctrl, integ] = pid.get_next_ctrl_state(integ, error)

            if integ > 255:
                integ = 255
            elif integ < -255:
                integ = -255

            if ctrl > 255:
                ctrl = 255
            elif ctrl < 0:
                ctrl = 0
            time.sleep(0.1)
            self.set_point(ctrl)

            duty_cycle = round(ctrl / 255 * 100, 1)
            print(f"current temp: {current_temp} C")
            print(f"error: {error} C")
            print(f"duty_cycle {duty_cycle}%")

            time.sleep(0.9)


            with open("test_file.csv", 'a') as fp:
                fp.write(f"{round(time.time())},{round(current_temp, 1)},{setpoint},{round(error, 1)},{duty_cycle}\n")

