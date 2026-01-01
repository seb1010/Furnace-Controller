import matplotlib.pyplot as plt
import numpy as np

def run_sim():
    num_pts = 100 
    time = [0.01 * x for x in range(num_pts)]
    plant_output = [0 for x in range(num_pts)]
    ctrl_output = [0 for x in range(num_pts)]

    plant_state = 0
    ctrl_state = 0
    setpoint = 1000
    integral = 0

    for i, _ in enumerate(time):
        error = setpoint - plant_state
        [ctrl, integral] = get_next_ctrl_state(integral, error)
        plant_state = get_next_plant_state(plant_state, ctrl)
        plant_output[i] = plant_state
        ctrl_output[i] = ctrl

    plt.plot(time, plant_output, time, ctrl_output)
    plt.show()

def get_next_ctrl_state(integ: int, error: int):
    integ_gain = 0.1
    pres_gain = 8

    integ += error * integ_gain
    pres = error * pres_gain
    ctrl = int(integ + pres)

    if ctrl >= (1 << 16):
        ctrl = (1 << 16) - 1
    elif ctrl < 0:
        ctrl = 0

    print(f"integral {integ}")
    print(f"pres {pres}")

    return ctrl, integ


def get_next_plant_state(current_state: int, ctrl: int):
    plant_gain = 0.1
    error_term = ctrl - current_state
    return round(current_state + error_term * plant_gain)


def get_thermocouple_temp(tc_counts: int, therm_counts: int):
    voltage_mv = get_thermocouple_voltage(tc_counts)
    cold_junct_temp = get_thermistor_temp(therm_counts)

    cold_voltage_mv = get_cold_voltage(cold_junct_temp)
    av_mv = voltage_mv + cold_voltage_mv

    temp = 0.524 + 23.9 * av_mv + 0.152 * av_mv ** 2 - 0.0121 * av_mv ** 3 + 0.000316 * av_mv ** 4 - 0.0000026 * av_mv ** 5

    return temp

def get_cold_voltage(cj_temp_c: float):
    voltage_mv = 9.52e-5 + 0.0394 * cj_temp_c + 2.64e-5 * cj_temp_c ** 2 - 1.11e-7 * cj_temp_c ** 3
    return voltage_mv

def get_thermocouple_voltage(counts: int, offset: int = 10):
    return (counts - offset) / 4096 * 3000 / 62

def get_thermistor_temp(counts: int, max_counts: int = 1024, beta: int = 3380):
    r = counts / (max_counts - counts)
    one_on_t = np.log(r) / beta + 1 / (273.15 + 25)
    t = 1 / one_on_t - 273.15

    return t

def get_setpoint_cts(temp_c: float, cold_junction_c: float, offset: int = 10):
    v_mv = 0.192 + 0.0393 * temp_c -1.03e-6 * temp_c ** 2 + 1.83e-8 * temp_c ** 3 - 2.44e-11 * temp_c ** 4 + 8.87e-15 * temp_c ** 5

    ad_v_mv = v_mv - get_cold_voltage(cold_junction_c)

    counts = int(ad_v_mv * 4096 / 3000 * 62)
    return counts + offset

