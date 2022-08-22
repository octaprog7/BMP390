# This is a sample Python script.
from machine import I2C, Pin
import bmp390
import time
from sensor_pack.bus_service import I2cAdapter
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def pa_mmhg(value: float) -> float:
    """Convert air pressure from Pa to mm Hg"""
    return value*7.50062E-3


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) № для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    i2c = I2C(id=0, scl=Pin(13), sda=Pin(12), freq=400000)  # on Arduino Nano RP2040 Connect tested
    adaptor = I2cAdapter(i2c)
    # ps - pressure sensor
    ps = bmp390.Bmp390(adaptor)
    # если у вас посыпались исключения, чего у меня на макетной плате с али и проводами МГТВ не наблюдается,
    # то проверьте все соединения.
    # Радиотехника - наука о контактах! РТФ-Чемпион!
    res = ps.get_id()
    print(f"chip_id: {res}")
    
    calibration_data = [ps.get_calibration_data(index) for index in range(0, 14)]
    print(f"Calibration data: {calibration_data}")

    print(f"Event: {ps.get_event()}; Int status: {ps.get_int_status()}; FIFO length: {ps.get_fifo_length()}")
    #
    ps.set_oversampling(2, 3)
    ps.set_sampling_period(5)
    ps.set_iir_filter(2)
    #
    ps.start_measurement(True, True, 2)
    for i in ps:
        time.sleep_ms(300)
        s = ps.get_status()
        if s[2] and s[1]:
            t, p, tme = ps.get_temperature(), ps.get_pressure(), ps.get_sensor_time()
        else:
            print(f"Data ready: temp {s[2]}, press {s[1]}")
            continue
        #
        print(f"Temperature: {t} \xB0C; pressure: {p} hPa ({pa_mmhg(p)} mm Hg); time: {hex(tme)}; ")
