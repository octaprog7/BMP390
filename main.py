from micropython import const
from machine import I2C, Pin
import bmp390mod
import time
from sensor_pack_2.bus_service import I2cAdapter


def pa_mmhg(value: float) -> float:
    """Перевод атмосферного давления из Па в мм рт.ст.
    Convert air pressure from Pa to mm Hg."""
    return 7.50062E-3 * value

I2C_ID: int = const(1)
SCL_PIN: int = const(7)
SDA_PIN: int = const(6)
I2C_FREQ: int = const(400_000)
SENSOR_ADDR: int = const(0x77)
ITERATIONS: int = const(33)


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе I2C, для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # i2c = I2C(id=0, scl=Pin(13), sda=Pin(12), freq=400_000)  # on Arduino Nano RP2040 Connect
    i2c = I2C(id=I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)   # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)
    # ps - pressure sensor
    ps = bmp390mod.Bmp390(adapter=adapter, address=SENSOR_ADDR)
    res = ps.get_id()
    print(f"chip_id: {res}")
    # если я не вызывал этот метод, то датчик не входил в режим
    # однократных измерений и был все время в режиме периодических измерений!!!
    # я не знаю, что это за глюк! Поэтому перед вызовом start_measurement(..) вызывайте soft_reset()!!
    ps.soft_reset()
    print(f"pwr mode: {ps.get_power_mode()}")

    calibration_data = [ps.get_calibration_coefficient(index) for index in range(14)]
    print(f"Calibration data: {calibration_data}")

    print(f"Event: {ps.get_event()}; Int status: {ps.get_int_status()}; FIFO length: {ps.get_fifo_length()}")
    #
    delay_func = time.sleep_ms
    #
    ps.set_oversampling(pressure_oversampling=2, temperature_oversampling=3)
    ps.set_sampling_period(5)
    ps.set_iir_filter(2)

    print("Режим однократных измерений по запросу!")
    print(f"pwr mode: {ps.get_power_mode()}")
    print(f"время преобразования в [мкс]: {ps.get_conversion_cycle_time()}")
    for _ in range(ITERATIONS):
        ps.start_measurement(enable_press=True, enable_temp=True, mode=1)
        delay_func(300)
        temperature_ready, pressure_ready, cmd_ready = ps.get_data_status()
        if cmd_ready and pressure_ready:
            t, p = ps.get_temperature(), ps.get_pressure()
            pm = ps.get_power_mode()
            print(f"Temperature: {t} \xB0C; pressure: {p} Pa ({pa_mmhg(p)} mm Hg); pwr_mode: {pm} ")
        else:
            print(f"Data ready: temp {temperature_ready}, press {pressure_ready}")
    #
    _min_p, _max_p = 1E6, 0
    print("Режим непрерывных периодических измерений!")
    ps.start_measurement(enable_press=True, enable_temp=True, mode=2)
    print(f"pwr mode: {ps.get_power_mode()}")
    for values in ps:
        delay_func(300)
        t, p = values.T, values.P
        tme = ps.get_sensor_time()
        _min_p = min(_min_p, p)
        _max_p = max(_max_p, p)
        if t is not None and p is not None:  # достаточно проверки на None
            print(f"T={t:.2f}°C, P={p:.2f} Pa, time={tme}, min_P={_min_p}, max_P={_max_p}")
