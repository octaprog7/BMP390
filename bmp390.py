# micropython
# mail: goctaprog@gmail.com
# MIT license
import micropython
import array

from sensor_pack import bus_service
from sensor_pack.base_sensor import BaseSensor, Iterator


# ВНИМАНИЕ: не подключайте питание датчика к 5В, иначе датчик выйдет из строя! Только 3.3В!!!
# WARNING: do not connect "+" to 5V or the sensor will be damaged!


@micropython.native
def _check_value(value: int, valid_range, error_msg: str) -> int:
    if value not in valid_range:
        raise ValueError(error_msg)
    return value


@micropython.native
def _calibration_regs_addr() -> iter:
    """возвращает кортеж из адреса регистра, размера значения в байтах, типа значения (u-unsigned, s-signed)"""
    start_addr = 0x31
    tpl = ('1b', '2h', '2H')
    """возвращает итератор с адресами внутренних регистров датчика, хранящих калибровочные коэффициенты """
    val_type = "22011002200100"
    for item in val_type:
        v_size, v_type = tpl[int(item)]
        yield int(start_addr), int(v_size), v_type
        start_addr += int(v_size)


class Bmp390(BaseSensor, Iterator):
    """Class for work with Bosh BMP180 pressure sensor"""

    def __init__(self, adapter: bus_service.BusAdapter, address=0xEE >> 1,
                 oversample_temp=0b11, oversample_press=0b11, iir_filter=0):
        """i2c - объект класса I2C; baseline_pressure - давление на уровне моря в Pa в твоей(!) местности;;
        oversample_settings (0..5) - точность измерения 0-грубо но быстро, 5-медленно, но точно;
        address - адрес датчика (0xEF (read) and 0xEE (write) from datasheet)
        iir_filter=0..7; 0 - off, 7 - max value

        i2c is an object of the I2C class; baseline_pressure - sea level pressure in Pa in your(!) area;
        oversample_settings (0..5) - measurement reliability 0-coarse but fast, 5-slow but accurate;"""
        super().__init__(adapter, address, False)
        self._buf_2 = bytearray((0 for _ in range(2)))  # для _read_buf_from_mem
        self._buf_3 = bytearray((0 for _ in range(3)))  # для _read_buf_from_mem
        self._t_lin = None  # for pressure calculation
        # for temperature only!
        self._oss_t = _check_value(oversample_temp, range(6),
                                   f"Invalid temperature oversample value: {oversample_temp}")
        self._oss_p = _check_value(oversample_press, range(6),
                                   f"Invalid pressure oversample value: {oversample_press}")
        self._adapter = adapter
        self._IIR = _check_value(iir_filter, range(8),
                                 f"Invalid iir_filter value: {iir_filter}")
        self._mode = 0  # sleep mode
        self._enable_pressure = False
        self._enable_temperature = False
        self._sampling_period = 0x02  # 1.28 sec
        # массив, хранящий калибровочные коэффициенты (14 штук)
        self._cfa = array.array("l", [0 for _ in range(14)])  # signed long elements
        # считываю калибровочные коэффициенты
        self._read_calibration_data()
        # предварительный расчет
        self._precalculate()

    def __del__(self):
        del self._cfa
        del self._buf_3
        del self._buf_2

    def _read_buf_from_mem(self, address: int, buf):
        """Читает из устройства, начиная с адреса address в буфер.
        Кол-во читаемых байт равно "длине" буфера в байтах!"""
        self._adapter.read_buf_from_mem(self.address, address, buf)
        return buf

    def get_calibration_data(self, index: int) -> int:
        """возвращает калибровочный коэффициент по его индексу (0..13).
        returns the calibration coefficient by its index (0..13)"""
        _check_value(index, range(14), f"Invalid index value: {index}")
        return self._cfa[index]

    @micropython.native
    def _precalculate(self):
        """предварительно вычисленные значения"""
        # для расчета температуры
        self.par_t1 = self.get_calibration_data(0) * 2 ** 8  #
        self.par_t2 = self.get_calibration_data(1) / 2 ** 30  #
        self.par_t3 = self.get_calibration_data(2) / 2 ** 48  #
        # для расчета давления
        self.par_p1 = (self.get_calibration_data(3) - 2 ** 14) / 2 ** 20
        self.par_p2 = (self.get_calibration_data(4) - 2 ** 14) / 2 ** 29
        self.par_p3 = self.get_calibration_data(5) / 2 ** 32
        self.par_p4 = self.get_calibration_data(6) / 2 ** 37
        self.par_p5 = 8 * self.get_calibration_data(7)
        self.par_p6 = self.get_calibration_data(8) / 2 ** 6
        self.par_p7 = self.get_calibration_data(9) / 2 ** 8
        self.par_p8 = self.get_calibration_data(10) / 2 ** 15
        self.par_p9 = self.get_calibration_data(11) / 2 ** 48
        self.par_p10 = self.get_calibration_data(12) / 2 ** 48
        self.par_p11 = self.get_calibration_data(13) / 2 ** 65

    # BaseSensor
    def _read_register(self, reg_addr, bytes_count=2) -> bytes:
        """считывает из регистра датчика значение.
        bytes_count - размер значения в байтах"""
        # print(f"DBG. _read_register. bytes_count: {bytes_count}")
        return self._adapter.read_register(self.address, reg_addr, bytes_count)

    # BaseSensor
    def _write_register(self, reg_addr, value: int, bytes_count=2) -> int:
        """записывает данные value в датчик, по адресу reg_addr.
        bytes_count - кол-во записываемых данных"""
        byte_order = self._get_byteorder_as_str()[0]
        return self._adapter.write_register(self.address, reg_addr, value, bytes_count, byte_order)

    def _read_calibration_data(self) -> int:
        """Читает калибровочные значение из датчика.
        read calibration values from sensor.
        return count read values"""
        if any(self._cfa):
            raise ValueError(f"calibration data array already filled!")
        index = 0
        for v_addr, v_size, v_type in _calibration_regs_addr():
            # print(v_addr, v_size, v_type)
            reg_val = self._read_register(v_addr, v_size)
            rv = self.unpack(f"{v_type}", reg_val)[0]
            # check
            if rv == 0x00 or rv == 0xFFFF:
                raise ValueError(f"Invalid register addr: {v_addr} value: {hex(rv)}")
            self._cfa[index] = rv
            index += 1
        return len(self._cfa)

    def get_id(self) -> tuple:
        """Возвращает идентификатор датчика и его revision ID.
        Returns the ID and revision ID of the sensor."""
        buf = self._buf_2
        self._read_buf_from_mem(0x00, buf)
        # chip id, rev_id
        return buf[0], buf[1]

    def get_error(self) -> int:
        """Возвращает три бита состояния ошибок.
        Bit 0 - fatal_err Fatal error
        Bit 1 - Command execution failed. Cleared on read.
        Bit 2 conf_err sensor configuration error detected (only working in normal mode). Cleared on read.
        """
        err = self._read_register(0x02, 1)[0]
        return err & 0x07

    def get_status(self) -> tuple:
        """Возвращает три бита состояния датчика как кортеж
        Data ready for temperature, Data ready for pressure, CMD decoder status
        бит 0 - CMD decoder status (0: Command in progress; 1: Command decoder is ready to accept a new command)
        бит 1 - Data ready for pressure. (It gets reset, when one pressure DATA register is read out)
        бит 2 - Data ready for temperature sensor. (It gets reset, when one temperature DATA register is read out)
        """
        val = self._read_register(0x03, 1)[0]
        i = 0x07 & (val >> 4)
        drdy_temp, drdy_press, cmd_rdy = 0x04 & i, 0x02 & i, 0x01 & i
        return drdy_temp, drdy_press, cmd_rdy

    @micropython.native
    def get_pressure_raw(self) -> int:
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._read_buf_from_mem(0x04, buf)
        return (h << 16) | (m << 8) | l

    def get_pressure(self) -> float:
        """Return pressure in Pascal [Pa].
        Call get_temperature() before call get_pressure() !!!"""
        uncompensated = self.get_pressure_raw()
        #
        t_lin = self._t_lin
        t_lin2 = t_lin * t_lin
        t_lin3 = t_lin * t_lin * t_lin
        #
        partial_data1 = self.par_p6 * t_lin
        partial_data2 = self.par_p7 * t_lin2
        partial_data3 = self.par_p8 * t_lin3
        partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3
        #
        partial_data1 = self.par_p2 * t_lin
        partial_data2 = self.par_p3 * t_lin2
        partial_data3 = self.par_p4 * t_lin3
        partial_out2 = uncompensated * (self.par_p1 + partial_data1 + partial_data2 + partial_data3)
        #
        partial_data1 = uncompensated * uncompensated
        partial_data2 = self.par_p9 + self.par_p10 * t_lin
        partial_data3 = partial_data1 * partial_data2
        partial_data4 = partial_data3 + (uncompensated * uncompensated * uncompensated) * self.par_p11
        #
        return partial_out1 + partial_out2 + partial_data4

    @micropython.native
    def get_temperature_raw(self) -> int:
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._read_buf_from_mem(0x07, buf)
        return (h << 16) | (m << 8) | l

    def get_temperature(self) -> float:
        """Return temperature in Celsius"""
        uncompensated = self.get_temperature_raw()
        partial_data1 = uncompensated - self.par_t1
        partial_data2 = partial_data1 * self.par_t2
        # Update the compensated temperature since this is needed for pressure calculation !!!
        self._t_lin = partial_data2 + (partial_data1 * partial_data1) * self.par_t3
        return self._t_lin

    @micropython.native
    def get_sensor_time(self):
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._read_buf_from_mem(0x0C, buf)
        return (h << 16) | (m << 8) | l

    def get_event(self) -> int:
        """Bit 0 por_detected ‘1’ after device power up or softreset. Clear-on-read
        Bit 1 itf_act_pt ‘1’ when a serial interface transaction occurs during a
        pressure or temperature conversion. Clear-on-read"""
        evt = self._read_register(0x10, 1)
        return int(evt[0]) & 0b11

    def get_int_status(self) -> int:
        """Bit 0 fwm_int FIFO Watermark Interrupt
        Bit 1 full_int FIFO Full Interrupt
        Bit 3 drdy data ready interrupt"""
        int_stat = self._read_register(0x11, 1)
        return int(int_stat[0]) & 0b111

    def get_fifo_length(self) -> int:
        """The FIFO byte counter indicates the current fill level of the FIFO buffer."""
        buf = self._buf_2
        self._read_buf_from_mem(0x12, buf)
        return self.unpack("H", buf)[0]

    def soft_reset(self, reset_or_flush: bool = True):
        """программный сброс датчика.
        software reset of the sensor"""
        value = 0xB6 if reset_or_flush else 0xB0
        self._write_register(0x7E, value, 1)

    def start_measurement(self, enable_press, enable_temp, mode: int = 2):
        """ # mode: 0 - sleep, 1-forced, 2-normal (continuously)"""
        if mode not in range(3):
            raise ValueError(f"Invalid mode value: {mode}")
        tmp = 0     # self._read_register(0x1B, 1)[0]
        if enable_press:
            tmp |= 0b01
        else:
            tmp &= ~0b01

        if enable_temp:
            tmp |= 0b10
        else:
            tmp &= ~0b10

        # обнуляю биты 4 и 5
        tmp &= ~0b0011_0000
        if 0 == mode:
            pass  # sleep mode
        if 1 == mode:
            tmp |= 0b0001_0000  # forced mode (режим однократных измерений)
        if 2 == mode:
            tmp |= 0b0011_0000  # continuous mode (режим непрерывных периодических измерений)
        # save
        self._write_register(0x1B, tmp, 1)
        self._mode = mode
        self._enable_pressure = enable_press
        self._enable_temperature = enable_temp

    def get_power_mode(self) -> int:
        """Возвращает текущий режим работы датчика:
        0 - сон
        1 или 2 - однократное измерение
        3 - периодические измерения"""
        tmp = self._read_register(0x1B, 1)[0]
        return (0b11_0000 & tmp) >> 4


    def set_oversampling(self, pressure_oversampling: int, temperature_oversampling: int):
        tmp = 0
        po = _check_value(pressure_oversampling, range(6),
                          f"Invalid value pressure_oversampling: {pressure_oversampling}")
        to = _check_value(temperature_oversampling, range(6),
                          f"Invalid value temperature_oversampling: {temperature_oversampling}")
        tmp |= po
        tmp |= to << 3
        self._write_register(0x1C, tmp, 1)
        self._oss_t = temperature_oversampling
        self._oss_p = pressure_oversampling

    def set_sampling_period(self, period: int):
        p = _check_value(period, range(18),
                         f"Invalid value output data rates: {period}")
        self._write_register(0x1D, p, 1)
        self._sampling_period = period

    def set_iir_filter(self, value):
        """Коэффициент IIR-фильтра"""
        p = _check_value(value, range(8),
                         f"Invalid value iir_filter: {value}")
        self._write_register(0x1F, p, 1)
        self._IIR = value

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """возвращает время преобразования в [мкс] датчиком температуры или давления в зависимости от его настроек"""
        k = 2020
        temp_us = 163 + k * 2 ** self._oss_t
        total = 234 + temp_us
        if self._enable_pressure:
            press_us = 392 + k * 2 ** self._oss_p
            total += press_us
        return total

    # Iterator
    def __next__(self) -> [tuple, float, None]:
        if not self._enable_temperature and not self._enable_pressure:
            return
        temperature = self.get_temperature()
        if self._enable_temperature and not self._enable_pressure:
            return temperature
        if self._enable_pressure and not self._enable_temperature:
            return self.get_pressure()
        return self.get_pressure(), temperature
