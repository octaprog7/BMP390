# micropython
# mail: goctaprog@gmail.com
# MIT license
import ustruct
import micropython
import array

# ВНИМАНИЕ: не подключайте питание датчика к 5В, иначе датчик выйдет из строя! Только 3.3В!!!
# WARNING: do not connect "+" to 5V or the sensor will be damaged!


# @micropython.native
def _check_value(value: int, valid_range, error_msg: str) -> int:
    if value not in valid_range:
        raise ValueError(error_msg)
    return value


def _calibration_regs_addr() -> iter:
    """возвращает кортеж из адреса регистра, размера значения в байтах, типа значения (u-unsigned, s-signed)"""
    start_addr = 0x31
    tpl = ('1b', '2h', '2H')
    """возвращает итератор с адресами внутренних регистров датчика, хранящих калибровочные коэффициенты """
    val_type = "22011002200100"
    for item in val_type:
        v_size, v_type = tpl[int(item)]
        yield start_addr, v_size, v_type
        start_addr += int(v_size)


class Bmp390:
    """Class for work with Bosh BMP180 pressure sensor"""

    def __init__(self, i2c, address=0xEE >> 1, baseline_pressure=101325.0, oversample_temperature=0b11, iir_filter=0):
        """i2c - объект класса I2C; baseline_pressure - давление на уровне моря в Pa в твоей(!) местности;;
        oversample_settings (0..5) - точность измерения 0-грубо но быстро, 5-медленно, но точно;
        address - адрес датчика (0xEF (read) and 0xEE (write) from datasheet)
        iir_filter=0..7; 0 - off, 7 - max value

        i2c is an object of the I2C class; baseline_pressure - sea level pressure in Pa in your(!) area;
        oversample_settings (0..5) - measurement reliability 0-coarse but fast, 5-slow but accurate;"""
        self.press4 = None  # for precalculate
        self.press3 = None
        self.press2 = None
        self.press1 = None
        self.press0 = None
        self.tmp1 = None
        self.tmp0 = None
        self.B5 = None   # for precalculate
        # for temperature only!
        self.oss = _check_value(oversample_temperature, range(0, 6),
                                f"Invalid oversample value: {oversample_temperature}")
        self.adr = address
        self.i2c = i2c
        self.IIR = _check_value(iir_filter, range(0, 8),
                                f"Invalid iir_filter value: {iir_filter}")
        self.base_pressure = baseline_pressure
        # массив, хранящий калибровочные коэффициенты (xx штук)
        self.cfa = array.array("l")  # signed long elements
        # считываю калибровочные коэффициенты
        self._read_calibration_data()
        # предварительный расчет
        self.precalculate()

    def get_calibration_data(self, index: int) -> int:
        """возвращает калибровочный коэффициент по его индексу (0..13).
        returns the calibration coefficient by its index (0..10)"""
        self._check_value(index, range(0, 14), f"Invalid index value: {index}")
        return self.cfa[index]

    def precalculate(self):
        """предварительно вычисленные значения"""
        # для расчета температуры
        self.tmp0 = self.get_calibration_data(4) / 2 ** 15  #
        self.tmp1 = self.get_calibration_data(9) * 2 ** 11  #
        # для расчета давления
        self.press0 = self.get_calibration_data(7) / 2 ** 23
        self.press1 = self.get_calibration_data(1) / 2 ** 11
        self.press2 = self.get_calibration_data(2) / 2 ** 13
        self.press3 = self.get_calibration_data(6) / 2 ** 28
        self.press4 = abs(self.get_calibration_data(3)) / 2 ** 15

    def _read_register(self, reg_addr, bytes_count=2) -> bytes:
        """считывает из регистра датчика значение.
        bytes_count - размер значения в байтах"""
        return self.i2c.readfrom_mem(self.adr, reg_addr, bytes_count)

    def _write_register(self, reg_addr, value: int, bytes_count=2) -> int:
        """записывает данные value в датчик, по адресу reg_addr.
        bytes_count - кол-во записываемых данных"""
        buf = value.to_bytes(bytes_count, "big")
        return self.i2c.writeto_mem(self.adr, reg_addr, buf)

    def _read_calibration_data(self) -> int:
        """Читает калибровочные значение из датчика.
        read calibration values from sensor.
        return count read values"""
        if len(self.cfa):
            raise ValueError(f"calibration data array already filled!")
        for v_addr, v_size, v_type in _calibration_regs_addr():
            reg_val = self._read_register(v_addr, v_size)
            rv = ustruct.unpack(f">{v_type}", reg_val)[0]
            # check
            if rv == 0x00 or rv == 0xFFFF:
                raise ValueError(f"Invalid register addr: {v_addr} value: {hex(rv)}")
            self.cfa.append(rv)
        return len(self.cfa)

    def get_chip_id(self) -> tuple:
        """Возвращает идентификатор датчика и его revision ID.
        Returns the ID and revision ID of the sensor."""
        chip_id = self._read_register(0x00, 1)
        rev_id = self._read_register(0x01, 1)
        return int(chip_id[0]), int(rev_id[0])

    def get_error(self) -> int:
        """Возвращает три бита состояния ошибок.
        Bit 0 - fatal_err Fatal error
        Bit 1 - Command execution failed. Cleared on read.
        Bit 2 conf_err sensor configuration error detected (only working in normal mode). Cleared on read.
        """
        err = self._read_register(0x02, 1)
        return int(err[0]) & 0x07

    def get_status(self) -> int:
        """Возвращает три бита состояния датчика
        бит 0 - CMD decoder status (0: Command in progress; 1: Command decoder is ready to accept a new command)
        бит 1 - Data ready for pressure. (It gets reset, when one pressure DATA register is read out)
        бит 2 - Data ready for temperature sensor. (It gets reset, when one temperature DATA register is read out)
        """
        val = self._read_register(0x03, 1)
        return (int(val[0]) >> 4) & 0x07

    def get_uncompensated_pressure(self) -> int:
        # трех байтовое значение
        val = self._read_register(0x04, 3)
        return ustruct.unpack(">H", val)[0]

    def get_uncompensated_temperature(self) -> int:
        # трех байтовое значение
        val = self._read_register(0x07, 3)
        return ustruct.unpack(">h", val)[0]


    def soft_reset(self):
        """программный сброс датчика.
        software reset of the sensor"""
        self._write_register(0xE0, 0xB6, 1)

    def start_measurement(self, temperature_or_pressure: bool = True):
        """Start measurement process in sensor.
        Если temperature_or_pressure==Истина тогда будет выполнен запуск измерения температуры иначе давления!
        Вы должны подождать результата 5 мс после запуска измерения температуры.
        Время ожидания результата после запуска измерения давления зависит от переменной self.осс.
        self.оss     задержка, мс
        0               5
        1               8
        2               14
        3               26

        Внимание! Глупо ждать результата методом time.delay_ms(value)!
        Вместо этого можно занять процессор полезным делом!"""
        loc_oss = self.oss
        start_conversion = 0b00100000   # bit 5 - запуск преобразования (1)
        bit_4_0 = 0x14  # давление
        if temperature_or_pressure:
            bit_4_0 = 0x0E  # температура
            loc_oss = 0  # обнуляю OSS при температуре
        val = loc_oss << 6 | start_conversion | bit_4_0
        self._write_register(0xF4, val, 1)

    def get_temperature(self) -> float:
        """возвращает значение температуры, измеренное датчиком в Цельсиях.
        returns the temperature value measured by the sensor in Celsius"""
        raw = self._read_register(0xF6, 2)  # считывание сырого значения
        temp = ustruct.unpack(">H", raw)[0]  # unsigned short
        a = self.tmp0 * (temp - self.get_calibration_data(5))
        b = self.tmp1 / (a + self.get_calibration_data(10))
        self.B5 = a + b  #
        return 6.25E-3 * (a + b + 8)

    def get_pressure(self) -> float:
        """возвращает значение давления, измеренное датчиком в Паскалях (Pa).
        До вызова этого метода нужно вызвать хотя-бы один раз метод get_temperature.
        Лучше вызывайте метод парами:
        get_temperature
        get_pressure

        returns the pressure value measured by the sensor in Pascals (Pa).
        Before calling this method, you need to call the get_temperature method at least once.
        Better call the method in pairs:
        get_temperature
        get_pressure"""
        raw = self._read_register(0xF6, 3)  # считывание сырого значения (три байта)
        msb, lsb, xlsb = raw
        uncompensated = ((msb << 16)+(lsb << 8)) >> (8-self.oss)
        b6 = self.B5-4000
        x1 = self.press0 * b6 ** 2  #
        x2 = self.press1 * b6
        x3 = x1 + x2
        b3 = (2 + ((x3 + 4 * self.get_calibration_data(0)) * 2**self.oss)) / 4

        x1 = b6 * self.press2
        x2 = self.press3 * b6 ** 2
        x3 = (2+x1+x2) / 4

        b4 = self.press4 * (x3+32768)
        b7 = (abs(uncompensated)-b3) * (50000 / 2**self.oss)

        curr_pressure = 2 * b7 / b4
        x1 = 7.073394953E-7 * curr_pressure ** 2
        x2 = -0.1122589111328125 * curr_pressure

        return curr_pressure + 6.25E-2 * (x1 + x2 + 3791)