# micropython
# mail: goctaprog@gmail.com
# MIT license
import array
import micropython
from micropython import const
from collections import namedtuple
from sensor_pack_2 import bus_service
from sensor_pack_2.bmp_common import (IBaseAirPresSensor, OversamplingCoeff, MeasChannels,
                                      MeasuredParams, SensorID, SensorMode)
from sensor_pack_2.base_sensor import Iterator, DeviceEx, check_value

# ВНИМАНИЕ: не подключайте питание датчика к 5В, иначе датчик выйдет из строя! Только 3.3В!!!
# WARNING: do not connect "+" to 5V or the sensor will be damaged!

# Внутренние адреса регистров BMP390
_REG_CHIP_ID    = const(0x00)
# _REG_REV_ID     = const(0x01)
_REG_ERR_REG    = const(0x02)
_REG_STATUS     = const(0x03)
_REG_PRESS_DATA = const(0x04)  # 0x04..0x06
_REG_TEMP_DATA  = const(0x07)  # 0x07..0x09
_REG_SENSORTIME = const(0x0C)  # 0x0C..0x0E
_REG_EVENT      = const(0x10)
_REG_INT_STATUS = const(0x11)
_REG_FIFO_LEN   = const(0x12)  # 0x12..0x13
#_REG_FIFO_CFG_1 = const(0x17)
#_REG_FIFO_CFG_2 = const(0x18)
#_REG_INT_CTRL   = const(0x19)
#_REG_IF_CONF    = const(0x1A)
_REG_PWR_CTRL   = const(0x1B)
_REG_OSR        = const(0x1C)
_REG_ODR        = const(0x1D)
_REG_CONFIG     = const(0x1F)
_REG_CMD        = const(0x7E)
_REG_CALIB_START = const(0x31)

# для расчета времени преобразования
_T_SETUP = const(234)
_T_BASE_TEMP = const(163)
_T_BASE_PRESS = const(392)
_T_PHASE = const(2020)

@micropython.native
def _calibration_regs_addr() -> iter:
    """возвращает кортеж из адреса регистра, размера значения в байтах, типа значения (u-unsigned, s-signed)"""
    start_addr = _REG_CALIB_START
    tpl = ('1b', '2h', '2H')
    # возвращает итератор с адресами внутренних регистров датчика, хранящих калибровочные коэффициенты
    val_type = "22011002200100"
    for item in val_type:
        v_size, v_type = tpl[int(item)]
        yield int(start_addr), int(v_size), v_type
        start_addr += int(v_size)

def _mode_to_raw_mode(mode: int) -> int:
    """Преобразует постоянные режима класса SensorMode в сырое значение,
    которое соответствует значению датчика.
    SensorMode  raw_mode    Описание
    0 (SLEEP)       0       Sleep
    1 (FORCED)      2       Forced
    2 (NORMAL)      1       Normal
    """
    if SensorMode.FORCED == mode:
        return 1
    if SensorMode.NORMAL == mode:
        return 3
    return mode # 0 - SLEEP

def _raw_mode_to_mode(raw_mode: int) -> int:
    """Преобразует сырое значение режима работы датчика в постоянные режима класса SensorMode.
        raw_mode    SensorMode      Описание
        0           0 (SLEEP)       Sleep
        1, 2        1 (FORCED)      Forced
        3           2 (NORMAL)      Normal
        """
    if 1 == raw_mode or 2 == raw_mode:
        return SensorMode.FORCED
    if 3 == raw_mode:
        return SensorMode.NORMAL
    return SensorMode.SLEEP

data_status_bmp390 = namedtuple("data_status_bmp390", "temp_ready press_ready cmd_decoder_ready")
int_status_bmp390 = namedtuple("int_status_bmp390", "data_ready fifo_is_full fifo_watermark")
event_bmp390 = namedtuple("event__bmp390", "itf_act_pt por_detected")
# Bit 0 - fatal_err Fatal error
# Bit 1 - Command execution failed
# Bit 2 - conf_err; sensor configuration error detected (only working in normal mode). Cleared on read.
error_flags_bmp390 = namedtuple("error_flags_bmp390", "fatal_err cmd_exec_failed conf_err")

class Bmp390(IBaseAirPresSensor, Iterator):
    """Class for work with Bosh BMP390 pressure sensor."""

    def __init__(self, adapter: bus_service.BusAdapter, address=0x77,
                 oversample_temp=0b11, oversample_press=0b11, iir_filter=0):
        """i2c - объект класса I2C; baseline_pressure - давление на уровне моря в Pa в твоей(!) местности;;
        oversample_settings (0..5) - точность измерения 0-грубо но быстро, 5-медленно, но точно;
        address - адрес датчика;
        iir_filter=0..7; 0 - off, 7 - max value

        i2c is an object of the I2C class; baseline_pressure - sea level pressure in Pa in your(!) area;
        oversample_settings (0..5) - measurement reliability 0-coarse but fast, 5-slow but accurate;"""
        # super().__init__(adapter, address, False)
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=False)
        self._buf_2 = bytearray(2)  # для _read_buf_from_mem
        self._buf_3 = bytearray(3)  # для _read_buf_from_mem
        self._t_lin = None  # for pressure calculation
        # for temperature only!
        self._oss_t = check_value(oversample_temp, range(6),
                                   f"Invalid temperature oversample value: {oversample_temp}")
        self._oss_p = check_value(oversample_press, range(6),
                                   f"Invalid pressure oversample value: {oversample_press}")
        self._adapter = adapter
        self._IIR = check_value(iir_filter, range(8),
                                 f"Invalid iir_filter value: {iir_filter}")
        self._mode = SensorMode.SLEEP  # sleep mode
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

    @staticmethod
    def _check_cc(index: int):
        """Проверяет на верность индекс калибровочного коэффициента."""
        check_value(index, range(14), f"Invalid index value: {index}")

    def get_calibration(self, index: int = None) -> int:
        """возвращает калибровочный коэффициент по его индексу (0..13).
        returns the calibration coefficient by its index (0..13)"""
        if index is None:
            return len(self._cfa)
        self._check_cc(index)
        return self._cfa[index]

    def refresh_config(self) -> None:
        """Считывает текущие настройки из регистров датчика в поля экземпляра класса."""
        reg_osr = self._connection.read_reg(_REG_OSR, 1)[0]
        self._oss_p = reg_osr & 0b111
        self._oss_t = (reg_osr >> 3) & 0b111

        reg_config = self._connection.read_reg(_REG_CONFIG, 1)[0]
        self._IIR = (reg_config >> 1) & 0b111  # биты 3:1

        reg_odr = self._connection.read_reg(_REG_ODR, 1)[0]
        self._sampling_period = reg_odr & 0b11111

        reg_pwr = self._connection.read_reg(_REG_PWR_CTRL, 1)[0]
        self._mode = _raw_mode_to_mode((reg_pwr >> 4) & 0b11)
        self._enable_pressure = bool(reg_pwr & 0b01)
        self._enable_temperature = bool(reg_pwr & 0b10)

    @micropython.native
    def _precalculate(self):
        """предварительно вычисленные значения"""
        get_cc = self.get_calibration
        # для расчета температуры
        self.par_t1 = get_cc(0) * 2 ** 8  #
        self.par_t2 = get_cc(1) / 2 ** 30  #
        self.par_t3 = get_cc(2) / 2 ** 48  #
        # для расчета давления
        self.par_p1 = (get_cc(3) - 2 ** 14) / 2 ** 20
        self.par_p2 = (get_cc(4) - 2 ** 14) / 2 ** 29
        self.par_p3 = get_cc(5) / 2 ** 32
        self.par_p4 = get_cc(6) / 2 ** 37
        self.par_p5 = get_cc(7) * 8
        self.par_p6 = get_cc(8) / 2 ** 6
        self.par_p7 = get_cc(9) / 2 ** 8
        self.par_p8 = get_cc(10) / 2 ** 15
        self.par_p9 = get_cc(11) / 2 ** 48
        self.par_p10 = get_cc(12) / 2 ** 48
        self.par_p11 = get_cc(13) / 2 ** 65

    @staticmethod
    @micropython.native
    def _validate_cc(index: int, value: int) -> tuple[bool, str | None]:
        """Проверка значения калибровочного коэффициента."""
        # 0x0000 или 0xFFFF/-1 в NVM обычно означают пустую ячейку или ошибку чтения
        # После unpack: 0 -> 0, 0xFFFF(signed) -> -1, 0xFFFF(unsigned) -> 65535
        if value == 0 or value == -1 or value == 0xFFFF:
            return False, f"Invalid NVM pattern 0x{value & 0xFFFF:04X} at index {index}"
        #
        return True, None

    def _read_calibration_data(self) -> int:
        """Читает калибровочные значение из датчика.
        read calibration values from sensor.
        return count read values"""
        if any(self._cfa):
            raise ValueError(f"calibration data array already filled!")
        _conn = self._connection
        index = 0
        for v_addr, v_size, v_type in _calibration_regs_addr():
            # print(v_addr, v_size, v_type)
            reg_val = _conn.read_reg(reg_addr=v_addr, bytes_count=v_size)
            rv = _conn.unpack(fmt_char=f"{v_type}", source=reg_val)[0]
            # check
            is_ok, err_msg = Bmp390._validate_cc(index, rv)
            if not is_ok:
                raise ValueError(f"Calibration coeff #{index} @0x{v_addr:02X}: {err_msg}")
            self._cfa[index] = rv
            index += 1
        return len(self._cfa)

    # IDentifier
    def get_id(self) -> SensorID:
        """Возвращает идентификатор датчика и его revision ID.
        Returns the ID and revision ID of the sensor."""
        buf = self._buf_2
        self._connection.read_buf_from_mem(address=_REG_CHIP_ID, buf=buf, address_size=1)
        # chip id, rev_id
        return SensorID(buf[0], buf[1], None, None)

    def soft_reset(self, reset_or_flush: bool = True):
        """программный сброс датчика.
        software reset of the sensor"""
        value = 0xB6 if reset_or_flush else 0xB0
        self._connection.write_reg(reg_addr=_REG_CMD, value=value, bytes_count=1)

    def get_error(self, raw: bool = True) -> int | error_flags_bmp390:
        """Возвращает три бита состояния ошибок.
        Bit 0 - fatal_err Fatal error
        Bit 1 - Command execution failed. Cleared on read.
        Bit 2 conf_err sensor configuration error detected (only working in normal mode). Cleared on read.
        """
        err = 0b111 & self._connection.read_reg(reg_addr=_REG_ERR_REG, bytes_count=1)[0]
        if raw:
            return err
        return error_flags_bmp390(fatal_err=0b001 & err, cmd_exec_failed=0b010 & err, conf_err=0b100 & err)

    def get_data_status(self, raw: bool = True) -> int | data_status_bmp390:
        """Возвращает три бита состояния датчика как кортеж
        Data ready for temperature, Data ready for pressure, CMD decoder status
        бит 4 - CMD decoder status (Command decoder ready (1 = готов принять команду))
        бит 5 - Data ready for pressure (сбрасывается при чтении регистра давления)
        бит 6 - Data ready for temperature (сбрасывается при чтении регистра температуры)
        """
        val = self._connection.read_reg(_REG_STATUS, 1)[0]
        if raw:
            return val
        i = 0x07 & (val >> 4)
        drdy_temp, drdy_press, cmd_rdy = 0 != 0x04 & i, 0 != 0x02 & i, 0 != 0x01 & i
        return data_status_bmp390(temp_ready=drdy_temp, press_ready=drdy_press, cmd_decoder_ready=cmd_rdy)

    @micropython.native
    def _get_pressure_raw(self) -> int:
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._connection.read_buf_from_mem(address=_REG_PRESS_DATA, buf=buf, address_size=1)
        return (h << 16) | (m << 8) | l

    def get_pressure(self) -> float:
        """Return pressure in Pascal [Pa].
        Call get_temperature() before call get_pressure() !!!"""
        uncompensated = self._get_pressure_raw()
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
    def _get_temperature_raw(self) -> int:
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._connection.read_buf_from_mem(address=_REG_TEMP_DATA, buf=buf, address_size=1)
        return (h << 16) | (m << 8) | l

    def get_temperature(self) -> float:
        """Return temperature in Celsius"""
        uncompensated = self._get_temperature_raw()
        partial_data1 = uncompensated - self.par_t1
        partial_data2 = partial_data1 * self.par_t2
        # Update the compensated temperature since this is needed for pressure calculation !!!
        self._t_lin = partial_data2 + (partial_data1 * partial_data1) * self.par_t3
        return self._t_lin

    @micropython.native
    def get_sensor_time(self) -> int:
        """Возвращает внутреннее время датчика!
        Считывает внутренний аппаратный счётчик времени датчика (24-битное значение).
        Согласно документации, счётчик увеличивается на 1 каждые ~40 мс (частота 25 Гц).
        По монотонному изменению значения можно убедиться, что внутренний тактовый генератор работает и датчик не завис!"""
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._connection.read_buf_from_mem(address=_REG_SENSORTIME, buf=buf, address_size=1)
        return (h << 16) | (m << 8) | l

    def get_event(self) -> event_bmp390:
        """Bit 0 por_detected ‘1’ after device power up or softreset. Clear-on-read
        Bit 1 itf_act_pt ‘1’ when a serial interface transaction occurs during a
        pressure or temperature conversion. Clear-on-read"""
        _evt = 0b11 & self._connection.read_reg(reg_addr=_REG_EVENT, bytes_count=1)[0]
        return event_bmp390(itf_act_pt=bool(0b10 & _evt), por_detected=bool(0b01 & _evt))

    def get_int_status(self, raw: bool = True) -> int | int_status_bmp390:
        """Bit 0 fwm_int FIFO Watermark Interrupt
        Bit 1 full_int FIFO Full Interrupt
        Bit 3 drdy data ready interrupt"""
        int_stat = 0b1011 & self._connection.read_reg(reg_addr=_REG_INT_STATUS, bytes_count=1)[0]
        if raw:
            return int_stat
        return int_status_bmp390(data_ready=bool(0b1000 & int_stat),
                                 fifo_is_full=bool(0b010 & int_stat),
                                 fifo_watermark=bool(0b0001 & int_stat))

    def get_fifo_length(self) -> int:
        """The FIFO byte counter indicates the current fill level of the FIFO buffer."""
        buf = self._buf_2
        self._connection.read_buf_from_mem(address=_REG_FIFO_LEN, buf=buf, address_size=1)
        return self._connection.unpack(fmt_char="H", source=buf)[0]

    def start_measurement(self):
        """ # mode: 0 - sleep, 1-forced, 2-normal (continuously)"""
        tmp = 0
        if self._enable_pressure:
            tmp |= 0b01
        if self._enable_temperature:
            tmp |= 0b10
        #
        rm = _mode_to_raw_mode(self._mode)
        tmp |= rm << 4  # режим
        # записываю в датчик. АЦП запускается автоматически при выходе из sleep mode.
        self._connection.write_reg(reg_addr=_REG_PWR_CTRL, value=tmp, bytes_count=1)

    def set_power_mode(self,  value: int | None = None) -> int:
        """Устанавливает или возвращает режим работы датчика.
        Если value is None -> считывает текущий режим из регистра PWR_CTRL (0x1B)
        и возвращает его (0=sleep, 1=forced, 2=normal). Кэш self._mode обновляется.
        Иначе -> сохраняет указанный режим во внутренний кэш.
        Запись в регистр PWR_CTRL произойдёт только при вызове start_measurement().

        Args:
            value (int | None): Режим работы. None = прочитать из регистра.
        Returns:
            int | None: Текущий режим, если value=None, иначе None.
        Raises:
            ValueError: Если value не в диапазоне 0..2."""
        if value is None:
            # reg = self._connection.read_reg(_REG_PWR_CTRL, 1)[0]
            # raw_mode = (reg >> 4) & 0b11  # сырые биты: 0, 1 или 3
            # self._mode = _raw_mode_to_mode(raw_mode)
            return self._mode

        if not value in range(3):
            raise ValueError(f"Invalid mode value: {value}")
        self._mode = value
        return value

    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement"""
        return SensorMode.FORCED == self.set_power_mode(None)

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement"""
        return SensorMode.NORMAL == self.set_power_mode(None)

    def set_oversampling(self, temp: int | None = None, press: int | None = None) -> OversamplingCoeff:
        """Устанавливает oversampling. Записывает в регистр OSR (0x1C) только если заданы параметры.
        Всегда считывает текущее состояние из регистра, обновляет кэш и возвращает OversamplingCoeff."""
        if temp is not None or press is not None:
            valid_rng = range(6)
            # Берём недостающие значения из кэша, чтобы сформировать полный байт без чтения регистра
            t = check_value(temp, valid_rng, f"Invalid temperature oversample: {temp}") if temp is not None else self._oss_t
            p = check_value(press, valid_rng, f"Invalid pressure oversample: {press}") if press is not None else self._oss_p
            self._connection.write_reg(_REG_OSR, (t << 3) | p, 1)

        # чтение и возврат
        reg = self._connection.read_reg(_REG_OSR, 1)[0]
        self._oss_t = (reg >> 3) & 0b111
        self._oss_p = reg & 0b111
        return OversamplingCoeff(temperature=self._oss_t, pressure=self._oss_p)

    def set_sampling_period(self, value: int | None = None) -> int:
        """Устанавливает или возвращает период дискретизации (ODR).
        Если period != None -> записывает значение в регистр 0x1D.
        Всегда считывает текущее состояние из регистра, обновляет кэш и возвращает его."""
        if value is not None:
            p = check_value(value, range(18), f"Invalid value output data rates: {value}")
            self._connection.write_reg(reg_addr=_REG_ODR, value=p, bytes_count=1)
        # Читаю состояние из регистра
        val = self._connection.read_reg(_REG_ODR, 1)[0] & 0b11111
        self._sampling_period = val
        return val

    def set_iir_filter(self, temp: int | None = None, press: int | None = None) -> tuple[int | None, int | None]:
        """Устанавливает коэффициент ФНЧ.
        BMP390 имеют один общий фильтр для T и P (регистр CONFIG, биты 3:1)."""

        if temp is not None or press is not None:
            # Выбираю значение
            val = temp if temp is not None else press
            c = check_value(val, range(8), f"Invalid IIR filter: {val}")

            # Читаю CONFIG
            reg = self._connection.read_reg(_REG_CONFIG, 1)[0]
            # Маска 0xF1 сбрасывает биты 3,2,1. Оставляю биты 7..4 и 0
            reg = (reg & 0xF1) | (c << 1)

            self._connection.write_reg(_REG_CONFIG, reg, 1)

        # Читаем подтверждённое значение
        reg = self._connection.read_reg(_REG_CONFIG, 1)[0]
        iir_val = (reg >> 1) & 0x07

        return iir_val, iir_val

    def set_channels(self, temp_en=None, press_en=None) -> MeasChannels | None:
        """Включает/выключает каналы. Если аргументы != None -> записывает в PWR_CTRL (0x1B).
        Всегда считывает текущее состояние из регистра, обновляет кэш и возвращает MeasChannels."""
        if temp_en is not None or press_en is not None:
            reg = self._connection.read_reg(_REG_PWR_CTRL, 1)[0] & 0b1111_1100
            if temp_en is not None: self._enable_temperature = temp_en
            if press_en is not None: self._enable_pressure = press_en
            if self._enable_temperature: reg |= 0b10
            if self._enable_pressure: reg |= 0b01
            self._connection.write_reg(_REG_PWR_CTRL, reg, 1)

        # Обязательное чтение и возврат
        reg = self._connection.read_reg(_REG_PWR_CTRL, 1)[0]
        self._enable_temperature = bool(reg & 0b10)
        self._enable_pressure = bool(reg & 0b01)
        return MeasChannels(temperature=self._enable_temperature, pressure=self._enable_pressure)

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """возвращает время преобразования в [мс] датчиком температуры или давления в зависимости от его настроек."""
        total = _T_SETUP

        if self._enable_temperature:
            total += _T_BASE_TEMP + (_T_PHASE * (1 << self._oss_t))

        if self._enable_pressure:
            total += _T_BASE_PRESS + (_T_PHASE * (1 << self._oss_p))

        return 1 + (total // 1000)

    # Iterator
    def __next__(self) -> None | MeasuredParams:
        if not self.is_continuously_mode():
            return None
        temperature = self.get_temperature()
        if self._enable_temperature and not self._enable_pressure:
            return MeasuredParams(temperature=temperature, pressure=None)
        if self._enable_pressure and not self._enable_temperature:
            return MeasuredParams(temperature=None, pressure=self.get_pressure())
        return MeasuredParams(temperature=temperature, pressure=self.get_pressure())

    def is_data_ready(self) -> bool:
        mask = 0x60 # 0b01100000 -> биты 5 (drdy_pres) и 6 (drdy_temp)
        raw_ds = self.get_data_status(raw=True)
        print(f"DBG: raw_ds: 0x{raw_ds:x}")
        return mask == (raw_ds & mask)
