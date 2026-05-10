# MIT license
# Copyright (c) 2022-2026 Roman Shevchik

"""
Соглашение для методов set_*:
    - Если все аргументы == None -> метод работает как геттер, возвращает текущее значение.
    - Если передан хотя бы один аргумент != None -> метод работает как сеттер, возвращает None.
    - Пример:
        bmp.set_oversampling(None, None)  # read -> OversamplingCoeff(0, 3)
        bmp.set_oversampling(press=2)     # write -> None
"""

from collections import namedtuple
from sensor_pack_2.base_sensor import IBaseSensorEx, IDentifier

# настройки oversampling (int, int)
OversamplingCoeff = namedtuple("OversamplingCoeff", "temperature pressure")
# возвращает активность каналов измерения (Истина->канал активен)
MeasChannels = namedtuple("MeasChannels", "temperature pressure")
MeasuredParams = namedtuple("MeasuredParams", "temperature pressure")
# Универсальный идентификатор датчика давления
# Неиспользуемые значения = None
SensorID = namedtuple("SensorID","chip_id revision_id spare1 spare2")

class SensorMode:
    """режимы работы всех датчиков серии BMP"""
    SLEEP      = 0  # Сон / Standby
    FORCED     = 1  # Однократное измерение
    NORMAL     = 2  # Периодическое непрерывное
    CONTINUOUS = 3  # Непрерывное на макс. частоте (для BMP581)


class IBMPCommon:
    """
    Унифицированный интерфейс для датчиков давления атмосферного воздуха.

    Для документации:
        get_calibration(index: int) -> int
        set_oversampling(temp: int|None, press: int|None) -> (int, int)
        set_iir_filter(coeff: int) -> int
        set_channels(temp_en: bool|None, press_en: bool|None) -> (bool, bool)
        refresh_config() -> None
    """

    def get_calibration(self, index: int | None) -> int:
        """Возвращает калибровочный коэффициент по индексу (int).
        Если index is None, возвращает кол-во калибровочных коэффициентов.
        Коэффициенты считываются из датчика 'приватным' методом."""
        raise NotImplementedError()

    def set_oversampling(self, temp: int | None = None, press: int | None = None) -> None | OversamplingCoeff:
        """
        Устанавливает oversampling. None = не менять.
        Возвращает фактические значения, если temp и pressure в None: (temp, pressure) -> (int, int).
        """
        raise NotImplementedError()

    def set_iir_filter(self, temp: int | None = None, press: int | None = None) -> tuple[int, int]:
        """Устанавливает коэффициенты ФНЧ для давления и/или температуры.

        - Если оба параметра None -> читает и возвращает текущие значения (temp_iir, press_iir)
        - Если передан хотя бы один параметр -> записывает и возвращает подтверждённые значения

        Args:
            press (int | None): Коэффициент для давления (0..N). 0=off, N=max. None = не менять.
            temp (int | None): Коэффициент для температуры (0..N). None = не менять.

        Returns:
            tuple[int, int]: (temp_iir, press_iir) — фактические значения из регистра.

        Raises:
            NotImplementedError: Если датчик не поддерживает ФНЧ.
        """
        raise NotImplementedError()

    def set_channels(self, temp_en, press_en) -> None | MeasChannels:
        """
        Включает/выключает каналы измерения. None = не менять.
        Возвращает фактические состояния, если temp_en и press_en в None: (temp_en, press_en) -> (bool, bool).
        """
        raise NotImplementedError()

    def refresh_config(self):
        """Перечитывает настройки из регистров датчика во внутренний кэш."""
        raise NotImplementedError()

    def set_power_mode(self, value: int | None = None) -> int:
        """Устанавливает или возвращает текущий режим питания.

        Стандартная маппинг режимов для всех датчиков серии BMP:
            0: Sleep / Standby (Сон)
            1: Forced (Однократное измерение по запросу)
            2: Normal (Периодическое непрерывное измерение)
            3: Continuous (Непрерывное на макс. частоте - опционально, только BMP581)

        Args:
            value (int | None): Код режима. None = прочитать текущее состояние.

        Returns:
            int: значение режима из регистра.

        Raises:
            ValueError: Если значение не входит в допустимый диапазон для конкретного датчика.
        """
        raise NotImplementedError()

    def set_sampling_period(self, value: int | None = None) -> int:
        """Устанавливает или возвращает период дискретизации (ODR).

        Args:
            value (int | None): Период в [мс]. None = прочитать текущее значение.

        Returns:
            int: Фактический период в [мс] или ближайший поддерживаемый.
        """
        raise NotImplementedError()


class IBaseAirPresSensor(IBaseSensorEx, IDentifier, IBMPCommon):
    """Интерфейс для всех барометрических датчиков Bosch и не только их.
    Example:
        >>> sensor: IBaseAirPresSensor = Bmp581(adapter)
        >>> sensor.start_measurement()
        >>> if sensor.is_data_ready():
        ...     t = sensor.get_temperature()
        ...     p = sensor.get_pressure()
    """

    def is_data_ready(self) -> bool:
        """Проверяет готовность новых данных температуры и/или давления.

         Возвращает `True`, если датчик завершил преобразование и данные
         доступны для чтения. В противном случае возвращает `False`.

         Returns:
             bool: `True` если данные готовы, `False` если измерение ещё
                   выполняется или датчик находится в режиме ожидания.
         """
        raise NotImplementedError()

    def get_temperature(self) -> float:
        """Возвращает температуру окружающего воздуха в градусах Цельсия.
        Returns:
             float: Температура в градусах Цельсия [°C].

         Raises:
             OSError: Если шина недоступна или датчик не отвечает.
             RuntimeError: Если данные не готовы (рекомендуется проверять
                           `is_data_ready()` перед вызовом).
         """
        raise NotImplementedError()

    def get_pressure(self) -> float:
        """Возвращает компенсированное атмосферное давление.
         Returns:
             float: Давление в Паскалях [Па].

         Raises:
             OSError: Если шина недоступна или датчик не отвечает.
             RuntimeError: Если канал давления отключён или данные не готовы.

         Note:
             Для перевода в мм рт. ст., гПа, PSI или атм используйте
             вспомогательные функции преобразования в прикладном коде.
         """
        raise NotImplementedError()