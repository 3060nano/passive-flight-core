#pragma once

namespace passive_flight {

/**
 * Параметры состояния атмосферы в заданной точке траектории.
 *
 * Все величины заданы в системе СИ.
 */
struct AtmosphereState {
    // Геометрическая высота над уровнем моря, м.
    double geometricAltitudeM{0.0};

    // Геопотенциальная высота, используемая в формулах атмосферы, м.
    double geopotentialAltitudeM{0.0};

    // Температура воздуха, К.
    double temperatureK{288.15};

    // Статическое давление воздуха, Па.
    double pressurePa{101325.0};

    // Плотность воздуха, кг/м^3.
    double densityKgM3{1.225};

    // Скорость звука, м/с.
    double speedOfSoundMps{340.294};

    // Динамическая вязкость воздуха, Па·с.
    double dynamicViscosityPaS{1.7894e-5};
};

/**
 * Набор констант, используемых моделью стандартной атмосферы.
 *
 * Константы вынесены в отдельную структуру, чтобы в дальнейшем можно было:
 * 1. использовать другую атмосферную модель;
 * 2. проводить исследование чувствительности;
 * 3. согласовать параметры с моделью преподавателя или SimInTech.
 */
struct StandardAtmosphereParameters {
    // Температура на уровне моря, К.
    double seaLevelTemperatureK{288.15};

    // Давление на уровне моря, Па.
    double seaLevelPressurePa{101325.0};

    // Ускорение свободного падения, м/с^2.
    double gravityMps2{9.80665};

    // Удельная газовая постоянная сухого воздуха, Дж/(кг·К).
    double specificGasConstantJkgK{287.05287};

    // Отношение теплоёмкостей воздуха.
    double heatCapacityRatio{1.4};

    // Температурный градиент в тропосфере, К/м.
    double troposphericLapseRateKPerM{-0.0065};

    // Верхняя граница тропосферы по геометрической высоте, м.
    double troposphereTopGeometricAltitudeM{11000.0};

    // Наибольшая допустимая геометрическая высота модели, м.
    double maximumGeometricAltitudeM{25000.0};

    // Средний радиус Земли, м.
    double earthRadiusM{6371210.0};

    // Опорная температура в формуле Сазерленда, К.
    double sutherlandReferenceTemperatureK{273.15};

    // Динамическая вязкость при опорной температуре, Па·с.
    double sutherlandReferenceViscosityPaS{1.716e-5};

    // Постоянная Сазерленда для воздуха, К.
    double sutherlandConstantK{110.4};
};

/**
 * Модель стандартной атмосферы.
 *
 * Реализованы два слоя:
 * 1. тропосфера с линейным уменьшением температуры;
 * 2. изотермический слой выше тропопаузы.
 *
 * Текущий рабочий диапазон: от 0 до 25 000 м.
 */
class StandardAtmosphere {
public:
    StandardAtmosphere();

    explicit StandardAtmosphere(
        const StandardAtmosphereParameters& parameters
    );

    /**
     * Вычисляет параметры атмосферы по геометрической высоте.
     *
     * Отрицательная высота приравнивается к нулю. Это необходимо,
     * поскольку при численном интегрировании последний шаг может
     * незначительно увести объект ниже поверхности земли.
     *
     * @param geometricAltitudeM геометрическая высота, м.
     *
     * @return параметры атмосферы в заданной точке.
     *
     * @throws std::invalid_argument, если высота не является конечным числом.
     * @throws std::out_of_range, если высота превышает допустимый диапазон.
     */
    [[nodiscard]]
    AtmosphereState evaluate(double geometricAltitudeM) const;

    [[nodiscard]]
    const StandardAtmosphereParameters& parameters() const noexcept;

private:
    StandardAtmosphereParameters parameters_;
};

} // namespace passive_flight