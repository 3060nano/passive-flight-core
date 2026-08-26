#pragma once

#include <vector>

namespace passive_flight {

/**
 * Одна точка табличной зависимости коэффициента
 * лобового сопротивления от числа Маха.
 */
struct DragCoefficientPoint {
    double mach{0.0};
    double zeroLiftDragCoefficient{0.0};
};

/**
 * Геометрические параметры отдельной несущей поверхности:
 * крыла или стабилизатора.
 */
struct LiftingSurfaceAerodynamics {
    // Площадь поверхности, м^2.
    double areaM2{0.0};

    // Удлинение поверхности.
    double aspectRatio{0.0};

    // Угол стреловидности по линии половин хорд, рад.
    double halfChordSweepRad{0.0};

    // Коэффициент аэродинамической эффективности.
    double efficiencyFactor{0.0};

    // Угол установки поверхности, рад.
    double installationAngleRad{0.0};

    // Продольная координата аэродинамического фокуса, м.
    double aerodynamicCenterXM{0.0};
};

/**
 * Геометрические и настроечные параметры,
 * необходимые непосредственно аэродинамической модели.
 *
 * На следующем этапе будет добавлен преобразователь
 * ObjectModel -> AerodynamicGeometry.
 */
struct AerodynamicGeometry {
    // Характерная площадь объекта, м^2.
    double referenceAreaM2{0.0};

    // Характерная хорда объекта, м.
    double referenceChordM{0.0};

    // Диаметр корпуса, м.
    double bodyDiameterM{0.0};

    // Продольная координата центра масс, м.
    double centerOfMassXM{0.0};

    // Продольная координата аэродинамического центра корпуса, м.
    double bodyAerodynamicCenterXM{0.0};

    /*
     * Поправка, учитывающая влияние формы носовой части
     * на производную нормальной силы корпуса.
     *
     * Для базового оживального носа принимается 1.
     */
    double noseNormalForceFactor{1.0};

    // Крыло.
    LiftingSurfaceAerodynamics wing;

    // Стабилизатор.
    LiftingSurfaceAerodynamics tail;

    /*
     * Производная скоса потока:
     *
     * d epsilon / d alpha.
     */
    double downwashGradient{0.0};

    /*
     * Приближённое отношение производной
     * m_z по скорости изменения угла атаки
     * к производной демпфирующего момента.
     */
    double alphaDotDampingRatio{0.0};
};

/**
 * Входные параметры аэродинамического расчёта.
 */
struct AerodynamicInput {
    // Число Маха.
    double mach{0.0};

    // Угол атаки, рад.
    double angleOfAttackRad{0.0};

    // Угловая скорость тангажа omega_z, рад/с.
    double pitchRateRadS{0.0};

    // Скорость изменения угла атаки, рад/с.
    double angleOfAttackRateRadS{0.0};

    // Модуль воздушной скорости, м/с.
    double speedMps{0.0};
};

/**
 * Результат расчёта аэродинамических коэффициентов.
 */
struct AerodynamicCoefficients {
    // Коэффициент сопротивления при нулевой подъёмной силе.
    double cx0{0.0};

    // Индуктивная составляющая сопротивления.
    double cxInduced{0.0};

    // Полный коэффициент сопротивления.
    double cx{0.0};

    // Составляющая коэффициента нормальной силы от корпуса.
    double cyBody{0.0};

    // Составляющая коэффициента нормальной силы от крыла.
    double cyWing{0.0};

    // Составляющая коэффициента нормальной силы от стабилизатора.
    double cyTail{0.0};

    // Полный коэффициент нормальной силы.
    double cy{0.0};

    // Статическая составляющая коэффициента момента.
    double mzStatic{0.0};

    // Составляющая момента от угловой скорости.
    double mzPitchDamping{0.0};

    // Составляющая момента от скорости изменения угла атаки.
    double mzAlphaDot{0.0};

    // Полный коэффициент продольного момента.
    double mz{0.0};

    // Производная полного cy по углу атаки, 1/рад.
    double cyAlphaPerRad{0.0};

    // Производная статического mz по углу атаки, 1/рад.
    double mzAlphaPerRad{0.0};

    /*
     * Производная коэффициента момента по безразмерной
     * угловой скорости:
     *
     * omega_z_bar = omega_z * b_a / (2 * V).
     */
    double mzPitchRateDerivative{0.0};

    /*
     * Производная коэффициента момента по безразмерной
     * скорости изменения угла атаки:
     *
     * alpha_dot_bar = alpha_dot * b_a / (2 * V).
     */
    double mzAlphaDotDerivative{0.0};
};

/**
 * Создаёт аэродинамическую геометрию базового
 * абстрактного объекта массой 500 кг.
 */
[[nodiscard]]
AerodynamicGeometry makeAbstract500AerodynamicGeometry();

/**
 * Возвращает таблицу коэффициента сопротивления,
 * извлечённую из переданной модели SimInTech.
 */
[[nodiscard]]
std::vector<DragCoefficientPoint>
makeAbstract500ZeroLiftDragTable();

/**
 * Расчётная аэродинамическая модель первого приближения.
 */
class PreliminaryAerodynamicModel {
public:
    PreliminaryAerodynamicModel();

    PreliminaryAerodynamicModel(
        const AerodynamicGeometry& geometry,
        std::vector<DragCoefficientPoint> zeroLiftDragTable
    );

    /**
     * Вычисляет аэродинамические коэффициенты.
     */
    [[nodiscard]]
    AerodynamicCoefficients evaluate(
        const AerodynamicInput& input
    ) const;

    /**
     * Возвращает используемую геометрию.
     */
    [[nodiscard]]
    const AerodynamicGeometry& geometry() const noexcept;

    /**
     * Возвращает таблицу cx0(M).
     */
    [[nodiscard]]
    const std::vector<DragCoefficientPoint>&
    zeroLiftDragTable() const noexcept;

private:
    AerodynamicGeometry geometry_;
    std::vector<DragCoefficientPoint> zeroLiftDragTable_;
};

} // namespace passive_flight