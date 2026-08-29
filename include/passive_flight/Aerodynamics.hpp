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
 * Одна точка табличной зависимости производной
 * коэффициента продольного момента по безразмерной
 * скорости изменения угла атаки.
 *
 * Используемое соглашение:
 *
 *     alphaDotBar =
 *         alphaDot * referenceChord / (2 * V)
 *
 * и
 *
 *     mzAlphaDot =
 *         mzAlphaDotDerivative * alphaDotBar.
 *
 * Поэтому mzAlphaDotDerivative является
 * безразмерной производной:
 *
 *     d(mz) / d(alphaDotBar).
 *
 * Если исходная таблица в будущем будет использовать
 * другую нормировку alphaDot, данные необходимо
 * предварительно привести к этому соглашению.
 */
struct PitchMomentAlphaDotDerivativePoint {
    double mach{0.0};
    double mzAlphaDotDerivative{0.0};
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
};

/**
 * Входные параметры аэродинамического расчёта.
 */
struct AerodynamicInput {
    // Число Маха.
    double mach{0.0};

    // Угол атаки объекта, рад.
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
     * Вклад корпуса в производную коэффициента момента
     * по безразмерной угловой скорости:
     *
     * omegaZBar =
     *     omega_z * referenceChord / (2 * V).
     *
     * На текущем этапе корпус представляется
     * сосредоточенной нормальной силой в его
     * аэродинамическом центре.
     */
    double mzPitchRateBodyDerivative{0.0};

    /*
     * Вклад крыла в производную коэффициента момента
     * по безразмерной угловой скорости.
     *
     * Пока равен нулю, поскольку для крыла требуется
     * отдельная методика распределённой нагрузки.
     * Нулевое значение здесь означает "не смоделировано",
     * а не физическое отсутствие демпфирования крыла.
     */
    double mzPitchRateWingDerivative{0.0};

    /*
     * Вклад стабилизатора в производную коэффициента момента
     * по безразмерной угловой скорости.
     *
     * Используется приближение сосредоточенной
     * нормальной силы на длинном плече относительно центра масс.
     */
    double mzPitchRateTailDerivative{0.0};

    /*
     * Полная производная коэффициента момента
     * по безразмерной угловой скорости:
     *
     * mzPitchRateDerivative =
     *     mzPitchRateBodyDerivative +
     *     mzPitchRateWingDerivative +
     *     mzPitchRateTailDerivative.
     */
    double mzPitchRateDerivative{0.0};

    /*
     * Производная коэффициента момента по безразмерной
     * скорости изменения угла атаки:
     *
     * alphaDotBar =
     *     alphaDot * referenceChord / (2 * V).
     *
     * В текущей модели определяется из таблицы
     * mzAlphaDotDerivative(M).
     *
     * При отсутствии таблицы равна нулю.
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
 * Возвращает таблицу коэффициента сопротивления
 * базового объекта.
 */
[[nodiscard]]
std::vector<DragCoefficientPoint>
makeAbstract500ZeroLiftDragTable();

/**
 * Возвращает таблицу
 *
 *     mzAlphaDotDerivative(M).
 *
 * На текущем этапе надёжные исходные данные
 * отсутствуют, поэтому таблица пустая.
 *
 * Пустая таблица означает:
 *
 *     mzAlphaDotDerivative = 0.
 *
 * Когда зависимость будет оцифрована,
 * достаточно заполнить эту функцию табличными точками.
 */
[[nodiscard]]
std::vector<PitchMomentAlphaDotDerivativePoint>
makeAbstract500PitchMomentAlphaDotDerivativeTable();

/**
 * Расчётная аэродинамическая модель первого приближения.
 */
class PreliminaryAerodynamicModel {
public:
    PreliminaryAerodynamicModel();

    /**
     * Конструктор без таблицы mzAlphaDotDerivative(M).
     *
     * В этом случае вклад alphaDot отключён.
     */
    PreliminaryAerodynamicModel(
        const AerodynamicGeometry& geometry,
        std::vector<DragCoefficientPoint> zeroLiftDragTable
    );

    /**
     * Полный конструктор аэродинамической модели.
     *
     * alphaDotDerivativeTable может быть пустой.
     * Пустая таблица означает:
     *
     *     mzAlphaDotDerivative = 0.
     */
    PreliminaryAerodynamicModel(
        const AerodynamicGeometry& geometry,
        std::vector<DragCoefficientPoint> zeroLiftDragTable,
        std::vector<PitchMomentAlphaDotDerivativePoint>
            alphaDotDerivativeTable
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

    /**
     * Возвращает таблицу mzAlphaDotDerivative(M).
     */
    [[nodiscard]]
    const std::vector<PitchMomentAlphaDotDerivativePoint>&
    alphaDotDerivativeTable() const noexcept;

private:
    AerodynamicGeometry geometry_;

    std::vector<DragCoefficientPoint>
        zeroLiftDragTable_;

    std::vector<PitchMomentAlphaDotDerivativePoint>
        alphaDotDerivativeTable_;
};

} // namespace passive_flight
