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
 * среднего угла скоса потока по углу атаки:
 *
 *     epsilonAlpha = d epsilon / d alpha.
 *
 * Значение безразмерное.
 */
struct DownwashGradientPoint {
    double mach{0.0};
    double gradient{0.0};
};

/**
 * Одна точка табличной зависимости производной
 * коэффициента продольного момента по безразмерной
 * скорости изменения угла атаки.
 *
 * Используемое соглашение предварительной модели:
 *
 *     alphaDotBar =
 *         alphaDot * referenceChord / (2 * V)
 *
 * и
 *
 *     mzAlphaDot =
 *         mzAlphaDotDerivative * alphaDotBar.
 *
 * Если источник использует другую нормировку alphaDot,
 * данные должны быть приведены к соглашению конкретной
 * аэродинамической модели.
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
 * необходимые непосредственно предварительной
 * аэродинамической модели крылатого объекта.
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
     * Резервное значение производной скоса потока:
     *
     *     d epsilon / d alpha.
     *
     * Используется только если табличная зависимость
     * downwashGradient(M) не передана в модель.
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
 *
 * Конкретная аэродинамическая модель обязана явно
 * документировать нормировку безразмерной угловой скорости,
 * если она использует производную момента по omega_z.
 *
 * Для будущей табличной модели ФАБ-1500Т используется
 * соглашение Лебедева--Чернобровкина и Постникова--Чуйко:
 *
 *     omegaZBar = omega_z * L_ref / V.
 *
 * Текущая PreliminaryAerodynamicModel пока сохраняет
 * своё прежнее соглашение с referenceChord / (2 * V),
 * чтобы на этом этапе не менять её численные результаты.
 */
struct AerodynamicCoefficients {
    // Коэффициент сопротивления при нулевой подъёмной силе.
    double cx0{0.0};

    /*
     * Добавочная составляющая сопротивления, зависящая
     * от угла атаки. Для предварительной модели это
     * индуктивное сопротивление; для табличной бомбы
     * здесь будет Cx^(alpha^2) * alpha^2.
     */
    double cxInduced{0.0};

    // Полный коэффициент сопротивления.
    double cx{0.0};

    /*
     * Использованная в текущей точке производная
     * среднего угла скоса потока по углу атаки.
     */
    double downwashGradient{0.0};

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
     * Диагностические составляющие производной момента
     * по безразмерной угловой скорости.
     *
     * Нормировка безразмерной угловой скорости определяется
     * конкретной аэродинамической моделью и её источником.
     */
    double mzPitchRateBodyDerivative{0.0};
    double mzPitchRateWingDerivative{0.0};
    double mzPitchRateTailDerivative{0.0};
    double mzPitchRateDerivative{0.0};

    /*
     * Производная коэффициента момента по безразмерной
     * скорости изменения угла атаки.
     */
    double mzAlphaDotDerivative{0.0};
};

/**
 * Общий контракт аэродинамической модели.
 *
 * Динамика не должна знать, каким способом получены
 * коэффициенты:
 *
 * - из готовых табличных аэродинамических характеристик;
 * - из геометрии;
 * - из пользовательской геометрии.
 */
class AerodynamicModel {
public:
    virtual ~AerodynamicModel() = default;

    [[nodiscard]]
    virtual AerodynamicCoefficients evaluate(
        const AerodynamicInput& input
    ) const = 0;
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
 * Возвращает дозвуковую таблицу
 *
 *     epsilonAlpha(M) = d epsilon / d alpha
 *
 * для базового объекта.
 */
[[nodiscard]]
std::vector<DownwashGradientPoint>
makeAbstract500DownwashGradientTable();

/**
 * Возвращает таблицу
 *
 *     mzAlphaDotDerivative(M).
 *
 * На текущем этапе надёжные исходные данные отсутствуют,
 * поэтому таблица пустая.
 */
[[nodiscard]]
std::vector<PitchMomentAlphaDotDerivativePoint>
makeAbstract500PitchMomentAlphaDotDerivativeTable();

/**
 * Расчётная аэродинамическая модель первого приближения.
 *
 * На этом этапе её физика не изменяется. Класс только
 * начинает реализовывать общий контракт AerodynamicModel.
 */
class PreliminaryAerodynamicModel final
    : public AerodynamicModel {
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
     * Конструктор с таблицей mzAlphaDotDerivative(M),
     * но без таблицы скоса потока.
     */
    PreliminaryAerodynamicModel(
        const AerodynamicGeometry& geometry,
        std::vector<DragCoefficientPoint> zeroLiftDragTable,
        std::vector<PitchMomentAlphaDotDerivativePoint>
            alphaDotDerivativeTable
    );

    /**
     * Полный конструктор аэродинамической модели.
     */
    PreliminaryAerodynamicModel(
        const AerodynamicGeometry& geometry,
        std::vector<DragCoefficientPoint> zeroLiftDragTable,
        std::vector<DownwashGradientPoint> downwashGradientTable,
        std::vector<PitchMomentAlphaDotDerivativePoint>
            alphaDotDerivativeTable
    );

    /**
     * Вычисляет аэродинамические коэффициенты.
     */
    [[nodiscard]]
    AerodynamicCoefficients evaluate(
        const AerodynamicInput& input
    ) const override;

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
     * Возвращает таблицу epsilonAlpha(M).
     */
    [[nodiscard]]
    const std::vector<DownwashGradientPoint>&
    downwashGradientTable() const noexcept;

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

    std::vector<DownwashGradientPoint>
        downwashGradientTable_;

    std::vector<PitchMomentAlphaDotDerivativePoint>
        alphaDotDerivativeTable_;
};

} // namespace passive_flight
