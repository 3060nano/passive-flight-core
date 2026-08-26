
#ifndef core_intf_def

#define core_intf_def


  /* Идентифкаторы типов данных */
#define    vt_double         0      /* Вещественное */
#define    vt_bool           1      /* Булевское */
#define    vt_int            2      /* Целое */
#define    vt_pointer        3      /* Это тип - целое число зависящее от разрядности системы (для локальных переменных-указателей объектов) */

  /* Вспомогательные
  Вещественные */
#define    vt_float          4      /* 4 байтовое с ПЗ */
#define    vt_fix32          5      /* 4 байтовое fixpoint(16.16) */
  /* Целые */
#define    vt_int64          6      /* 64 битное целое */
#define    vt_int16          7      /* 16 битное целое */

#define    vt_complex        8      /* Комплексное из 2 64 битных вещественных */
#define    vt_complex_32     9      /* Комплексное из 2 32 битных вещественных */
#define    vt_complex_fix16 10      /* Комплексное из 2 32 битных вещественных с фиксированной точкой */

  /* Дополнительный тип - 8 битное значащее целое */
#define    vt_int8          11

  /* Беззначные целые типы данных */
#define    vt_uint8         12
#define    vt_uint16        13
#define    vt_uint32        14
#define    vt_uint64        15

  /* Направление переменной */
#define    dir_input         0
#define    dir_out           1
#define    dir_inout         2

  /* Флаги вызова run-функции блока */
#define    f_InitState       1      /* Запись начальных состояний */
#define    f_UpdateOuts      2      /* Обновить выходы на предварительном шаге */
#define    f_GoodStep        3      /* Обновить выходы на "хорошем" шаге */
#define    f_GetDeri         4      /* Вычислить значения правых частей дифференциальных уравнений */
#define    f_GetAlgFun       5      /* Вычислить значения правых частей алгебраических уравнений */
#define    f_SetState        6      /* Вычислить значения дискретных переменных состояния (после шага интегрирования) */
#define    f_UpdateProps     7      /* Обновить список параметров (с учётом флага изменяемости) */
#define    f_GetJacobyState  8      /* Вычислить значения дискретных переменных состояния при расчете Якобиана */
#define    f_UpdateJacoby    9      /* Обновить Якобиан блока */
#define    f_RestoreOuts    10      /* Обновить выходы после рестарта (только если очень надо, т.к. выходы всё равно будут запоминаться) */
#define    f_SetAlgOut      11      /* Выставить выходы блока, содержащих алгебраические переменные */
#define    f_InitAlgState   12      /* Выставить начальное приближение для алгебраические переменных */
#define    f_Stop           13      /* Вызывается при остановке расчёта (конец моделирования) */
#define    f_InitObjects    14      /* Инициализация объектов, массивов и т.д. (сразу после сортировки) (начало моделирования) */
#define    f_EndTimeTask    15      /* Вызывается по окончании выполнения задачи (для проверки оптимизации и т.п.) */
  /* Это надо для частотного анализа и всяких расчётов Якобиана  */
#define    f_GetDisState    16      /* Получить значения дискретных переменных состояния блока */
#define    f_SetDisState    17      /* Присвоить возмущение для дискретной переменной */
#define    f_GetDelayTime   18      /* Получить время задержки блока */
#define    f_GetInitDisState  19    /* Получить значения дискретных переменных состояния, соответствующих выходам блока в начальный момент времени */
    /* Дополнительные флаги */
#define    f_BeforeInitState  20 /* Вызов непосредтсвенно перед f_InitState (инициализировать внутренние переменные) */
#define    f_BeforeStop     21 /* Вызов перед остановкой задачи при нажатии на стоп пользователем (чтобы прекратить ожидания) */

  /* Флаги вызовов информационной функции блока */ 
#define    i_GetBlockType            1   /* Получить тип блока (источник, динамический и т.д */
#define    i_GetDifCount             2   /* Получить число дифференциальных переменных */
#define    i_GetAlgCount             4   /* Получить число алгебраических переменных */
#define    i_GetCount                5   /* Получить размерности входов\выходов */
#define    i_GetInit                 6   /* Получить флаг зависимости выходов от входов */
#define    i_GetPropErr              7   /* Проверка правильности задания параметров блока (перед сортировкой) */
#define    i_HaveSpetialEditor       8   /* Флаг - run-объект имеет специализированный редактор блока */
#define    i_GetPostSection          9   /* Флаг - блоку нужна пост-секция для выполнения run-функции */
#define    i_ReconnectPorts          10  /* Флаг - действия выполняемые до сортировки для переназначения портов блока */
#define    i_GetSyncPostSection      11  /* Флаг - блоку нужна секция, выполняемая последовательно при синхронном получении данных */
#define    i_GetDisCount             12  /* Получить к-во переменных для дискретных блоков */
#define    i_BeforeEditBlock         13  /* Вызов перед открытием штатного редактора блока */
#define    i_CheckCanClose           14  /* Вызов для проверки нужно ли закрывать проект при изменении блока */
#define    i_GetCustomInfoStageCount 15  /* Запрос к-ва дополнительных кастомных стадий инициализации блока */
#define    i_CustomInfoFuncStage     16  /* Вызов дополнительной стадии информационной функции блока */
#define    i_GetConturSection        17  /* Флаг наличие доп секции расчёта вспомогательного контура (подсистемы) */

  /* Типы блоков (для сортировки, частотного анализа, синтеза) */
#define    t_none              0    /* Сервисный блок, в расчете не участвует */
#define    t_src               1    /* Блок-источник сигнала */
#define    t_fun               2    /* Функциональный блок */
#define    t_dst               3    /* Блок-приемник информации */
#define    t_del               4    /* Блоки запаздывания */
#define    t_ext               5    /* Блоки-экстраполяторы */
#define    t_der               6    /* Блоки-производные */
#define    t_imp               7    /* Блоки-импортеры данных */
#define    t_exp               8    /* Блоки-экспортеры данных */

  /* Возможные результаты функций */
#define    r_Success        0       /* Нет ошибки */
#define    r_Fail           1       /* Возникла ошибка */

 // Определение системного строкового типа для интерфейсных функций
#define syschartype char

  /* Типы вызовов функций структуры ядра автоматики */
#if __GNUC__
#define core_call_type 
#else
#define core_call_type _cdecl
#endif

  /* Макрос - экспорт функции в DLL */
#ifndef linux
#define EXPORTED_FUNC int _stdcall
#else
#define EXPORTED_FUNC int
#endif

  /* Подключаем декларацию типов для решателя разреженных СЛАУ */
#include "sit_sparce_interface.h"

  /* Комплексное двойной точности */
typedef struct { 
 double re;
 double im;
} complex_64;

  /* Комплексное одинарной точности */
typedef struct { 
 float re;
 float im;
} complex_32;

  /* Запись переменной */
typedef struct {
  char* name;
  char* description;
  void* default_ptr;
  int   data_type;
  int   dim[3];
  int   index;
  int   direction;  
  int   data_size;
  int   reserved_int;
} ext_var_info_record;

typedef int (core_call_type *t_glob_obj_destructor_proc)(void* aGlobalObjPtr);


  /* Структура для возврата вспомогательной информации о задаче от INFO_FUNC */
typedef struct {
	
    //    Версия API - 64 битное число
    uint64_t            TASK_API_VER_NUM;

    //    Сначала - double (8 байт)
    double              AbsErr;             // Абсолютная ошибка
    double              RelErr;             // Относительная ошибка
    double              Hmin;               // Минимальный шаг интегрирования
    double              Hmax;               // Максимальный шаг интегрирования
    double              StartStep;          // Начальный шаг интегрирования, если 0 вычисляется автоматически
    double             	EndTime;            // Конечное время расчёта
    uintptr_t           MaxLoopIt;          // максимальное количество итераций для НАУ
    char*               TaskDAELibraryName; // Имя метода интегрирования если надо
	
} TTaskInfoStruct;

typedef  TTaskInfoStruct* PTaskInfoStruct;


  /* Описание структуры для доступа к специальным переменным и методам решателя */
typedef struct {
    /* SimInTech core structure */	
	
    //    Версия API - 64 битное число
    uint64_t  DAE_API_VER_NUM;

    //    Сначала - double (8 байт)
    double    AbsErr;                                 // Абсолютная ошибка
    double    RelErr;                                 // Относительная ошибка
    double    Hmin;                                   // Минимальный шаг интегрирования
    double    Hmax;                                   // Максимальный шаг интегрирования
    double    newstep;                                // Новый прогнозный шаг интегрирования
    double    StartStep;                              // Начальный шаг интегрирования, если 0 вычисляется автоматически
    double    Step;                                   // Текущий (хороший) шаг интегрирования
    double    mtime;                                  // Текущее время расчёта шага интегрирования   (DoStep)
    double    FStartModelTime;                        // Переменные для замера реального времени
    double    SummRTDelay;                            // Суммарная рассчитанная задержка реального времени, сек
    double    WorkStepDelay;                          // Задержка рабочего шага, сек
    double    LastSyncStepTimeDelta;                  // Сдвиг реального времени последнего шага интегрирования, сек

    //Вспомогательные настроки для решения НАУ
    double    realzero;                               // ноль для НАУ
    double    kerr_loop;                              // максимальная допустимая ошибка при решении НАУ
    double    teta1_loop;                             // коэффициент оценки сходимости итераций при решении НАУ
    double    teta_min;                               // минимальное ограничение для teta при решении НАУ
    double    teta_0999;                              //
    double    teta_max;                               // Максимальное teta
    double    d_min_loop;
    double    d_jacoby;                               // относительное приращение переменных при расчёте Якобиана
    double    min_delta_step;                         // минимальное относительное ограничение изменения шага
    double    max_delta_step;                         // максимальное ограничение для деления шага на 2
    double    fmax;                                   // Верхнее ограничение для значений алгебраических функций
		
	//Вспомогательные настройки для дискретных блоков и периодических источников	
	double    time_rel_error;                         // Относительная ошибка сравнения таймеров для блоков с дискретным временем		
		
    //Счетчики - 64 битные числа
    uint64_t  NAccuracyErrors;                        // Текущее к-во ошибок несходимости
    uint64_t  JacobyCalcBlockCount;                   // К-во вызовов блоков при вычислении матрицы Якоби за цикл
    uint64_t  JacobyGetDifCount;                      // К-во вызовов диф блоков при вычислении матрицы Якоби за цикл
    uint64_t  JacobuGetAlgCount;                      // К-во вызовов алг блоков при вычислении матрицы Якоби за цикл
    uint64_t  MaxJacobyCalcBlockCount;                // Максимальное к-во вызовов блоков при вычислении матрицы Якоби за цикл
    uint64_t  MaxJacobyGetDifCount;                   // Максимальное к-во вызовов диф блоков при вычислении матрицы Якоби за цикл
    uint64_t  MaxJacobuGetAlgCount;                   // Максимальное к-во вызовов алг блоков при вычислении матрицы Якоби за цикл
    uint64_t  Nfun;                                   // Число вычислений функции FUN
    uint64_t  NLI;                                    // Число итераций петель общее
    uint64_t  NLforCycle;                             // Число петель на цикл
    uint64_t  NLU;                                    // Число LU-разложений
    uint64_t  Ngood;                                  // Число хороших шагов
    uint64_t  Nbad;                                   // Число плохих шагов
    uint64_t  NJ;                                     // Число вычислений Якобиана
    uint64_t  NDIRKInternalIterCount;                 // Количество итерационных уточнений решения для DIRK методов
    uint64_t  NDIRKMaxInternalIterCount;              // Максимальное нужное к-во повторов решения

    uintptr_t Ndif;                                   // Текущее число дифференциальных уравнений
    uintptr_t Nalg;                                   // Текущее число алгебраических уравнений
    uintptr_t Nall;                                   // Суммарное число уравнений Ndif+Nalg
    uintptr_t MaxLoopIt;                              // Максимальное число итераций при решении системы НАУ
    intptr_t  NLocalIter;                             // Количество текущих локальных итераций (для распределённого анализа точности)
    uintptr_t MaxNumbersOfAccuracyErors;              // Максимальное к-во ошибок несходимости
    uintptr_t jmax;                                   // максимальное число делений шага для НАУ

    // ---   Резервные указатели  для дальнейших нужд расширения интерфейса решателя ---
	PTaskInfoStruct TaskInfo;                         // Дополнительные данные задачи - настройки исполнения
    intptr_t  FunAction;                              // Текущее значение флага вызова функции решателя
    void*     ReserverPtr3;
    void*     ReserverPtr4;
	
    // Контекст владельца системы
    void*     TaskContext;
    // Контекст обработчика системы уравнений
    void*     ODEContext;
			
    /* Эти методы используются для того чтобы зарегситрировать и удалить специализированные объекты схемы (например распределённый решатель лин. уравнений)
       Найти глобальный объект по имени    */
    void*     (core_call_type *FindGlobalObject)(void* ALayerContext,char* aGlobObjectName);  
    /* Зарегистрировать новый глобальный объект */
    void      (core_call_type *RegisterGlobalObject)(void* ALayerContext,char* aGlobObjectName,void* aNewObject,t_glob_obj_destructor_proc destructor_proc_ptr);
    /* Регистрация нужной библиотеки и получение от неё функций */
    uintptr_t (core_call_type *DoLoadNeedPlugin)(char* aPluginName);  	
    //Получение указателя интерфейса объекта DLL (загрузка DLL <имя dll> + вызов функции get_object_interface(<имя объекта внутри>)
    //Формат имени:  <имя dll>@<имя объекта внутри>
    //Возвращает указатель на структуру интерфейса объекта
    void*     (core_call_type *LoadInterfacedObject)(char* aSrcLibName);
    //Добавить зависимость сортировки между двумя Run-объектами. aFirstRunObject рассчитывается до aSecondRunObject
    void      (core_call_type *AddDependByRunObjects)(void* aODEIntf, void* aFirstRunObject, void* aSecondRunObject, char aStrictSort);
    /* Ссылка на глобальные списки именованных зависимостей и соотв-й хеш-лист */
    void*     GlobalDepList;
    void*     GlobalDepHash;  
    /* Текущий список зависимостей для записи дополнительных данных */
    void*     CurentDepList;  
    /* Ссылка на список замены портов для оптимизации передачи данных */
    void*     GlobalWherewithList;
    void*     GlobalWherewithHash;  
    /* Функция поиска указателя на данные по имени объекта, возврат = тип данных, и указатель */
    unsigned char  (core_call_type *GetDataPtr)(void* TaskContext, char* aSignalName, void** DataPtr, int* dimension); 
    /* Проверка необходимости принудительной остановки  */
    char      (core_call_type *StopCheck)(void* TaskContext);   
    /* Имя библиотеки решения разреженной СЛАУ  */  
    char*     DefaultLAESolverLibraryName;  
	
    //------------------------------------------------------------------------------
    //Подпрограммы требуемые из ядра для методов интегрирования
    //Процедура вычисления
    void      (core_call_type *Fun)(void* aODEIntf, double at, double h, double* x, double* fx, int Action, intptr_t* ner);
    //Процедура вычисления производных
    void      (core_call_type *GetDifFun)(void* aODEIntf, double at, double, double* x, double* fx, int Action, intptr_t* ner);
    //Процедура вычисления правых частей для алгебраических переменных
    void      (core_call_type *GetAlgFun)(void* aODEIntf, double at, double h, double* x, double* fx, int Action, intptr_t* ner);
    //Процедура расчета алгебраических петель
    void      (core_call_type *AlgLoop)(void* aODEIntf, double at, double h, double* x, double* fx, intptr_t* ner);
    //Расчет возмущения
    void      (core_call_type *Perturbation)(void* aODEIntf, double at, double h, double dx, double* x, double* fx, uintptr_t iX, int Action, char fFull,
                              intptr_t* ner, uint64_t* chaincount, uint64_t* diffuncount, uint64_t* algfuncount, char* AllLinear);
    //Если True - то принудительный останов расчёта
    char      (core_call_type *GetStopRun)(void* aODEIntf);
    //Вызывается для задержки вычислений в режиме реального времени
    void      (core_call_type *RealTimeDelay)(void* aODEIntf, double dt, double t);
    //Вызов LU-разложения
    void      (core_call_type *LUFact)(void* aODEIntf, double h, uintptr_t N, uintptr_t NM, intptr_t* ner);
    //Вызов решения СЛАУ
    void      (core_call_type *SolveLU)(void* aODEIntf, uintptr_t N, double* b, double* x, intptr_t* ner);
    //Вызов расчёта матрицы Якоби
    void      (core_call_type *Jacoby)(void*  aODEIntf, double at, double h, double* x, double* f, double* f1,char fs, int Action, intptr_t* ner);
    //Процедура системного котроля шага интегрирования
    void      (core_call_type *StepControlSection)(void* aODEIntf, double* h, double* t, double* tfin);
    //Округление шага
    double    (core_call_type *RoundStep)(void* aODEIntf, double h, char need_gt_zero);
    //Итерация алгебраических петель (для начальной инициализации неявных методов или явных методов при наличии алг. уравнений)
    void      (core_call_type *LoopIter)(void* aODEIntf, double t, double h, double* x, double* y, int Action, intptr_t* ner);
    //Расчёт строки Якобиана полный
    void      (core_call_type *FillJacobyColumn)(void* aODEIntf,int off,int N, int col, double dx, double* f, double* f1,
                                 double** aa, int** iCol, intptr_t* cCol, intptr_t* sCol, char s);
    //Расчёт строки Якобиана с учётом блоков-терминаторов
    void      (core_call_type *FillJacobyColumnWithTermList)(void* aODEIntf, int col, int mainoff, double dx, double* f, double* f1,
                                 double** aa, int** iCol, intptr_t* cCol, intptr_t* sCol);
    //Вычисление матрицы Якоби для алгебраических переменных только
    void      (core_call_type *GetLoopJacoby)(void* aODEIntf, double t, double h, double* x, double* y, int Action, intptr_t* ner);
    //Прочитать необходимые данные для метода интегрирования перед шагом
    void      (core_call_type *ReadCalcData)(void* aODEIntf, double t, double h);
    //Записать рассчитанные данные куда-то
    void      (core_call_type *WriteCalcData)(void* aODEIntf, double t, double h);
	
    //Получение данных перед шагом синхронизации
    char      (core_call_type *OnBeforeStep)(void* aODEIntf);
    //Отдача данных после шага синхронизации
    char      (core_call_type *OnAfterStep)(void* aODEIntf);

    //Выдача диагностики
    void      (core_call_type *ErrorEvent)(void* aODEIntf, syschartype* Msg, char MsgType, void* Obj, intptr_t TextPos);

    // ---  Управляющие функции --------------------
    char      (core_call_type *Reset)(void* aODEIntf);
    char      (core_call_type *Start)(void* aODEIntf, double* t);
    char      (core_call_type *DoStep)(void* aODEIntf, double* tfin, char _Precition, char _OneStep, intptr_t* ner);
    char      (core_call_type *RestoreOuts)(void* aODEIntf, double* t);

    // ---  Присвоить имя метода интегрирования ----
    void      (core_call_type *SetDAEName)(void* aODEIntf, syschartype* aDAEName);
    void      (core_call_type *SetLAEName)(void* aODEIntf, syschartype* aLAEName);

    // ---- Функция выделения памяти для добавления элементов в якобиан ----
    void      (core_call_type *ODEReallocMem)(void** P, int Size);
	
    // ---  Матрица Якоби метода интегрирования и вспомогательные массивы ----                                                  
                                                      // Память под них выделяется ядром программы !
    double**   AA;                                    // Матрица коэффициентов системы ЛАУ (разреженная)
    int**      iCol;                                  // Матрица индексов ННЭ в AA
    intptr_t*  cCol;                                  // Массив, i-ый элемент которого - число ННЭ в столбце матрицы коэффициентов AA         
    intptr_t*  sCol;                                  // Массив, i-ый элемент которого - фактический размер памяти в столбце матрицы коэффициентов AA
    double**   Mbr;                                   // Матрица для метода Бройдена

    //  --- Интерфейс и ссылка на объект решателя разреженных матриц ----                                                  
    p_sparce_solver_interface  SP_SOLVER_INTF;
    TSolverInterfacedObject*   SP_SOLVER_HDL;
    TSP_SolverSettingsRec*     SP_SOLVER_DFLTS;
	
    //Диагностический возврат от метода интегрирования для внешнего наблюдения
    //эти массивы выделяются методом интегрирования, могут быть инициализированы не все !
    double*    xn;                                    // X(n) - на хорошем шаге Xn=Work[0]
    double*    fn;                                    // X'(n) - на хорошем шаге Fn=Work[-1]
    double*    errs;                                  // массив для ошибок Errs=Work[-2]
    intptr_t*  NconvArr;                              // Число итерация до сходимости для каждой из переменных
    intptr_t*  NconvMax;
	
      /* Контрольная сумма структуры текущей схемы (для проверки совпадения со сгенерированным кодом) */
    unsigned int  ShemeHash;
  
    //    Байты (1 байт)
    //Параметры интегрирования - передаются извне
    char       LoopMet;                               // Метод решения системы НАУ

    //    Двоичные флаги (1 байт)
    char       IsRemoteStubMode;                      // Флаг - используется пустой метод интегрирования
    char       IsImlicit;                             // Идентификатор неявного метода интегрирования - присваивается методом
    char       IsNeedLoop;                            // Флаг возможна ли в принципе итерация петель
    char       IsLoop;                                // Флаг итерации петли в данный момент
    char       fNeedIter;                             // Флаг необходимости повторного шага

    //Параметры, задающие тип синхронизации расчета
    char       fPrecition;                            // Флаг точной синхронизации
    char       fOneStep;                              // Флаг выполнения одного шага интегрирования
    char       fFirstStep;                            // Флаг первого шага расчёта
    char       accuracy;                              // Флаг соблюдения точности
    char       reset_first_step_flag;                 // Флаг сбороса первого шага
    char       conv_alg;                              // Флаг сходимости алгебраических переменных

    //Переменные управления решателем
    char       fsetstep;                              // Флаг - установить новый шаг интегрирования  
    char       UseSignalExtendedSort;                 // Флаг учёта расширенных зависимостей сортировки для блоков чтения и записи сигналов  
    char       ErrorOnSignalLoop;                     // Флаг - ошибка при наличии алгебраических петель образованных блоками чтения и записи сигналов 
    char       UseConditionsExtendedSort;             // Учёт зависимостей сортировки блоков типа "Условие выполенния субмодели"    
    char       UseSignalsPortReconnection;            // Использовать замену портов при оптимизации связок "чтение сигналов"-"запись сигналов"  
    char       fConstantCheckMode;                    // Флаг режима проверки констант 
    char       fCodeGenMode;                          // Флаг режима генерации кода - для блокировки работы некоторых блоков   
    char       WriteSignalsOnSyncStep;                // Флаг - блоки записи выполнять только на шаге синхронизации
    char       common_translation_flag;               // Флаг - транслировать сигналы из\в внешней исполнительной системы 
    char       fStartAgain;                           // Флаг - необходимо перезапустить расчёт с нулевой точки заново  
    char       fSaveModelState;                       // Флаг - необходимо запомнить стартовое состояние модели  
    char       fWriteSignalsOnInitState;              // Флаг - нужно перезаписывать значения сигналов на выходах при вызове f_InitState  
    char       UseAlgVarsStepControl;                 // Флаг - использовать контроль точности для алгебраических переменных для DIRK и явных методов   
    char       fNeedUpdateOutsBeforeGoodStep;         // Флаг - делать дополнительные пробные шаги для внутренних итераций модели. При наличии блоков анализа точности и повтора расчёта надо выставить этот флаг, чтобы перед выполнением f_GoodStep делался f_UpdateOuts с тем же шагом интегрирования    
    char       fPreciseSrcStep;                       // Флаг - использовать уточнение шага для разрывных источников сигнала   
    char       IsFinalStage;                          // Текущая стадия решения метода интегрирования 0 - финальная, 1 - предварительная   
    char       IterateLoopsOnStartForImlicitMethods;  // Флаг - итерировать алгебраические переменные на старте для неявных методов интегрирования   
    char       ShowAccuracyErrorsFlag;                // Флаг - показывать ошибки несходимости   
    char       LimitMinStep;                          // Флаг - не снижать шаг при точной синхронизации ниже min_step   
    char       RoundTimeStepToMinStep;                // Флаг - округлять шаг интегрирования кратно минимальному шагу min_step   
    char       fOptimizeJacoby;                       // Флаг - использовать оптимизацию при расчёта матрицы Якоби    
    char       fOptimizeLinear;                       // Использовать оптимизацию линейных членов Якобиана    
    char       ChangeJac;                             // Флаг изменения матрицы Якоби для некоторых методов интегрирования (для аналитического формирования матрицы может выставляться ручками)    
    char       ChangeJacobyStructFlag;                // Флаг изменения структуры матрицы Якоби - для переиндексации   
    char       NeedRefactFlag;                        // Флаг необходимости перефакторизации, выставляемый на основании изменений матрицы
	char       SetMinStepOnSteps;                     // Флаг выставить минимальный шаг при переходе для ступенчатых источников 
	char       fSetBadErrFlag;                        // Флаг индикации со стороны блока для решателя, что шаг интегрирования плохой и нужен повтор
	
} solver_struct;

typedef solver_struct* PModelODEStruct;

 // Идентификатор объекта-решателя
typedef void* TImId; 
 
 // Интерфейс решателя разреженных СЛАУ
typedef struct { 
	//Создать солвер
	TImId    (core_call_type *im_solver_create)();  
	//Уничтожить солвер
	int      (core_call_type *im_solver_free)(TImId im_solver_id);  
	//Получить информацию о методе нужную для выделения памяти ядром программы
	intptr_t (core_call_type *im_getinfo)(TImId  im_solver_id, solver_struct* pdae_struct);
	//Инициализировать метод интегрирования заданной размерностью
	intptr_t (core_call_type *im_init)(TImId im_solver_id);
	//Главная Функция внешнего метода интегрирования
	// im_solver_id  - идентификатор объекта метода интегрирования
	// t             - модельное время метода
	// h             - шаг метода
	// tfin          - финальное время метода интегрирования
	intptr_t (core_call_type *im_run)(TImId im_solver_id, double* t, double* h, double* tfin);
} TImAbstractInterface;


//Указатель на интерфейсные функции метода
typedef TImAbstractInterface* PImAbstractInterface; 

 // Интерфейс загрузчика решателя ДАУ
typedef struct {
 //Создасть солвер
 solver_struct*  (core_call_type *ode_solver_create_ode)(void* aOwnerObj);
 //Уничтожить солвер
 int             (core_call_type *ode_solver_free_ode)(solver_struct* im_solver_id);  
} TODEInterface;

//Указатель на загрузчик решателя ДАУ
typedef TODEInterface* PODEInterface; 

#endif

