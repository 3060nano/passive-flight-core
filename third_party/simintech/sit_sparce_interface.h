
#if __GNUC__
 // GCC compiler
#define sparce_solver_call_type 

#else

 // MSVC 
#define sparce_solver_call_type _cdecl

#endif

#ifndef sp_intf_def

#define sp_intf_def


 //Структура настроек для метода старого метода интегрирования МВТУ-3.7
typedef struct {
   uint64_t API_VER;               // Резервные данные - версия API
   double   SP_TB;                 // барьер
   double   SP_TU;                 // коэффициент численной устойчивости
   double   SP_sp_eps;             // допустимая относительная ошибка итераций
   double   SP_sp_abserr;          // допустимая абсолютная ошибка
   double   SP_sp_errdxstart;      // максимальная начальная ошибка
   int      SP_Rows;               // число просматриваемых строк
   int      SP_sp_maxiter;         // максималльное к-во итераций для SparceIter
   char     SP_Iter_flag;          // Флаг итерационного уточнения решения
} TSP_SolverSettingsRec;


  /* Тип указателя на объект-решатель разреженных СЛАУ - по умолчанию это обычный указатель */
typedef void* sp_solve_id;

  /* Описание интерфейса решателя разреженных матриц */
typedef struct {

  //Создание экземпляра решателя разреженных матриц с заданным интерфейсом
  sp_solve_id (sparce_solver_call_type *lae_solver_create)();
  //Уничтожение экземпляра объекта-решателя разреженной матрицы  
  int (sparce_solver_call_type *lae_solver_free)(sp_solve_id lae_solver_id);  
    
  //Возвращает количество уравнений глобальной линейной системы
  int (sparce_solver_call_type *lae_solver_getlaecount)(sp_solve_id lae_solver_id);
  //Увеличивает количество уравнений глобальной линейной системы
  int (sparce_solver_call_type *lae_solver_addlaecount)(sp_solve_id lae_solver_id, int added_count);
  
  //Добавить указатели на собственный блок
  int (sparce_solver_call_type *lae_solver_setselfptr)(sp_solve_id lae_solver_id, void* self_ptr);
  //Получить идентификатор последнего блока
  void* (sparce_solver_call_type *lae_solver_getlastptr)(sp_solve_id lae_solver_id);
  //Получить идентификатор первого блока
  void* (sparce_solver_call_type *lae_solver_getfirstptr)(sp_solve_id lae_solver_id);
    
  //Инициализация линейной системы 
  int (sparce_solver_call_type *lae_solver_begininit)(sp_solve_id lae_solver_id);
  //Инициализация размера и номеров строки уравнения
  int (sparce_solver_call_type *lae_solver_initrow)(sp_solve_id lae_solver_id, int n_row, int n_col, int* col_indexes);  
  //Присвоение в метод начального состояния (для итерационных методов)
  int (sparce_solver_call_type *lae_solver_setinitresult)(sp_solve_id lae_solver_id, int index, double x0);
  //Инициализация LU-разложения
  int (sparce_solver_call_type *lae_solver_endinit)(sp_solve_id lae_solver_id);
   
  //Установить значение правой части в строке уравнения
  int (sparce_solver_call_type *lae_solver_setrightcol)(sp_solve_id lae_solver_id,int n_row,double bvalue);
  //Сбросить коэффициенты строки с 0
  int (sparce_solver_call_type *lae_solver_resetrow)(sp_solve_id lae_solver_id,int n_row,int n_col);
  //Установить значение коэффициента строки с заданным порядковым номером order_index
  int (sparce_solver_call_type *lae_solver_fillitem)(sp_solve_id lae_solver_id,int n_row,int index,double coef,int sum_cols,int order_index);
  
  //LU-разложение
  int (sparce_solver_call_type *lae_solver_ludecomp)(sp_solve_id lae_solver_id);
  //Решение системы
  int (sparce_solver_call_type *lae_solver_solve)(sp_solve_id lae_solver_id);
  //Считывание результатов решения линейной системы
  double (sparce_solver_call_type *lae_solver_getresult)(sp_solve_id lae_solver_id,int index);
  
  //Установить флаг пересортировки системы (в дальнейшем это скорее всего будет убрано за ненужностью, т.к. проверка на перефакторизацию будет встроенной)
  int (sparce_solver_call_type *lae_solver_setneedsort)(sp_solve_id lae_solver_id);
  //Установить флаг необходимости LU-разложения матрицы системы уравнений
  int (sparce_solver_call_type *lae_solver_setneedlu)(sp_solve_id lae_solver_id);
  //Установить флаг необходимости LU-разложения матрицы системы при изменении к-ва нулевых элементов
  int(sparce_solver_call_type *lae_solver_setzeroschange)(sp_solve_id lae_solver_id);
  
   //Установить настроечные параметры метода (могут быть разные для разных вариантов методов разреженных СЛАУ)
  int(sparce_solver_call_type *lae_solver_setsettings)(sp_solve_id lae_solver_id, void* DataPtr, unsigned int DataSize);
    
} TSparceSolverAbstractInterface;

/* ссылка на интерфейс решателя разреженных матриц */
typedef TSparceSolverAbstractInterface* p_sparce_solver_interface;

  //Базовая структура для объекта, имеющего интерфейс
typedef struct {

	//Ссылка на интерфейс решателя  -  этот член должен быть всегда первым, т.к. по нему мы определяем что нам вытащить !!!
	p_sparce_solver_interface  solver_interface;

} TSolverInterfacedObject;

  /* получение интерфейса по указателю объекта */
p_sparce_solver_interface lae_solver_getinterface(sp_solve_id lae_solver_id)
{
	void* result;

	result = ((TSolverInterfacedObject*)(lae_solver_id))->solver_interface;

	return result;
};

#endif
