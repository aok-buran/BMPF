#pragma once

namespace bmpf {

    /**
     * вычесть из первого состояния второе
     * @param a первое состояние
     * @param b второе состояние
     * @return разность
     */
    std::vector<double> subtractStates(std::vector<double> a, std::vector<double> b);

    /**
     * вычесть из первого состояния второе
     * @param a первое состояние
     * @param b второе состояние
     * @return разность
     */
    std::vector<int> subtractStates(const std::vector<int> &a, const std::vector<int> &b);

    /**
     * сложить состояния
     * @param a первое состояние
     * @param b второе состояние
     * @return сумма
     */
    std::vector<double> sumStates(std::vector<double> a, std::vector<double> b);

    /**
     * сложить состояния
     * @param a первое состояние
     * @param b второе состояние
     * @return сумма
     */
    std::vector<int> sumStates(const std::vector<int> &a, const std::vector<int> &b);

    /**
     * скалярно умножить первое состояние на второе
     * @param a первое состояние
     * @param b второе состояние
     * @return скалярное произведение
     */
    std::vector<double> mulState(const std::vector<double> &a, double b);

    /**
     * скалярно умножить первое состояние на второе
     * @param a первое состояние
     * @param b второе состояние
     * @return скалярное произведение
     */
    std::vector<int> mulState(const std::vector<int> &a, int scalar);

    /**
     * получить норму расстояния между состояниями
     * @param a первое состояние
     * @param b второе состояние
     * @return норма расстояния
     */
    double getStateDistance(const std::vector<double> &a, const std::vector<double> &b);

    /**
     * вывод состояния с точностью до трёх знаков после запятой с помощью infoMsg
     * @param caption заголовок
     * @param state состояние
     */
    void infoState(const char *caption, const std::vector<int> &state);

    /**
     * вывод состояния с точностью до трёх знаков после запятой с помощью infoMsg
     * @param caption заголовок
     * @param state состояние
     */
    void infoState(const char *caption, const std::vector<double> &state);

    /**
     * вывод состояния с точностью до трёх знаков после запятой с помощью errMsg
     * @param caption заголовок
     * @param state состояние
     */
    void errState(const char *caption, const std::vector<double> &state);

    /**
     * свернуть координаты coords в скаляр (size - размер СК)
     * @param coords координаты
     * @param size размер вдоль каждой оси
     * @return свёртка
     */
    unsigned long convState(std::vector<int> &coords, int size);

    /**
     * квадрат расстояния между двумя состояниями (сумма квадратов разностей соответствующих координат)
     * @param a первое состояние
     * @param b второе состояние
     * @return расстояние между двумя состояниями
     */
    int getSqrDistance(std::vector<int> &a, std::vector<int> &b);

    /**
     * модуль расстояния (L0) между двумя состояниями (сумма модулей разностей соответствующих координат)
     * @param a первое состояние
     * @param b второе состояние
     * @return  модуль расстояния между двумя состояниями
     */
    int getAbsDistance(std::vector<int> &a, std::vector<int> &b);

    /**
     * квадрат расстояния между двумя состояниями (сумма квадратов разностей соответствующих координат)
     * @param a первое состояние
     * @param b второе состояние
     * @return расстояние между двумя состояниями
     */
    double getSqrDistance(std::vector<double> &a, std::vector<double> &b);

    /**
     * модуль расстояния (L0) между двумя состояниями (сумма модулей разностей соответствующих координат)
     * @param a первое состояние
     * @param b второе состояние
     * @return  модуль расстояния между двумя состояниями
     */
    double getAbsDistance(std::vector<double> &a, std::vector<double> &b);

    /**
     * проверка, равны ли два состояния
     * @param a первое состояние
     * @param b второе состояние
     * @return флаг, равны ли два состояния
     */
    bool areStatesEqual(std::vector<int> &a, std::vector<int> &b);

    /**
     * проверка, равны ли два состояния
     * @param a первое состояние
     * @param b второе состояние
     * @param opacity точность сравнения
     * @return флаг, равны ли два состояния
     */
    bool areStatesEqual(std::vector<double> &a, std::vector<double> &b, double opacity);

    /**
     * проверка, равны ли два состояния
     * @param a первое состояние
     * @param b второе состояние
     * @param opacity точность сравнения
     * @return флаг, равны ли два состояния
     */
    bool areStatesEqual(const std::vector<double> &a, const std::vector<double> &b, double opacity);

    /**
     * проверка, что каждая координата состояния state ограничена значением limit
     * @param state состояние
     * @param limit граница
     * @return флаг, ограничено ли состояние
     */
    bool isStateLimited(std::vector<int> &state, int limit);

    /**
     * проверка, что каждая координата состояния state ограничена значением limit
     * @param state состояние
     * @param limit граница
     * @return флаг, ограничено ли состояние
     */
    bool isStateLimited(std::vector<double> &state, double limit);

    /**
     * проверка, ограничено ли включительно состояние state двумя другими снизу(min) сверху(max)
     * возвращает номер первой координаты, значение которой вышло за границы
     * @param state состояние
     * @param min граница снизу
     * @param max граница сверху
     * @return флаг, ограничено ли включительно состояние state
     */
    int isInRange(std::vector<double> &state, std::vector<double> &min, std::vector<double> &max);
}