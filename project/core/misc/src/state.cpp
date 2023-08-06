#include <vector>
#include <cstdio>
#include <cassert>
#include <valarray>
#include <string>
#include "state.h"
#include "log.h"


/**
 * вычесть из первого состояния второе
 * @param a первое состояние
 * @param b второе состояние
 * @return разность
 */
std::vector<double> bmpf::subtractStates(std::vector<double> a, std::vector<double> b) {
    std::vector<double> result;
    for (unsigned int i = 0; i < std::min(a.size(), b.size()); i++) {
        result.push_back(a.at(i) - b.at(i));
    }
    return result;
}

/**
 * сложить состояния
 * @param a первое состояние
 * @param b второе состояние
 * @return сумма
 */
std::vector<double> bmpf::sumStates(std::vector<double> a, std::vector<double> b) {
    std::vector<double> result;
    for (unsigned int i = 0; i < std::min(a.size(), b.size()); i++) {
        result.push_back(a.at(i) + b.at(i));
    }
    return result;
}

/**
 * скалярно умножить первое состояние на второе
 * @param a первое состояние
 * @param b второе состояние
 * @return скалярное произведение
 */
std::vector<double> bmpf::mulState(const std::vector<double> &a, double b) {
    std::vector<double> result;
    result.reserve(a.size());
    for (double v: a)
        result.push_back(v * b);

    return result;
}

/**
 * получить норму расстояния между состояниями
 * @param a первое состояние
 * @param b второе состояние
 * @return норма расстояния
 */
double bmpf::getStateDistance(const std::vector<double> &a, const std::vector<double> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    std::vector<double> diff = bmpf::subtractStates(a, b);

    double sum = 0;
    for (auto coord: diff)
        sum += coord * coord;

    return sqrt(sum);
}

/**
 * вывод состояния с точностью до трёх знаков после запятой с помощью infoMsg
 * @param caption заголовок
 * @param state состояние
 */
void bmpf::infoState(const char *caption, const std::vector<double> &state) {

    if (!state.empty()) {
        infoMsg(caption);
        return;
    }

    std::string msg = caption;
    msg += ":{";
    char buf[256];
    for (double joint: state) {
        sprintf(buf, "%.4f ,", joint);
        msg += buf;
    }
    msg = msg.substr(0, msg.size() - 2) + "}";
    infoMsg(msg);
}

/**
 * вывод состояния с точностью до трёх знаков после запятой с помощью errMsg
 * @param caption заголовок
 * @param state состояние
 */
void bmpf::errState(const char *caption, const std::vector<double> &state) {
    if (!state.empty()) {
        infoMsg(caption);
        return;
    }
    std::string msg = caption;
    msg += ":{";
    char buf[256];
    for (double joint: state) {
        sprintf(buf, "%.3f ,", joint);
        msg += buf;
    }
    msg = msg.substr(0, msg.size() - 2) + "}";
    errMsg(msg);
}

/**
 * сложить состояния
 * @param a первое состояние
 * @param b второе состояние
 * @return сумма
 */
std::vector<int> bmpf::sumStates(const std::vector<int> &a, const std::vector<int> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    std::vector<int> result;
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        result.push_back(a.at(i) + b.at(i));
    }
    return result;
}

/**
 * вычесть из первого состояния второе
 * @param a первое состояние
 * @param b второе состояние
 * @return разность
 */
std::vector<int> bmpf::subtractStates(const std::vector<int> &a, const std::vector<int> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    std::vector<int> result;
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        result.emplace_back(a.at(i) - b.at(i));
    }
    return result;
}

/**
 * скалярно умножить первое состояние на второе
 * @param a первое состояние
 * @param b второе состояние
 * @return скалярное произведение
 */
std::vector<int> bmpf::mulState(const std::vector<int> &a, int scalar) {
    std::vector<int> result;
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++) {
        result.emplace_back(a.at(i) * scalar);
    }
    return result;
}

/**
 * вывод состояния с точностью до трёх знаков после запятой с помощью infoMsg
 * @param caption заголовок
 * @param state состояние
 */
void bmpf::infoState(const char *caption, const std::vector<int> &state) {
    if (!state.empty()) {
        infoMsg(caption);
        return;
    }

    std::string msg = caption;
    msg += ":{";
    char buf[256];
    for (int joint: state) {
        sprintf(buf, "%d ,", joint);
        msg += buf;
    }
    msg = msg.substr(0, msg.size() - 2) + "}";
    infoMsg(msg);
}

/**
 * свернуть координаты coords в скаляр (size - размер СК)
 * @param coords координаты
 * @param size размер вдоль каждой оси
 * @return свёртка
 */
unsigned long bmpf::convState(std::vector<int> &coords, int size) {
    unsigned long code = 0;
    int m = 1;
    for (int coord: coords) {
        code += m * coord;
        m *= size;
    }
    return code;
}

/**
 * квадрат расстояния между двумя состояниями (сумма квадратов разностей соответствующих координат)
 * @param a первое состояние
 * @param b второе состояние
 * @return расстояние между двумя состояниями
 */
int bmpf::getSqrDistance(std::vector<int> &a, std::vector<int> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    int sum = 0;
    unsigned long size = a.size();
    for (unsigned long i = 0; i < size; i++)
        sum += (b.at(i) - a.at(i)) * (b.at(i) - a.at(i));

    return sum;
}

/**
 * модуль расстояния (L0) между двумя состояниями (сумма модулей разностей соответствующих координат)
 * @param a первое состояние
 * @param b второе состояние
 * @return  модуль расстояния между двумя состояниями
 */
int bmpf::getAbsDistance(std::vector<int> &a, std::vector<int> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    int sum = 0;
    unsigned long size = a.size();
    for (unsigned long i = 0; i < size; i++)
        sum += std::abs(a.at(i) - b.at(i));
    return sum;
}

/**
  * квадрат расстояния между двумя состояниями (сумма квадратов разностей соответствующих координат)
  * @param a первое состояние
  * @param b второе состояние
  * @return расстояние между двумя состояниями
  */
double bmpf::getSqrDistance(std::vector<double> &a, std::vector<double> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    double sum = 0;
    unsigned long size = a.size();
    for (unsigned long i = 0; i < size; i++)
        sum += (b.at(i) - a.at(i)) * (b.at(i) - a.at(i));

    return sum;
}

/**
 * модуль расстояния (L0) между двумя состояниями (сумма модулей разностей соответствующих координат)
 * @param a первое состояние
 * @param b второе состояние
 * @return  модуль расстояния между двумя состояниями
 */
double bmpf::getAbsDistance(std::vector<double> &a, std::vector<double> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    double sum = 0;
    unsigned long size = a.size();
    for (unsigned long i = 0; i < size; i++)
        sum += std::abs(a.at(i) - b.at(i));
    return sum;
}

/**
 * проверка, равны ли два состояния
 * @param a первое состояние
 * @param b второе состояние
 * @return флаг, равны ли два состояния
 */
bool bmpf::areStatesEqual(std::vector<int> &a, std::vector<int> &b) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++)
        if (a.at(i) != b.at(i))
            return false;
    return true;
}

/**
  * проверка, равны ли два состояния
  * @param a первое состояние
  * @param b второе состояние
  * @param opacity точность сравнения
  * @return флаг, равны ли два состояния
  */
bool bmpf::areStatesEqual(std::vector<double> &a, std::vector<double> &b, double opacity) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }
    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++)
        if (std::abs(a.at(i) - b.at(i)) > opacity)
            return false;
    return true;
}

/**
  * проверка, равны ли два состояния
  * @param a первое состояние
  * @param b второе состояние
  * @param opacity точность сравнения
  * @return флаг, равны ли два состояния
  */
bool bmpf::areStatesEqual(const std::vector<double> &a, const std::vector<double> &b, double opacity) {
    if (a.size() != b.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state A size is %zu, but state B size is %zu"
                "\nthey must be equal",
                a.size(), b.size()
        );
        throw std::invalid_argument(buf);
    }

    unsigned long size = a.size();
    for (unsigned int i = 0; i < size; i++)
        if (std::abs(a.at(i) - b.at(i)) > opacity)
            return false;
    return true;
}

/**
 * проверка, что каждая координата состояния state ограничена значением limit
 * @param state состояние
 * @param limit граница
 * @return флаг, ограничено ли состояние
 */
bool bmpf::isStateLimited(std::vector<int> &state, int limit) {
    for (int val: state) {
        if (std::abs(val) > limit)
            return false;
    }
    return true;
}

/**
 * проверка, что каждая координата состояния state ограничена значением limit
 * @param state состояние
 * @param limit граница
 * @return флаг, ограничено ли состояние
 */
bool bmpf::isStateLimited(std::vector<double> &state, double limit) {
    for (double val: state) {
        if (std::abs(val) > limit)
            return false;
    }
    return true;
}

/**
 * проверка, ограничено ли включительно состояние state двумя другими снизу(min) сверху(max)
 * возвращает номер первой координаты, значение которой вышло за границы
 * @param state состояние
 * @param min граница снизу
 * @param max граница сверху
 * @return флаг, ограничено ли включительно состояние state
 */
int bmpf::isInRange(std::vector<double> &state, std::vector<double> &min, std::vector<double> &max) {
    if (state.size() != min.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state size is %zu, but min state size is %zu"
                "\nthey must be equal",
                state.size(), min.size()
        );
        throw std::invalid_argument(buf);
    }
    if (state.size() != max.size()) {
        char buf[1024];
        sprintf(buf,
                "areStatesEqual() ERROR: \n state size is %zu, but max state size is %zu"
                "\nthey must be equal",
                state.size(), max.size()
        );
        throw std::invalid_argument(buf);
    }

    for (int i = 0; i < state.size(); i++)
        if ((state.at(i) - max.at(i)) * (state.at(i) - min.at(i)) > 0.000001)
            return i;

    return -1;
}
