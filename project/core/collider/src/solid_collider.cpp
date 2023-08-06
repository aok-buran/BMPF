#include "solid_collider.h"

using namespace bmpf;

/**
 * Деструктор
 */
SolidCollider::~SolidCollider() {
    // добавляем все звенья
    for (std::shared_ptr<Solid3Object> &obj: _links)
        DT_RemoveObject(_scene, obj->getHandle());

    DT_DestroyScene(_scene);

    _collidersMap.clear();
}

/**
 * получить подвектор списков моделей объектов по вектору индексов
 * @param groupedModelPaths вектор векторов путей к моделям. это связано с тем,
 * что сцена строится на основе набора описаний роботов
 * каждый из них содержит список путей к своим моделям
 * @param indexes индексы роботов, списки моделей которых нужно использовать
 * @return подвектор списков моделей объектов
 */
std::vector<std::vector<std::string>> SolidCollider::_getSubGroupedModelPaths(
        const std::vector<std::vector<std::string>> &groupedModelPaths, const std::vector<int> &indexes
) {
    std::vector<std::vector<std::string>> res;
    for (int index: indexes)
        res.push_back(groupedModelPaths.at(index));

    return res;
}


/**
 * заполнить словарь коллайдеров подгрупп объектов
 * @param groupedModelPaths  вектор векторов путей к моделям. это связано с тем,
 * что сцена строится на основе набора описаний роботов
 * каждый из них содержит список путей к своим моделям
 * @param indexes индексы роботов, списки моделей которых нужно использовать
 * @param pos текущее количество объектов
 * @param val текущий индекс перебора
 */
void SolidCollider::_populateColliderMap(
        const std::vector<std::vector<std::string>> &groupedModelPaths, std::vector<int> indexes, int pos, int val
) {
    if (pos > groupedModelPaths.size() + 2)
        return;

    for (int i = val; i < groupedModelPaths.size(); i++) {
        if (indexes.size() <= pos)
            indexes.push_back(i);
        else
            indexes[pos] = i;

        std::vector<int> key;
        key.assign(indexes.begin(), indexes.end());

        std::shared_ptr<SolidCollider> solidCollider = std::make_shared<SolidCollider>();

        solidCollider->init(
                _getSubGroupedModelPaths(groupedModelPaths, indexes), false
        );
        _collidersMap.insert(std::make_pair(key, solidCollider));
        _populateColliderMap(groupedModelPaths, indexes, pos + 1, i + 1);
    }
}


/**
 * инициализация коллайдера
 *
 * @param groupedModelPaths вектор векторов путей к моделям. это связано с тем,
 * что сцена строится на основе набора описаний роботов
 * каждый из них содержит список путей к своим моделям
 * @param subColliders флаг, нудно ли составить виртуальные сцены для
 * каждого подмножества объектов сцены
 */
void SolidCollider::init(std::vector<std::vector<std::string>> groupedModelPaths, bool subColliders) {
    // очищаем списки
    _links.clear();
    _objectIndexRanges.clear();
    _groupedLinks.clear();
    _collidersMap.clear();

    // робот один, если список путей к моделям всего один
    _isSingleObject = groupedModelPaths.size() == 1;

    if (!_isSingleObject && subColliders)
        _populateColliderMap(groupedModelPaths);

    // Формируем отдельные списки диапазонов звеньев и их моделей для каждого робота
    unsigned long pos = 0;
    for (const std::vector<std::string> &modelPaths: groupedModelPaths) {

        std::vector<std::shared_ptr<Solid3Object>> localLinks;
        //if (!_isSingleObject) {
        _objectIndexRanges.emplace_back(pos, modelPaths.size() + pos - 1);
        pos += modelPaths.size();
        //}
        for (const std::string &path: modelPaths) {
            std::shared_ptr<Solid3Object> ptr = Solid3Object::fromStlFile(
                    path, modelPaths.size() != 1
            );
            _links.emplace_back(ptr);
            localLinks.emplace_back(ptr);
        }
        _groupedLinks.emplace_back(std::move(localLinks));
    }
    // создаём сцену
    _scene = DT_CreateScene();
    // добавляем на неё все звенья
    for (std::shared_ptr<Solid3Object> &obj: _links)
        DT_AddObject(_scene, obj->getHandle());
}

/**
 *  порождает массив из 16 элементов из значений матрицы m
 * (после использования не забудьте освободить память)
 * @param m матрица преобразования
 * @return массив из 16 элементов из значений матрицы
 */
double *SolidCollider::_eigenToDouble(Eigen::Matrix4d m) {
    auto *matrix = new double[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            matrix[4 * i + j] = m(i, j);
    }
    return matrix;
}

/**
 * задать матрицы преобразования, библиотека solid построена так,
 * что нужно сначала задать матрицы преобразования, а потом
 * выполнить то или иное действие: нарисовать, проверить коллизии и т.д.
 * поэтому метод в начале пытается заблокировать мьютекс _setTransformMutex
 * с заданным интервалом, когда закончите использование текущее состояние сцены
 * НЕ ЗАБУДЬТЕ ВЫЗВАТЬ МЕТОД _makeFree()
 * @param matrices матрицы преобразования
 */
void SolidCollider::_setTransformMatrices(std::vector<Eigen::Matrix4d> matrices) {
    if (_links.size() != matrices.size()) {
        char buf[1024];
        sprintf(buf,
                "SolidCollider::getBoxPoints ERROR: \n _links size is %zu, but matrices size is %zu"
                "\nthey must be equal",
                _links.size(), matrices.size()
        );
        throw std::invalid_argument(buf);
    }

    // пока не получается выставить мьютекс
    while (!_setTransformMutex.try_lock())
        // делаем паузу в одну мкросекунду
        std::this_thread::sleep_for(std::chrono::microseconds(1));

    // задаём матрицы трансформации
    const unsigned long itCnt = _links.size();
    for (unsigned long i = 0; i < itCnt; i++) {
        double *position = _eigenToDouble(matrices.at(i).transpose());
        DT_SetMatrixd(_links.at(i)->getHandle(), position);
        delete[] position;
    }
}

/**
 * задать матрицы преобразования для робота с индексом robotNum,
 * библиотека solid построена так,
 * что нужно сначала задать матрицы преобразования, а потом
 * выполнить то или иное действие: нарисовать, проверить коллизии и т.д.
 * поэтому метод в начале пытается заблокировать мьютекс _setTransformMutex
 * с заданным интервалом, когда закончите использование текущее состояние сцены
 * НЕ ЗАБУДЬТЕ ВЫЗВАТЬ МЕТОД _makeFree()
 * @param robotNum номер робота
 * @param matrices матрицы преобразования
 */
void SolidCollider::_setTransformMatrices(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) {
    if (robotNum >= _groupedLinks.size()) {
        char buf[1024];
        sprintf(buf,
                "SolidCollider::getBoxPoints ERROR: \n robotNum is %zu, but _groupedLinks size is %zu",
                robotNum, _groupedLinks.size()
        );
        throw std::invalid_argument(buf);
    }

    // пока не получается выставить мьютекс
    while (!_setTransformMutex.try_lock())
        // делаем паузу в одну мкросекунду
        std::this_thread::sleep_for(std::chrono::microseconds(1));

    // обновляем матрицы для робота с индексом robotNum
    for (unsigned long i = _objectIndexRanges.at(robotNum).first;
         i < _objectIndexRanges.at(robotNum).second;
         i++) {
        double *position = _eigenToDouble(matrices.at(i).transpose());
        DT_SetMatrixd(_links.at(i)->getHandle(), position);
        delete[] position;
    }
}

/**
 * рисование сцены коллайдера
 * @param matrices список матриц преобразования из текущей
 * СК звена в следующую
 * @param onlyRobot флаг, нужно ли рисовать только роботов
 * или ещё и статические объекты сцены
 */
void SolidCollider::paint(std::vector<Eigen::Matrix4d> matrices, bool onlyRobot) {
    _setTransformMatrices(matrices);
    for (auto &_link: _links)
        _link->paintGL(onlyRobot);
    _makeFree();
}

/**
 * @brief  получить координаты куба, ограничивающего объём робота
 * получить координаты куба, ограничивающего объём, в котором
 * помещён робот с индексом robotNum, если задать его звеньям
 * матрицы трансформации из списка всех матриц трансформации
 * звеньев сцены, возвращает список из шести координат:
 * min.x(), min.y(), min.z(), max.x(), max.y(), max.z()
 * @param robotNum номер робота в списке роботов
 * @param matrices список матриц преобразований звеньев
 * @return получить координаты куба, ограничивающего объём робота
 */
std::vector<double> SolidCollider::getBoxCoords(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) {
    if (robotNum >= _groupedLinks.size()) {
        char buf[1024];
        sprintf(buf,
                "SolidCollider::getBoxCoords() ERROR: \n robotNum is %zu, but _groupedLinks size is %zu",
                robotNum, _groupedLinks.size()
        );
        throw std::invalid_argument(buf);
    }
    _setTransformMatrices(robotNum, matrices);

    // определяем максимальные и минимальные значения каждой из координат
    MT_Point3 min;
    MT_Point3 max;
    DT_GetBBox(_groupedLinks.at(robotNum).front()->getHandle(), min, max);
    std::vector<double> startMinMax = {min.x(), min.y(), min.z(), max.x(), max.y(), max.z()};
    for (int j = 1; j < _groupedLinks.at(robotNum).size(); j++) {
        auto &link = _groupedLinks.at(robotNum).at(j);
        // получаем координаты параллелепипеда, ограничивающего j-е звено
        DT_GetBBox(link->getHandle(), min, max);
        std::vector<double> minMax = {min.x(), min.y(), min.z(), max.x(), max.y(), max.z()};

        for (unsigned int i = 0; i < 3; i++)
            if (startMinMax.at(i) > minMax.at(i))
                startMinMax.at(i) = minMax.at(i);

        for (unsigned int i = 3; i < 6; i++)
            if (startMinMax.at(i) < minMax.at(i))
                startMinMax.at(i) = minMax.at(i);
    }
    _makeFree();
    return startMinMax;
}

/**
 * @brief получить точки куба, ограничивающего объём робота
 * получить точки куба, ограничивающего объём, в котором
 * помещён робот с индексом robotNum, если задать его звеньям
 * матрицы трансформации из списка всех матриц трансформации
 * звеньев сцены, возвращает список из точек,
 * по которым построится куб линиями GL_LINES
 * ax, ay, az, bx, by, bz
 * @param robotNum номер робота в списке роботов
 * @param matrices список матриц преобразований звеньев
 * @return получить точки куба, ограничивающего объём робота
 */
std::vector<double> SolidCollider::getBoxPoints(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) {
    if (robotNum >= _groupedLinks.size()) {
        char buf[1024];
        sprintf(buf,
                "SolidCollider::getBoxPoints() ERROR: \n robotNum is %zu, but _groupedLinks size is %zu"
                "\nthey must be equal",
                robotNum, _groupedLinks.size()
        );
        throw std::invalid_argument(buf);
    }

    std::vector<double> mm = getBoxCoords(robotNum, matrices);

    return {
            mm.at(0), mm.at(1), mm.at(2), // x1
            mm.at(0), mm.at(4), mm.at(2), // x2

            mm.at(0), mm.at(1), mm.at(2), // x1
            mm.at(0), mm.at(1), mm.at(5), // x5

            mm.at(0), mm.at(1), mm.at(2), // x1
            mm.at(3), mm.at(1), mm.at(2), // x4


            mm.at(3), mm.at(4), mm.at(2), // x3
            mm.at(0), mm.at(4), mm.at(2), // x2

            mm.at(3), mm.at(4), mm.at(2), // x3
            mm.at(3), mm.at(1), mm.at(2), // x4

            mm.at(3), mm.at(4), mm.at(2), // x3
            mm.at(3), mm.at(4), mm.at(5), // x7


            mm.at(0), mm.at(4), mm.at(5), // x6
            mm.at(3), mm.at(4), mm.at(5), // x7

            mm.at(0), mm.at(4), mm.at(5), // x6
            mm.at(0), mm.at(4), mm.at(2), // x2

            mm.at(0), mm.at(4), mm.at(5), // x6
            mm.at(0), mm.at(1), mm.at(5), // x5


            mm.at(3), mm.at(1), mm.at(5), // x8
            mm.at(3), mm.at(1), mm.at(2), // x4

            mm.at(3), mm.at(1), mm.at(5), // x8
            mm.at(3), mm.at(4), mm.at(5), // x7

            mm.at(3), mm.at(1), mm.at(5), // x8
            mm.at(0), mm.at(1), mm.at(5), // x5
    };
}

/**
 * проверка, соответствует ли коллизии текущее состояние сцены
 * @return флаг, соответствует ли коллизии текущее состояние сцены
 */
bool SolidCollider::_isCollided() {
    // специальная переменная, в которую solid3 сохраняет точку пересечения
    MT_Point3 cp1;
    // перебираем пары звеньев
    for (int i = 0; i < _links.size(); i++)
        for (int j = 0; j < _links.size(); j++) {
            // если робот всего один
            if (_isSingleObject) {
                // если звенья соседние, пропускаем это проверку
                if (std::abs(i - j) <= 1)
                    continue;

                // если звенья пересекаются
                if (DT_GetCommonPoint(
                        _links.at(i)->getHandle(), _links.at(j)->getHandle(), cp1))
                    return true;
            } else {
                // если звенья совпадают, пропускаем это проверку
                if (i == j)
                    continue;
                // определяем, являются ли звенья соседними в одном и то же роботом
                bool neighbors = false;
                for (auto robotRange: _objectIndexRanges) {
                    if (i >= robotRange.first && i <= robotRange.second &&
                        j >= robotRange.first && j <= robotRange.second &&
                        (i == j + 1 || i == j - 1)) {
                        neighbors = true;
                        break;
                    }
                }
                // если звенья не являются соседями и пересекаются
                if (!neighbors && DT_GetCommonPoint(
                        _links.at(i)->getHandle(), _links.at(j)->getHandle(), cp1))
                    return true;
            }
        }

    return false;
}


/**
 * проверка соответствует ли состояние сцены столкновению
 * @param matrices список матриц преобразований звеньев
 * @return флаг, соответствует ли состояние сцены столкновению
 */
bool SolidCollider::isCollided(std::vector<Eigen::Matrix4d> matrices) {
    _setTransformMatrices(std::move(matrices));
    bool ic = _isCollided();
    _makeFree();
    return ic;
}

/**
 * @brief проверка соответствует ли состояние сцены столкновению
 * проверка соответствует ли состояние сцены (список матриц преобразований звеньев
 * только для задействованных объектов)
 * столкновению при этом учитываются только объекты с индексами из robotIndexes
 * @param matrices список матриц преобразований звеньев
 * @param robotIndexes индексы роботов
 * @return флаг, соответствует ли состояние сцены столкновению
 */
bool SolidCollider::isCollided(std::vector<Eigen::Matrix4d> matrices, std::vector<int> robotIndexes) {
    if (_collidersMap.empty()) {
        throw std::runtime_error("SolidCollider::isCollided() ERROR: \n _collidersMap is empty");
    }

    return _collidersMap[robotIndexes]->isCollided(matrices);
}

/**
 * возвращает список всех координат полигона (вектор нормали и координаты вершины):
 * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
 * @param matrices список матриц преобразований звеньев
 * @return возвращает список всех координат полигона
 */
std::vector<float> SolidCollider::getPoints(std::vector<Eigen::Matrix4d> matrices) {
    _setTransformMatrices(std::move(matrices));
    std::vector<float> pointList;
    for (auto &link: _links) {
        std::vector<float> points = link->getTransformedPointsList();
        pointList.insert(pointList.begin(), points.begin(), points.end());
    }
    _makeFree();
    return pointList;
}

