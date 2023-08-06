#pragma once

#include "base/robot.h"

#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <memory>

namespace bmpf {

    /**
     * Класс сцены
     */
    class Scene {
    public:

        /**
         * Конструктор по умолчанию
         */
        Scene() = default;

        /**
         * конструктор сцены от одного робота
         * @param robot робот
         */
        explicit Scene(const std::vector<std::shared_ptr<bmpf::BaseRobot>> &robot);

        /**
         * Деструктор по умолчанию
         */
        ~Scene() = default;

        /**
         * @brief посчитать количество активных роботов
         * посчитать количество активных роботов на сцене (объектов с переменными состояниями)
         * @return количество активных роботов на сцене
         */
        unsigned long getActiveRobotCnt() const;

        /**
         * Добавить робота на сцену
         * @param path путь к описанию робота
         * @return id добавленного робота в общем списке роботов
         */
        unsigned long addObject(std::string path);

        /**
         * добавить робота на сцену
         * @param path путь к описанию робота,
         * @param transformVector вектор трансформации position: x,y,z; rotation: r,p,y; scale: x,y,z
         * @return id добавленного робота в общем списке роботов
         */
        unsigned long addObject(std::string path, std::vector<double> &transformVector);

        /**
         * удалить робота со сцены
         * @param robotNum номер робота в общем списке
         */
        void deleteRobot(long robotNum);

        /**
         * сохранить в файл
         * @param path
         */
        void saveToFile(const std::string &path);

        /**
         * загрузить из файла
         * @param path путь к описанию сцены
         */
        void loadFromFile(const std::string &path);

        /**
         * получить диапазоны индексов сочленений всех роботов
         * @return диапазоны индексов сочленений всех роботов
         */
        std::vector<std::pair<long, long>> getJointIndexRanges() { return _jointIndexRanges; }

        /**
         * получить суммарное количество сочленений у всех роботов на сцене
         * @return суммарное количество сочленений у всех роботов на сцене
         */
        unsigned long getJointCnt() const { return _jointCnt; }

        /**
         * проверка, находится ли на сцене всего один активный робот
         * @return находится ли на сцене всего один активный робот
         */
        bool isSingleActiveRobot() const { return getActiveRobotCnt() == 1; };

        /**
         * получить список параметров сочленений робота
         * @return список параметров сочленений робота
         */
        std::vector<std::shared_ptr<bmpf::JointParams>> getJointParamsList() { return _jointParams; }

        /**
         * получить максимальные по модулю значения скорости каждой из координат
         * @return максимальные по модулю значения скорости каждой из координат
         */
        std::vector<double> getMaxSpeedModules() { return _maxSpeedModules; }

        /**
         * получить максимальные по модулю значения ускорения каждой из координат
         * @return максимальные по модулю значения ускорения каждой из координат
         */
        std::vector<double> getMaxAccelerationModules() { return _maxAccelerationModules; }

        /**
         * получить список звеньев робота
         * @return список звеньев робота
         */
        std::vector<std::shared_ptr<bmpf::Link>> getLinks() const { return _links; }

        /**
         * @brief получить список матриц преобразований всех звеньев
         * получить список матриц преобразований всех звеньев по состоянию сцены
         * @param state состояние
         * @return список матриц преобразований
         */
        std::vector<Eigen::Matrix4d> getTransformMatrices(const std::vector<double> &state);

        /**
         * @brief получить список положений (x,y,z) рабочих инструментов всех роботов
         * получить список положений (x,y,z) рабочих инструментов всех роботов по состоянию сцены
         * @param state состояние сцены
         * @return список положений
         */
        std::vector<double> getEndEffectorPositions(const std::vector<double> &state);

        /**
         * Получить частную производную положений рабочих инструментов роботов по i-ой координате
         * @param state состояние сцены
         * @param iVal индекс координаты, по которой берётся производная
         * @return  частная производная положений рабочих инструментов
         */
        std::vector<double> getEndEffectorDiffPositions(const std::vector<double>& state, int iVal);

        /**
         * получить частную производную положений рабочих инструментов роботов по i-ой и j-ой координатам
         * @param state состояние сцены
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @return  частная производная положений рабочих инструментов
         */
        std::vector<double> getEndEffectorDiff2Positions(const std::vector<double>& state, int iVal, int jVal);

        /**
         * получить матрицы преобразования из СК базы робота в СК рабочего инструмента для каждого робота сцены
         * @param state  состояние сцены
         * @return матрицы преобразования
         */
        std::vector<Eigen::Matrix4d> getEndEffectorTransformMatrices(const std::vector<double>& state);

        /**
         * получить частные производные матриц преобразования из СК базы робота в СК рабочего инструмента
         * для каждого робота сцены по i-ой координате
         * @param state  состояние сцены
         * @param iVal индекс координаты, по которой берётся производная
         * @return частные производные матриц преобразования
         */
        std::vector<Eigen::Matrix4d> getEndEffectorDiffTransformMatrices(const std::vector<double>& state, int iVal);

        /**
         * получить частные производные матриц преобразования из СК базы робота в СК рабочего инструмента для каждого робота сцены
         * по i-ой координате и j-ой координате
         * @param state  состояние сцены
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @return частные производные матриц преобразования
         */
        std::vector<Eigen::Matrix4d>
        getEndEffectorDiff2TransformMatrices(const std::vector<double>& state, int iVal, int jVal);

        /**
         * получить список положений и ориентаций рабочего инструмента (заданы параметрами Родриго-Гамильтона)
         * @param state  состояние сцены
         * @return список положений и ориентаций
         */
        std::vector<double> getEndEffectorRGVectors(const std::vector<double> &state);

        /**
         * получить частную производную положения и ориентации рабочего инструмента
         * (заданы параметрами Родриго-Гамильтона) по i-ой координате
         * @param state  состояние сцены
         * @param iVal индекс координаты, по которой берётся производная
         * @return частная производная
         */
        std::vector<double> getEndEffectorDiffRGVector(const std::vector<double> &state, int iVal);

        /**
         *  получить частную производную положения и ориентации рабочего инструмента
         * (заданы параметрами Родриго-Гамильтона) по i-ой и j-ой координатам
         * @param state  состояние сцены
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @return частная производная
         */
        std::vector<double> getEndEffectorDiff2RGVector(const std::vector<double> &state, int iVal, int jVal);

        /**
         * получить список положений звеньев всех роботов по состоянию сцены (x,y,z)
         * @param state  состояние сцены
         * @return список положений звеньев
         */
        std::vector<double> getAllLinkPositions(const std::vector<double> &state);

        /**
         * получить список положений (x,y,z) и ориентаций (параметры Родриго-Гамильтона)
         * звеньев всех роботов по состоянию сцены
         * @param state состояние сцены
         * @return список положений и ориентаций
         */
        std::vector<double> getAllLinkRGVectors(const std::vector<double> &state);

        /**
         * проверка, допустимо ли состояние сцены (по допустимым диапазонам звеньев)
         * @param  state состояние сцены
         * @return флаг, допустимо ли состояние сцены
         */
        bool isStateEnabled(const std::vector<double> &state);

        /**
         * проверка, допустимо ли состояние сцены для конкретного робота
         * (по допустимым диапазонам звеньев)
         * @param state состояние сцены
         * @param robotNum номер робота в списке роботов
         * @return  флаг, допустимо ли состояние сцены
         */
        bool isStateEnabled(std::vector<double> state, unsigned long robotNum);

        /**
         * получить случайное состояние сцены
         * @return случайное состояние сцены
         */
        std::vector<double> getRandomState();

        /**
         * получить список роботов
         * @return список роботов
         */
        const std::vector<std::shared_ptr<bmpf::BaseRobot>> &getRobots() const { return _objects; }

        /**
         * получить путь к описанию сцены
         * @return путь к описанию сцены
         */
        std::string getScenePath() const { return _path; }

        /**
         * Получить виртуальные сцены с одним активным роботом
         * @return  виртуальные сцены с одним активным роботом
         */
        std::vector<std::shared_ptr<Scene>> getSingleRobotScenes() { return _singleRobotScenes; }

        /**
         * получить состояние робота
         * @param state состояние сцены
         * @param objectNum номер робота в списке роботов
         * @return состояние робота
         */
        std::vector<double> getSingleObjectState(const std::vector<double> &state, unsigned long objectNum);

        /**
         * получить смещения для каждого из роботов
         * @return смещения для каждого из роботов
         */
        std::vector<std::vector<double>> getGroupedTranslation() const;

        /**
         * получить повороты для каждого из роботов
         * @return повороты для каждого из роботов
         */
        std::vector<std::vector<double>> getGroupedRotation() const;

        /**
         * получить масштабы для каждого из роботов
         * @return масштабы для каждого из роботов
         */
        std::vector<std::vector<double>> getGroupedScale() const;

        /**
         * получить векторы преобразования для каждого из роботов
         * @return векторы преобразования для каждого из роботов
         */
        std::vector<std::vector<double>> getGroupedTransformVector() const;

        /**
         * получить пути к моделям для каждого из роботов
         * (группируются по роботам)
         * @return пути к моделям для каждого из роботов
         */
        std::vector<std::vector<std::string>> getGroupedModelPaths() const;

        /**
         * задать смещения для каждого из роботов
         * @param groupedTranslation смещения для каждого из роботов
         */
        void setGroupedTranslation(std::vector<std::vector<double>> groupedTranslation);

        /**
         * задать повороты для каждого из роботов
         * @param groupedRotation повороты для каждого из роботов
         */
        void setGroupedRotation(std::vector<std::vector<double>> groupedRotation);

        /**
         * задать масштабы для каждого из роботов
         * @param groupedScale масштабы для каждого из роботов
         */
        void setGroupedScale(std::vector<std::vector<double>> groupedScale);

        /**
         * задать векторы преобразования для каждого из роботов
         * @param transformVectors векторы преобразования для каждого из роботов
         */
        void setGroupedTransformVector(std::vector<std::vector<double>> transformVectors);

        /**
         * объединить состояния роботов (последовательно)
         * @param states список состояний
         * @return объединённое состояние
         */
        static std::vector<double> concatenateStates(const std::vector<std::vector<double>> &states);

        /**
         * объединить состояния роботов (вспомогательный метод)
         * @param states список состояний
         * @return объединённое состояние
         */
        template<typename ...ArgsT>
        std::vector<double> concatenateStates(ArgsT ... states) { return concatenateStates({states...}); }

        /**
         * объединить состояния роботов (последовательно)
         * @param states список состояний
         * @return объединённое состояние
         */
        static std::vector<double> concatenateStates(std::initializer_list<std::vector<double>> states);

        /**
         * объединить состояния роботов (вспомогательный метод)
         * @param states список состояний
         * @return объединённое состояние
         */
        template<typename ...ArgsT>
        std::vector<int> concatenateCoords(ArgsT ... states) { return concatenateCoords({states...}); }

        /**
         * объединить координаты роботов
         * @param coords координаты
         * @return объединённый список координат
         */
        static std::vector<int> concatenateCoords(const std::vector<std::vector<int>> &coords);

        /**
         * получить список флагов, имеет ли тот или иной объект звенья
         * @return список флагов, имеет ли тот или иной объект звенья
         */
        std::vector<bool> areObjectsJointed() { return _areObjectsJointed; }

        /**
         * получить список индексов всех объектов, имеющих звенья
         * @return список индексов всех объектов, имеющих звенья
         */
        std::vector<int> getJointedObjectIndexes() { return _jointedObjectIndexes; }

        /**
         * получить список индексов всех объектов, не имеющих звенья
         * @return список индексов всех объектов, не имеющих звенья
         */
        std::vector<int> getNotJointedObjectIndexes() { return _notJointedObjectIndexes; }

    protected:
        /**
         * заполняет по имеющимся на сцене роботам общие списки jointCnt, _jointIndexRanges, _jointParams и _links
         */
        void _initObjects();

    private:
        /**
         * максимальные по модулю значения скорости каждой из координат
         */
        std::vector<double> _maxSpeedModules;
        /**
         * максимальные по модулю значения ускорения каждой из координат
         */
        std::vector<double> _maxAccelerationModules;
        /**
         * диапазоны индексов сочленений всех роботов
         */
        std::vector<std::pair<long, long>> _jointIndexRanges;
        /**
         *  суммарное количество сочленений у всех роботов на сцене
         */
        unsigned int _jointCnt{};
        /**
         * параметры приводов всех роботов на сцене
         */
        std::vector<std::shared_ptr<bmpf::JointParams>> _jointParams;
        /**
         * звенья роботов, последовательно для каждого робота.
         * Внутри каждого робота сначала идут все звенья, участвующие в иерархии,
         * затем - не участвующие
         */
        std::vector<std::shared_ptr<bmpf::Link>> _links;
        /**
         * роботы на сцене
         */
        std::vector<std::shared_ptr<bmpf::BaseRobot>> _objects;
        /**
         * путь к файлу с описанием сцены
         */
        std::string _path;
        /**
         * список виртуальных сцен для каждого робота в отдельности со всеми статическими объектами
         * (у сцены, созданной от ссылки на робота, остаётся пустым)
         */
        std::vector<std::shared_ptr<Scene>> _singleRobotScenes;
        /**
         * список флагов, имеет ли тот или иной объект звенья
         */
        std::vector<bool> _areObjectsJointed;
        /**
         * список индексов всех объектов, не имеющих звенья
         */
        std::vector<int> _notJointedObjectIndexes;
        /**
         * список индексов всех объектов, имеющих звенья
         */
        std::vector<int> _jointedObjectIndexes;
    };


}