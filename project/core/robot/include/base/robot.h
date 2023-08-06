#pragma once

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <utility>

#include "log.h"
#include "matrix_math.h"
#include "link.h"
#include "joint_params.h"
#include "joint.h"

namespace bmpf {
/**
 * Базовый класс робота
 */
    class BaseRobot {
    public:
        /**
         * Виртуальный деструктор (по умолчанию)
         */
        virtual ~BaseRobot() = default;

        /**
         * Конструктор
         */
        BaseRobot();

        /**
         * @brief загрузить параметры робота из файла
         *
         * Загрузить параметры робота из файла, нужно заполнить следующие поля:
         * _jointParams, _links, _nonHierarchicalLinks, path
         * @param path путь к файлу описания робота
         */
        virtual void loadFromFile(std::string path) = 0;

        /**
         * получить список осей вращения сочленений робота в СК мира по состоянию
         * @param state состояние
         * @return список осей вращения сочленений робота
         */
        virtual std::vector<Eigen::Vector3d> getJointAxes(std::vector<double> state);

        /**
         * получить список матриц преобразований всех звеньев по состоянию
         * @param state состояние
         * @return список матриц преобразований
         */
        virtual std::vector<Eigen::Matrix4d> getTransformMatrices(std::vector<double> state);

        /**
         * получить матрицу преобразования из СК базы робота в СК рабочего инструмента по состоянию
         * @param state  состояние
         * @return матрица преобразования
         */
        virtual Eigen::Matrix4d getEndEffectorTransformMatrix(std::vector<double> state);

        /**
         * Получить частную производную по i-ой координате матрицы преобразования
         * из СК базы робота в СК рабочего инструмента
         *
         * @param state состояние
         * @param iVal индекс координаты, по которой берётся производная
         * @return частная производная по i-ой координате
         */
        Eigen::Matrix4d getEndEffectorDiffTransformMatrix(std::vector<double> state, int iVal);

        /**
         * Получить частную производную по i-ой и j-ой координатам матрицы преобразования
         * из СК базы робота в СК рабочего инструмента
         *
         * @param state состояние
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @return частная производная по i-ой и j-ой координатам
         */
        Eigen::Matrix4d getEndEffectorDiff2TransformMatrix(std::vector<double> state, int iVal, int jVal);

        /**
         * проверка допустимости углов поворота сочленения (рассчитывается по ограничениям сочленений)
         * @param state состояние
         * @return флаг, допустим ли соответствующий набор углов поворота (конфигурация)
         */
        bool isStateEnabled(std::vector<double> state);

        /**
         * получить смещение из СК мира в СК робота
         * @return смещение из СК мира в СК робота
         */
        std::vector<double> getWorldTranslation() const {
            return {_worldTransformVector.begin(), _worldTransformVector.begin() + 3};
        }

        /**
         * получить поворот из СК мира в СК робота
         * @return поворот из СК мира в СК робота
         */
        std::vector<double> getWorldRotation() const {
            return {_worldTransformVector.begin() + 3, _worldTransformVector.begin() + 6};
        }

        /**
         * получить масштабирование из СК мира в СК робота
         * @return масштабирование из СК мира в СК робота
         */
        std::vector<double> getWorldScale() const {
            return {_worldTransformVector.begin() + 6, _worldTransformVector.begin() + 9};
        }

        /**
         * получить вектор перехода из СК мира в СК робота: x,y,z; rotation: r,p,y; scale: x,y,z
         * @return вектор перехода из СК мира в СК робота
         */
        std::vector<double> getWorldTransformVector() const { return _worldTransformVector; }

        /**
         * получить матрицу перехода из СК мира в СК робота
         * @return матрица перехода из СК мира в СК робота
         */
        const std::shared_ptr<Eigen::Matrix4d> &getWorldTransformMatrix() const { return _worldTransformMatrix; }

        /**
         * получить кол-во сочленений робота
         * @return
         */
        unsigned long getJointCnt() const { return _jointParams.size(); }


        /**
         * получить максимальные по модулю значения скорости каждой из обобщённых координат
         * @return максимальные по модулю значения скорости
         */
        std::vector<double> getMaxSpeedModules() { return _maxSpeedModules; }

        /**
        * получить максимальные по модулю значения ускорения каждой из обобщённых координат
        * @return максимальные по модулю значения ускорения
        */
        std::vector<double> getMaxAccelerationModules() { return _maxAccelerationModules; }

        /**
         * получить количество звеньев робота
         * @return
         */
        unsigned long getLinkCnt() const { return _links.size(); }

        /**
         * получить количество параметров сочленений робота
         * @return
         */
        std::vector<std::shared_ptr<JointParams>> getJointParamsList() const { return _jointParams; }

        /**
         * получить список звеньев робота
         * @return список звеньев робота
         */
        std::vector<std::shared_ptr<Link>> getLinks() const { return _links; }

        /**
         * получить путь к описанию робота
         * @return путь к описанию робота
         */
        std::string getPath() const { return _path; }

        /**
         * задать смещение из СК мира в СК робота
         * @param translation смещение
         */
        void setWorldTranslation(const std::vector<double> &translation);

        /**
         * задать поворот из СК мира в СК робота
         * @param rotation поворот
         */
        void setWorldRotation(const std::vector<double> &rotation);

        /**
         * задать масштабирование из СК мира в СК робота
         * @param scale масштабирование
         */
        void setWorldScale(const std::vector<double> &scale);

        /**
         * задать вектор перехода из СК мира в СК робота: x,y,z; rotation: r,p,y; scale: x,y,z
         * @param vec  вектор перехода
         */
        void setWorldTransformVector(const std::vector<double> &vec);

        /**
         * получить список моделей, используемых роботом
         * @return список моделей, используемых роботом
         */
        std::vector<std::string> getModelPaths() const;

        /**
         * получить положение рабочего инструмента робота
         * @param state состояние
         * @return положение рабочего инструмента
         */
        std::vector<double> getEndEffectorPos(std::vector<double> state);

        /**
         * получить частную производную положения энд-эффектора робота по i-ой координате
         * @param state состояние
         * @param iVal индекс координаты, по которой берётся производная
         * @return
         */
        std::vector<double> getEndEffectorDiffPos(std::vector<double> state, int iVal);

        /**
         * получить частную производную положения энд-эффектора робота по i-ой и j-ой координатам
         * @param state состояние
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @return
         */
        std::vector<double> getEndEffectorDiff2Pos(std::vector<double> state, int iVal, int jVal);

        /**
         * получить положение и ориентацию(задана параметрами Родриго-Гамильтона)
         * @param state состояние
         * @return положение и ориентацию(задана параметрами Родриго-Гамильтона)
         */
        std::vector<double> getEndEffectorRGVector(std::vector<double> state);

        /**
         *  получить частную производную положения и ориентации(задана параметрами
         *  Родриго-Гамильтона) по i-ой координате
         * @param state состояние
         * @param iVal индекс координаты, по которой берётся производная
         * @return
         */
        std::vector<double> getEndEffectorDiffRGVector(std::vector<double> state, int iVal);

        /**
         * получить частную производную положения и ориентации(задана параметрами
         * Родриго-Гамильтона) по i-ой и j-ой координатам
         * @param state состояние
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @return
         */
        std::vector<double> getEndEffectorDiff2RGVector(std::vector<double> state, int iVal, int jVal);

        /**
         * получить положения всех звеньев
         * @param state  состояние
         * @return положения всех звеньев
         */
        std::vector<double> getAllLinkPositions(std::vector<double> state);

        /**
         * получить список положений(x,y,z) и ориентаций (параметры Родриго-Гамильтона) всех звеньев
         * @param state состояние
         * @return  положения всех звеньев
         */
        std::vector<double> getAllLinkRGVectors(std::vector<double> state);

        /**
         * получить список координат центров масс в СК базы
         * робота(x1, y1, z1, x2, y2, z2, ...)
         * @param state  состояние
         * @return  список координат центров масс
         */
        std::vector<double> getMassCenterAbsolutePositions(std::vector<double> state);

        /**
         * получить список координат центров масс в СК соответствующих
         * звеньев робота(x1, y1, z1, x2, y2, z2, ...)
         * @return список координат центров масс
         */
        virtual std::vector<double> getMassCenterLocalPositions();

        /**
         * получить список тензоров инерции
         * @return список тензоров инерции
         */
        virtual std::vector<Eigen::Matrix3d> getInertias();

        /**
         * получить случайное состояние
         * @return случайное состояние
         */
        std::vector<double> getRandomState();

    protected:
        /**
         * Заполнить матрицу перехода из СК мира в СК робота
         */
        void _fillWorldTransformMatrix();

        /**
         * перебор матриц преобразования из СК мира в СК соответствующего звена,
         * при состоянии робота равном state
         * @param state состояние
         * @param consumer обработчик для каждой матрицы
         */
        template<typename F>
        void _forEachJoint(std::vector<double> state, const F &consumer);

        /**
         * перебор матриц преобразования из СК мира в СК соответствующего звена,
         * при состоянии робота равном state (вместо i-ой матрицы преобразования берётся
         * её частная производная по i-ой координате)
         * @param state состояние
         * @param iVal индекс координаты, по которой берётся производная
         * @param consumer обработчик для каждой матрицы
         */
        template<typename F>
        void _forEachDiffJoint(std::vector<double> state, int iVal, const F &consumer);

        /**
         * перебор матриц преобразования из СК мира в СК соответствующего звена,
         * при состоянии робота равном state (вместо i-ой и j-ой матриц преобразования берутся
         * её частные производные по i-ой и j-ой координатам соответственно и вторая производная
         * по i-ой координате, если i==j)
         * @param state состояние
         * @param iVal индекс координаты, по которой первый раз берётся производная
         * @param jVal индекс координаты, по которой второй раз берётся производная
         * @param consumer обработчик для каждой матрицы
         */
        template<typename F>
        void _forEachDiff2Joint(std::vector<double> state, int iVal, int jVal, const F &consumer);

        /**
         * Сочленения робота
         */
        std::vector<std::shared_ptr<Joint>> _joints;
        /**
         * Параметры сочленений робота
         */
        std::vector<std::shared_ptr<JointParams>> _jointParams;
        /**
         * максимальные по модулю значения скорости каждой из координат
         */
        std::vector<double> _maxSpeedModules;
        /**
         * максимальные по модулю значения ускорения каждой из координат
         */
        std::vector<double> _maxAccelerationModules;
        /**
         * Звенья робота, сначала идут все звенья, участвующие в
         * иерархии, затем - неучаствующие
         */
        std::vector<std::shared_ptr<Link>> _links;
        /**
         * звенья, которые не участвуют в иерархии сцены
         */
        std::vector<std::shared_ptr<Link>> _nonHierarchicalLinks;
        /**
         * путь к описанию робота
         */
        std::string _path;
        /**
         * вектор перехода из СК мира в СК робота:
         * position: x,y,z; rotation: r,p,y; scale: x,y,z
         */
        std::vector<double> _worldTransformVector;
        /**
         * матрица перехода из СК мира в СК робота
         */
        std::shared_ptr<Eigen::Matrix4d> _worldTransformMatrix;

    };
}