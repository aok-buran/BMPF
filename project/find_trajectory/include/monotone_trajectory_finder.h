#pragma once

#include "base/path_finder.h"
#include "continuous_path_finder.h"
#include "monotone_cubic_interpolation.h"
#include <scene.h>
#include <log.h>
#include "state.h"

namespace bmpf {
    /**
     * Планировщик монотонных траекторий
     */
    class MonotoneTrajectoryFinder {
    public:
        /**
         * инициализация
         * @param scene сцена
         * @param intervalDuration интервал между временными метками
         * @param showTrace флаг, выводить ли лог
         * @param maxOpenSetSize максимальный размер открытого множества
         * @param gridSize размер сетки
         * @param maxNodeCnt максимальное кол-во нод в закрытом множестве
         * @param checkCnt количество проверок каждого этапа пути
         * @param threadCnt количество потоков планировщика
         */
        virtual void init(const std::shared_ptr<bmpf::Scene> &scene, double intervalDuration,
                          bool showTrace,
                          unsigned int maxOpenSetSize, int gridSize, unsigned int maxNodeCnt, int checkCnt,
                          int threadCnt = 1);

        /**
         * Подготовка траектории
         * @param startState стартовое положение
         * @param endState конечное
         * @param errorCode код ошибки
         */
        virtual void prepareTrajectory(
                const std::vector<double> &startState, const std::vector<double> &endState, int &errorCode
        );

        /**
         * Деструктор
         */
        virtual ~MonotoneTrajectoryFinder() {
            _interpolators.clear();
        }

        /**
         * @brief получить все ноды траекторий
         * получить все ноды траекторий (первая координата время, потом положения,
         * потом скорости, потом ускорения, размерность каждого узла можно узнать из сцены
         * scene->getJointCnt()
         * @param tm время
         * @return все ноды траекторий
         */
        std::vector<double> getTrajectoryNode(double tm);

        /**
         * получить все положения траекторий (первая координата время, потом положения)
         * @param tm время
         * @return все положения траекторий
         */
        std::vector<double> getTrajectoryPosition(double tm);

        /**
         * получить все скорости траекторий (первая координата время, потом скорости)
         * @param tm время
         * @return все скорости траекторий
         */
        std::vector<double> getTrajectorySpeed(double tm);

        /**
         * получить все ускорения траекторий (первая координата время, потом ускорения)
         * @param tm время
         * @return все ускорения траекторий
         */
        std::vector<double> getTrajectoryAcceleration(double tm);


    protected:

        /**
         * Подготовка траектории
         * @param startState стартовое положение
         * @param endState конечное
         * @param timeIntervals интервалы времени
         * @param errorCode код ошибки
         */
        void _prepareTrajectory(
                const std::vector<double> &startState, const std::vector<double> &endState,
                std::vector<double> &timeIntervals, int &errorCode
        );

        /**
         * Интерполяторы для каждой из координат
         */
        std::vector<std::shared_ptr<MonotoneCubicInterpolation>> _interpolators;
        /**
         * Последний построенный путь
         */
        std::vector<std::vector<double>> _lastPath;
        /**
         * планировщик пути
         */
        std::shared_ptr<bmpf::PathFinder> _pf;
        /**
         * список временных интервалов
         */
        std::vector<double> _timeIntervals;
        /**
         * временной интервал между соседними точками траектории
         */
        double _startIntervalDuration{};
        /**
         * Последняя построенная траектория
         */
        std::vector<std::vector<double>> _lastTrajectory;
        /**
         * количество опорных точек траектории
         */
        int _nodeCnt{};
        /**
         * сцена
         */
        std::shared_ptr<bmpf::Scene> _scene;
        /**
         * общее время траектории
         */
        double _wholeDuration{};
        /**
         * флаг, готов ли планировщик
         */
        bool _isReady{};

    public:

        /**
         * Получить общее время траектории
         * @return общее время траектории
         */
        double getWholeDuration() const { return _wholeDuration; }

        /**
         * Получить последний построенный путь
         * @return последний построенный путь
         */
        const std::vector<std::vector<double>> &getLastPath() const { return _lastPath; }

        /**
         * Получить последняя построенная траектория
         * @return последнюю построенную траекторию
         */
        const std::vector<std::vector<double>> &getLastTrajectory() const { return _lastTrajectory; }

        /**
         * Получить количество опорных точек траектории
         * @return количество опорных точек траектории
         */
        int getNodeCnt() const { return _nodeCnt; }

        /**
         * Получить планировщик пути
         * @return планировщик пути
         */
        const std::shared_ptr<bmpf::PathFinder> &getPF() const { return _pf; }

        /**
         * Получить сцену
         * @return сцена
         */
        const std::shared_ptr<bmpf::Scene> &getScene() const { return _scene; }

        /**
         * Получить флаг, готов ли планировщик
         * @return флаг, готов ли планировщик
         */
        bool isReady() const { return _isReady; }

    };
}