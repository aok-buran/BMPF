#pragma once

#include <mutex>
#include "solid_collider.h"


namespace bmpf {
/**
 * @brief Класс класс многопоточного коллайдера
 * Класс класс многопоточного коллайдера. Он является надстройкой
 * над массивом однопоточных коллайдеров заданной длины. Каждому
 * колладйеру соответствует свой мьютекс.
 * При обращении к методу проверки коллизий происходит
 * поиск первого незанятого мьютекса, он блокируется, запускается проверка коллизии
 * с помощью соответствующего коллайдера, после чего мьютекс освобождается.
 * Если все мьютексы заняты, то после паузы в одну микросекунду
 * поиск свободного мьютекса запускается заново, пока проверка не будет выполнена.
 * Остальные методы просто пробрасывают обращение к первому однопоточному коллайдеру
 */
    class SolidSyncCollider : public Collider {
    public:

        /**
         * максимальное количество потоков
         */
        const static int MAX_MUTES_CNT = 60;

        /**
         * конструктор
         * @param mutexCnt количество мьютексов
         */
        explicit SolidSyncCollider(unsigned int mutexCnt);

        /**
         * инициализация коллайдера
         *
         * @param groupedModelPaths вектор векторов путей к моделям. это связано с тем,
         * что сцена строится на основе набора описаний роботов
         * каждый из них содержит список путей к своим моделям
         * @param subColliders флаг, нудно ли составить виртуальные сцены для
         * каждого подмножества объектов сцены
         */
        void init(std::vector<std::vector<std::string>> groupedModelPaths, bool subColliders) override;

        /**
         * рисование сцены коллайдера
         * @param matrices список матриц преобразования из текущей
         * СК звена в следующую
         * @param onlyRobot флаг, нужно ли рисовать только роботов
         * или ещё и статические объекты сцены
         */
        void paint(std::vector<Eigen::Matrix4d> matrices, bool onlyRobot) override {
            _colliders.front()->paint(matrices, onlyRobot);
        }

        /**
         * проверка соответствует ли состояние сцены столкновению
         * @param matrices список матриц преобразований звеньев
         * @return флаг, соответствует ли состояние сцены столкновению
         */
        bool isCollided(std::vector<Eigen::Matrix4d> matrices) override;

        /**
         * возвращает список всех координат полигона (вектор нормали и координаты вершины):
         * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
         * @param matrices список матриц преобразований звеньев
         * @return возвращает список всех координат полигона
         */
        std::vector<float> getPoints(std::vector<Eigen::Matrix4d> matrices) override {
            return _colliders.front()->getPoints(matrices);
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
        std::vector<double> getBoxCoords(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) override {
            return _colliders.front()->getBoxCoords(robotNum, matrices);
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
        std::vector<double> getBoxPoints(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) override {
            return _colliders.front()->getBoxPoints(robotNum, matrices);
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
        bool isCollided(std::vector<Eigen::Matrix4d> matrices, std::vector<int> robotIndexes) override;

    private:
        // кол-во мьютексов
        unsigned int _mutexCnt{};
        // список коллайдеров
        std::vector<std::shared_ptr<SolidCollider>> _colliders;
        // массив мьютексов
        std::mutex _colliderMutexes[MAX_MUTES_CNT];

    };

}