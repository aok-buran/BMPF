#pragma once


#include <MT_Quaternion.h>
#include <vector>
#include <map>
#include <utility>
#include <thread>
#include <memory>
#include <mutex>

#include "base/collider.h"
#include "log.h"

namespace bmpf {
/**
 * @brief Класс однопоточного коллайдера
 * Класс однопоточного коллайдера.
 * Класс работает по следующему принципу: сначала мы задаём
 * состояние кооллайдеру, а потом либо запускаем его рисование, либо
 * проверку коллизий, одновременно эти два дейсвтвия выполняться не могут.
 * При этом он оперирует только 3D объектами и их матрицами преобразований
 */
    class SolidCollider : public Collider {
    public:

        /**
         * Конструктор по умолчанию
         */
        SolidCollider() = default;

        /**
         * Деструктор
         */
        virtual ~SolidCollider();

        /**
         * инициализация коллайдера
         *
         * @param groupedModelPaths вектор векторов путей к моделям. это связано с тем,
         * что сцена строится на основе набора описаний роботов
         * каждый из них содержит список путей к своим моделям
         * @param subColliders флаг, нудно ли составить виртуальные сцены для
         * каждого подмножества объектов сцены
         */
        void init(std::vector<std::vector<std::string>> groupedModelPaths, bool subCollider) override;

        /**
         * рисование сцены коллайдера
         * @param matrices список матриц преобразования из текущей
         * СК звена в следующую
         * @param onlyRobot флаг, нужно ли рисовать только роботов
         * или ещё и статические объекты сцены
         */
        void paint(std::vector<Eigen::Matrix4d> matrices, bool onlyRobot) override;

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
        std::vector<float> getPoints(std::vector<Eigen::Matrix4d> matrices) override;

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
        std::vector<double> getBoxCoords(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) override;

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
        std::vector<double> getBoxPoints(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices) override;


    private:


        /**
         * получить подвектор списков моделей объектов по вектору индексов
         * @param groupedModelPaths вектор векторов путей к моделям. это связано с тем,
         * что сцена строится на основе набора описаний роботов
         * каждый из них содержит список путей к своим моделям
         * @param indexes индексы роботов, списки моделей которых нужно использовать
         * @return подвектор списков моделей объектов
         */
        static std::vector<std::vector<std::string>> _getSubGroupedModelPaths(
                const std::vector<std::vector<std::string>> &groupedModelPaths, const std::vector<int> &indexes
        );

        /**
         * @brief заполнить словарь коллайдеров подгрупп объектов
         * заполнить словарь коллайдеров подгрупп объектов, запускает рекуррентную функцию
         * _populateColliderMap()
         * @param groupedModelPaths  вектор векторов путей к моделям. это связано с тем,
         * что сцена строится на основе набора описаний роботов
         * каждый из них содержит список путей к своим моделям
         */
        void _populateColliderMap(const std::vector<std::vector<std::string>> &groupedModelPaths) {
            _populateColliderMap(groupedModelPaths, {}, 0, 0);
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
        void _populateColliderMap(
                const std::vector<std::vector<std::string>> &groupedModelPaths,
                std::vector<int> indexes, int pos, int val
        );


        /**
         * освобождение блокирующего мьютекса
         */
        void _makeFree() { _setTransformMutex.unlock(); };

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
        void _setTransformMatrices(unsigned long robotNum, std::vector<Eigen::Matrix4d> matrices);

        /**
         * задать матрицы преобразования, библиотека solid построена так,
         * что нужно сначала задать матрицы преобразования, а потом
         * выполнить то или иное действие: нарисовать, проверить коллизии и т.д.
         * поэтому метод в начале пытается заблокировать мьютекс _setTransformMutex
         * с заданным интервалом, когда закончите использование текущее состояние сцены
         * НЕ ЗАБУДЬТЕ ВЫЗВАТЬ МЕТОД _makeFree()
         * @param matrices матрицы преобразования
         */
        void _setTransformMatrices(std::vector<Eigen::Matrix4d> matrices);

        /**
         *  порождает массив из 16 элементов из значений матрицы m
         * (после использования не забудьте освободить память)
         * @param m матрица преобразования
         * @return массив из 16 элементов из значений матрицы
         */
        static double *_eigenToDouble(Eigen::Matrix4d m);

        /**
         * проверка, соответствует ли коллизии текущее состояние сцены
         * @return флаг, соответствует ли коллизии текущее состояние сцены
         */
        bool _isCollided();

        /**
         * флаг, состоит ли система из одного робота,
         * в этом случае по-другому выполняется проверка коллизий
         */
        bool _isSingleObject{};

        /**
         * диапазоны индексов сочленений для каждого из роботов
         */
        std::vector<std::pair<long, long>> _objectIndexRanges;

        /**
         * сцена solid3
         */
        DT_SceneHandle _scene{};
        /**
         * список 3d объектов каждого звена
         */
        std::vector<std::shared_ptr<Solid3Object>> _links;
        /**
         * список 3d объектов каждого звена, сгруппированный по роботам
         * чтобы получить список моделей i-го робота, нужно обратиться
         * к i-му элементу этого списка
         */
        std::vector<std::vector<std::shared_ptr<Solid3Object>>> _groupedLinks;
        /**
         * мьютекс для упорядочивания доступа к данным 3D объекта
         */
        std::mutex _setTransformMutex;
        /**
         * словарь соответствий наборов индексов роботов и сцен,
         * построенных на этом наборе
         */
        std::map<std::vector<int>, std::shared_ptr<SolidCollider>> _collidersMap;

    };
}