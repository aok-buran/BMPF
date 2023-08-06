#pragma once


#include <GL/glut.h>
#include <vector>
#include <memory>
#include <fstream>
#include "MT_Point3.h"
#include "SOLID.h"

namespace bmpf {

    /**
     * Класс STL модели
     */
    class StlShape {
    public:
        /**
         * Получить STL-модель из файла
         * @param path - путь к файлу модели
         */
        static std::shared_ptr<StlShape> fromStlFile(const std::string &path);

        /**
         * Прочитать точки STL-модели из файла
         *
         * @param path - путь к файлу модели
         * @return список координат полигона вектор нормали и координаты вершин:
         * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
         */
        static std::vector<float> readStl(const std::string &path);

        /**
         * Конструктор
         * @param points - список вершин модели
         */
        explicit StlShape(std::vector<float> points);

        /**
         * Деструктор
         */
        ~StlShape();

        /**
         * Рисование модели средствами OpenGl
         */
        void paintGL() const;

        /**
         * Получить список вершин модели
         * @return  список вершин модели
         */
        std::vector<float> &getPointList() { return _pointsList; }

        /**
         * Получить поверхность модели
         * @return поверхность модели
         */
        DT_ShapeHandle getDTShape() const { return _dtShape; }

        /**
         * Получить количество полигонов
         * @return количество полигонов
         */
        unsigned int getPolygonCnt() const { return _polygonCnt; }

    private:
        /**
         * Получить координаты из сырых данных STL-файла
         * @param data - сырые данные
         * @return список вершин модели (x, y, z)
         */
        static std::vector<float> getCoords(char *data);

        /**
         * Модель из библиотеки Solid3
         */
        DT_ShapeHandle _dtShape{};
        /**
         * Список точек solid3 (координаты x, y, z)
         */
        MT_Point3 *_points;
        /**
         * Количество полигонов модели
         */
        unsigned int _polygonCnt;
        /**
         * GПервый полигон модели из библиотеки Solid3
         */
        DT_VertexBaseHandle _base;
        /**
         * список координат полигона: вектор нормали и координаты вершин:
         * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
         */
        std::vector<float> _pointsList;
    };
}