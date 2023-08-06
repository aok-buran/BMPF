#pragma once

#include <GL/glut.h>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "SOLID.h"
#include "MT_Scalar.h"
#include "MT_Point3.h"
#include "stl_shape.h"


namespace bmpf {
/**
 * @brief Класс 3D объектов
 * Класс 3D объектов, из которых строятся роботы
 * на сцене коллайдера. На каждое звено создаётся
 * по одному объекту Solid3Object
 */
    class Solid3Object {
    public:
        /**
         * конструктор по умолчанию
         */
        Solid3Object() = default;

        /**
         * запрещаем коструктор копии
         */
        Solid3Object(const Solid3Object &) = delete;

        /**
         *  запрещаем оператор присваивания
         * @return
         */
        Solid3Object &operator=(const Solid3Object &) = delete;

        /**
         * конструктор
         * @param shape  stl модель
         * @param isRobot  флаг, является ли объект частью робота
         * @param margin отступ
         */
        Solid3Object(const std::shared_ptr<bmpf::StlShape> &shape, bool isRobot, MT_Scalar margin = 0.0f)
                : _stl_shape(shape),
                  _object(DT_CreateObject(this, shape->getDTShape())),
                  _isRobot(isRobot) {
            DT_SetMargin(_object, margin);
        }

        /**
         *  деструктор
         */
        virtual ~Solid3Object() { DT_DestroyObject(_object); }

        /**
         * загрузить объект из stl файла
         * @param path путь к модели
         * @param isRobot флаг, является ли объект частью робота
         * @param margin рамки объекта (отступы)
         * @return ссылку на новый объект
         */
        static std::shared_ptr<Solid3Object>
        fromStlFile(const std::string &path, bool isRobot, MT_Scalar margin = 0.0f);

        /**
         * рисование OpenGL
         * @param onlyRobot нужно ли рисовать только роботов
         */
        void paintGL(bool onlyRobot = false) const;

        /**
         * Получить solid3-объект
         * @return solid3-объект
         */
        DT_ObjectHandle getHandle() const { return _object; }

        /**
         * @brief возвращает список точек
         * возвращает список точек, к каждой применяется матрица преобразования
         * список координат полигона: вектор нормали и координаты вершин:
         * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
         * @return список точек
         */
        std::vector<float> getTransformedPointsList();

    private:
        /**
         * флаг, является ли объект частью робота
         */
        bool _isRobot{};
        /**
         * модель stl
         */
        std::shared_ptr<bmpf::StlShape> _stl_shape;
        /**
         * solid3-объект
         */
        DT_ObjectHandle _object{};
    };
}