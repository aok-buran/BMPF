#include "base/solid_3d_object.h"

using namespace bmpf;

/**
 * загрузить объект из stl файла
 * @param path путь к модели
 * @param isRobot флаг, является ли объект частью робота
 * @param margin рамки объекта (отступы)
 * @return ссылку на новый объект
 */
std::shared_ptr<Solid3Object>
Solid3Object::fromStlFile(const std::string &path, bool isRobot, MT_Scalar margin) {
    std::shared_ptr<bmpf::StlShape> stlShape = bmpf::StlShape::fromStlFile(path);
    return std::make_shared<Solid3Object>(stlShape, isRobot, margin);
}

/**
 * рисование OpenGL
 * @param onlyRobot нужно ли рисовать только роботов
 */
void Solid3Object::paintGL(bool onlyRobot) const {
    // если рисуются все объекты или объект является роботом
    if (!onlyRobot || _isRobot) {
        // получаем матрицу преобразования объекта
        double m[16];
        DT_GetMatrixd(_object, m);
        // сохраняем текущую матрицу преобразования
        glPushMatrix();
        // домножаем текущую матрицу преобразования на матрицу преобразования о ъекта
        glMultMatrixd(m);
        // рисуем stl-модель
        _stl_shape->paintGL();
        // восстанавливаем матрицу
        glPopMatrix();
    }
}

/**
 * @brief возвращает список точек
 * возвращает список точек, к каждой применяется матрица преобразования
 * список координат полигона: вектор нормали и координаты вершин:
 * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
 * @return список точек
 */
std::vector<float> Solid3Object::getTransformedPointsList() {
    // получаем список координат полигонов stl-модели
    std::vector<float> shapePointList = _stl_shape->getPointList();
    // список преобразованных координат точек
    std::vector<float> transformedPointList;
    // перебираем полигоны
    for (unsigned int i = 0; i < _stl_shape->getPolygonCnt(); i++) {
        // получаем матрицу преобразования объекта
        double m[16];
        DT_GetMatrixd(_object, m);
        // переводим матрицу в Eigen
        Eigen::Matrix4d em(m);
        for (unsigned int j = 0; j < 4; j++) {
            // создаём 4-мерный вектор, чтобы работать с матрицей преобразования 4х4
            Eigen::Vector4d point
                    (shapePointList.at(i * 12 + 3 * j),
                     shapePointList.at(i * 12 + 3 * j + 1),
                     shapePointList.at(i * 12 + 3 * j + 2),
                     1);
            // получаем новые преобразованные координаты вектора
            point = em * point;
            // добавляем в список преобразованных точек
            for (int k = 0; k < 3; k++)
                transformedPointList.push_back((float) point[k]);
        }
    }
    return transformedPointList;
}