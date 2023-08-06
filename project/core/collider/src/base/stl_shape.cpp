#include "base/stl_shape.h"

using namespace bmpf;

/**
 * Конструктор
 * @param points - список вершин модели
 */
StlShape::StlShape(std::vector<float> points) {
    _pointsList = points;
    _polygonCnt = points.size() / 12;
    _points = new MT_Point3[_polygonCnt * 3];
    // заполняем список точек
    for (unsigned int i = 0; i < _polygonCnt; i++) {
        DT_Vector3 point1{points.at(i * 12 + 3), points.at(i * 12 + 4), points.at(i * 12 + 5)};
        DT_Vector3 point2{points.at(i * 12 + 6), points.at(i * 12 + 7), points.at(i * 12 + 8)};
        DT_Vector3 point3{points.at(i * 12 + 9), points.at(i * 12 + 10), points.at(i * 12 + 11)};
        _points[i * 3].setValue(point1);
        _points[i * 3 + 1].setValue(point2);
        _points[i * 3 + 2].setValue(point3);
    }
    // сохраняем ссылку на первую точку
    _base = DT_NewVertexBase(_points, 0);
    // создаём модель solid3
    _dtShape = DT_NewComplexShape(_base);
    for (unsigned int i = 0; i < _polygonCnt; i++) {
        DT_Begin();
        // перечисляем полигоны
        DT_VertexIndex(i * 3);
        DT_VertexIndex(i * 3 + 1);
        DT_VertexIndex(i * 3 + 2);
        DT_End();
    }
    // завершаем формирование модели
    DT_EndComplexShape();
}

/**
 * Прочитать точки STL-модели из файла
 *
 * @param path - путь к файлу модели
 * @return список координат полигона вектор нормали и координаты вершин:
 * nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz по списку матриц состояния
 */
std::vector<float> StlShape::readStl(const std::string &path) {
    // открываем файл
    std::ifstream myFile(path.c_str(), std::ios::in | std::ios::binary);

    // читаем заголовок файла, размером 80 байт
    char header_info[80] = "";
    if (myFile)
        myFile.read(header_info, 80);
    else
        throw std::invalid_argument("header error: " + path);

    // читаем количество полигонов (4 байта беззнаковое целое)
    unsigned int nTriLong;
    if (myFile) {
        char nTri[4] = {0};
        myFile.read(nTri, 4);
        nTriLong = *((unsigned int *) nTri);
    } else
        throw std::invalid_argument("read polygon cnt error: " + path);

    // итоговый список точек
    std::vector<float> points;

    // читаем сами точки полигона
    for (int i = 0; i < nTriLong; i++) {
        // буффер, в который читаются значения
        char facet[50];
        // если с файлом всё в порядке
        if (myFile) {
            // читаем данные координат одного треугольника (50 байт)
            myFile.read(facet, 50);
            // добавляем в список координаты из данных
            for (float coord: getCoords(facet))
                points.push_back(coord);
            for (float coord: getCoords(facet + 12))
                points.push_back(coord);
            for (float coord: getCoords(facet + 24))
                points.push_back(coord);
            for (float coord: getCoords(facet + 36))
                points.push_back(coord);
        } else
            throw std::invalid_argument("read polygon coords error: " + path);
    }
    return points;
}

/**
 * Получить координаты из сырых данных STL-файла
 * @param data - сырые данные
 * @return список вершин модели (x, y, z)
 */
std::vector<float> StlShape::getCoords(char *data) {
    char f1[4] = {data[0],
                  data[1], data[2], data[3]};

    char f2[4] = {data[4],
                  data[5], data[6], data[7]};

    char f3[4] = {data[8],
                  data[9], data[10], data[11]};

    float x = *((float *) f1); // x
    float y = *((float *) f2); // y
    float z = *((float *) f3); // z
    std::vector<float> point{x, y, z};
    return point;
};

/**
 * Получить STL-модель из файла
 * @param path - путь к файлу модели
 */
std::shared_ptr<StlShape> StlShape::fromStlFile(const std::string &path) {
    std::vector<float> points = readStl(path);
    return std::make_shared<StlShape>(points);
}

/**
 * Деструктор
 */
StlShape::~StlShape() {
    DT_DeleteVertexBase(_base);
    DT_DeleteShape(_dtShape);
    delete[] _points;
}

/**
 * Рисование модели средствами OpenGl
 */
void StlShape::paintGL() const {
    glBegin(GL_TRIANGLES);
    for (DT_Index i = 0; i < _polygonCnt * 3; i++)
        glVertex3fv(_points[i]);
    glEnd();
}