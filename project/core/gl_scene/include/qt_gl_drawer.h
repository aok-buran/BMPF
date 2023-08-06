#pragma once


#include <qopengl.h>
#include <QVector>
#include <QVector3D>

namespace bmpf {

    /**
     * Базовый класс для рисования объекта OpenGL средствами QT
     */
    class QTGLDrawer {
    public:

        /**
         * Конструктор по умолчанию
         */
        QTGLDrawer() : m_count(0) {}

        /**
         * Получить данные о меше
         * @return данные о меше
         */
        const GLfloat *constData() const { return mData.constData(); }

        /**
         * Получить количество вершин
         * @return  количество вершин
         */
        int count() const { return m_count; }

        /**
         * Получить количество полигонов
         * @return количество полигонов
         */
        int vertexCount() const { return m_count / 6; }

    protected:
        /**
         * Метод добавления вершины
         * @param v положение вершины
         * @param n нормаль к этой вершине
         */
        void add(const QVector3D &v, const QVector3D &n);

        /**
         * Данные о меше объекта
         */
        QVector<GLfloat> mData;
        /**
         * Размер данных
         */
        int m_count;
    };

}