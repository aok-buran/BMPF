#include "qt_gl_drawer.h"

/**
 * Метод добавления вершины
 * @param v положение вершины
 * @param n нормаль к этой вершине
 */
void bmpf::QTGLDrawer::add(const QVector3D &v, const QVector3D &n) {
    GLfloat *p = mData.data() + m_count;
    *p++ = v.x();
    *p++ = v.y();
    *p++ = v.z();
    *p++ = n.x();
    *p++ = n.y();
    *p++ = n.z();
    m_count += 6;
}
