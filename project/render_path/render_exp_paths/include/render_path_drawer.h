#pragma  once

#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <scene.h>
#include <base/path_finder.h>
#include "qt_gl_drawer.h"

/**
 * Класс для рисования сцены
 */
class RenderPathDrawer : public bmpf::QTGLDrawer {
public:

    /**
     * Задать планировщик пути
     * @param pathFinder планировщик
     */
    void setPathFinder(std::shared_ptr<bmpf::PathFinder> pathFinder);

    /**
     * Задать состояние
     * @param state состояние
     */
    void setState(std::vector<double> &state);

private:

    /**
     * Планировщик
     */
    std::shared_ptr<bmpf::PathFinder> _pathFinder;

};
