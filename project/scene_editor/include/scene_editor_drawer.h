#pragma  once

#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <scene.h>
#include <base/path_finder.h>
#include "qt_gl_drawer.h"
#include <qmath.h>
#include <memory>
#include "log.h"


/**
 * Класс для рисования сцены
 */
class SceneEditorDrawer: public bmpf::QTGLDrawer{
public:

    /**
     * Задать планировщик пути
     * @param pathFinder планировщик
     */
    void setPathFinder(const std::shared_ptr<bmpf::PathFinder>& pathFinder);

    /**
     * Задать состояние
     * @param state состояние
     */
    void setState(const std::vector<double> &state);
    /**
     * Задать масштабирование
     * @param scale масштаб
     */
    void setScale(double scale);
    /**
     * Обновить состояние
     */
    void updateState();
    /**
     * Получить последнее использованное состояние
     * @return последнее использованное состояние
     */
    std::vector<double> getLastState() {return _lastState;}

    /**
     * Получить случайное состояние
     */
    void setRandomState();

private:
    /**
     * Планировщик
     */
    std::shared_ptr<bmpf::PathFinder> _pathFinder;
    /**
     * Масштаб
     */
    double _scale;
    /**
     * последнее использованное состояние
     */
    std::vector<double> _lastState;

};
