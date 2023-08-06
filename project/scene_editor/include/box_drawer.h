#pragma  once

#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <scene.h>
#include <base/path_finder.h>
#include "qt_gl_drawer.h"
#include "log.h"
#include <memory>

/**
 * Класс для рисования куба выбора
 */
class BoxDrawer : public bmpf::QTGLDrawer {
public:

    /**
     * Задать планировщик пути
     * @param robotNum номер робота
     * @param pathFinder планировщик
     * @param state начальное состояние
     */
    void setPathFinder(
            int robotNum, const std::shared_ptr<bmpf::PathFinder> &pathFinder,
            const std::vector<double> &state
    );

    /**
     * Задать состояние
     * @param robotNum номер робота
     * @param state состояние
     */
    void setState(int robotNum, const std::vector<double> &state);

    /**
     * Задать масштабирование
     * @param scale масштаб
     */
    void setScale(double scale);

    /**
     * Обновить состояние
     * @param robotNum номер робота
     */
    void updateState(int robotNum);

    /**
     * Получить последнее использованное состояние
     * @return последнее использованное состояние
     */
    std::vector<double> getLastState() { return _lastState; }

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
