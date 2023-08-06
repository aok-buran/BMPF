#include "render_single_path_drawer.h"
#include <qmath.h>
#include <memory>
#include <utility>


/**
 * Задать состояние
 * @param state состояние
 */
void RenderSinglePathDrawer::setState(std::vector<double> &state) {
    float scale = 0.3;

    assert(_pathFinder);

    auto matrices = _pathFinder->getScene()->getTransformMatrices(state);

    const std::vector<float> &points = _pathFinder->getCollider()->getPoints(matrices);

    m_count = 0;
    mData.clear();
    mData.resize(points.size() * 12);

    unsigned long triCnt = points.size() / 12;

    for (unsigned int i = 0; i < triCnt; i++) {
        QVector3D normal{points.at(i * 12) * scale, points.at(i * 12 + 1) * scale, points.at(i * 12 + 2) * scale};
        QVector3D point1{points.at(i * 12 + 3) * scale, points.at(i * 12 + 4) * scale, points.at(i * 12 + 5) * scale};
        QVector3D point2{points.at(i * 12 + 6) * scale, points.at(i * 12 + 7) * scale, points.at(i * 12 + 8) * scale};
        QVector3D point3{points.at(i * 12 + 9) * scale, points.at(i * 12 + 10) * scale, points.at(i * 12 + 11) * scale};
        add(point1, normal);
        add(point2, normal);
        add(point3, normal);
    }
}

/**
 * Задать планировщик пути
 * @param pathFinder планировщик
 */
void RenderSinglePathDrawer::setPathFinder(std::shared_ptr<bmpf::PathFinder> pathFinder) {
    _pathFinder = std::move(pathFinder);
}
