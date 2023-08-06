
#include "scene_editor_drawer.h"


/**
 * Задать масштабирование
 * @param scale масштаб
 */
void SceneEditorDrawer::setScale(double scale) {
    _scale = scale;
}

/**
 * Получить случайное состояние
 */
void SceneEditorDrawer::setRandomState() {
    _lastState = _pathFinder->getScene()->getRandomState();
}

/**
 * Обновить состояние
 */
void SceneEditorDrawer::updateState() {
    if (_lastState.empty())
        setRandomState();

    if (_pathFinder->getScene()->getRobots().empty())
        return;

    std::vector<Eigen::Matrix4d> matrices = _pathFinder->getScene()->getTransformMatrices(_lastState);

    _pathFinder->updateCollider();

    const std::vector<float> &points = _pathFinder->getCollider()->getPoints(matrices);

    m_count = 0;
    mData.resize(static_cast<int>(points.size() * 6));

    unsigned long triCnt = points.size() / 12;

    for (unsigned int i = 0; i < triCnt; i++) {
        QVector3D normal{static_cast<float>(points.at(i * 12) * _scale),
                         static_cast<float>(points.at(i * 12 + 1) * _scale),
                         static_cast<float>(points.at(i * 12 + 2) * _scale)};
        QVector3D point1{static_cast<float>(points.at(i * 12 + 3) * _scale),
                         static_cast<float>(points.at(i * 12 + 4) * _scale),
                         static_cast<float>(points.at(i * 12 + 5) * _scale)};
        QVector3D point2{static_cast<float>(points.at(i * 12 + 6) * _scale),
                         static_cast<float>(points.at(i * 12 + 7) * _scale),
                         static_cast<float>(points.at(i * 12 + 8) * _scale)};
        QVector3D point3{static_cast<float>(points.at(i * 12 + 9) * _scale),
                         static_cast<float>(points.at(i * 12 + 10) * _scale),
                         static_cast<float>(points.at(i * 12 + 11) * _scale)};
        add(point1, normal);
        add(point2, normal);
        add(point3, normal);
    }

}

/**
 * Задать состояние
 * @param state состояние
 */
void SceneEditorDrawer::setState(const std::vector<double> &state) {
    _lastState = state;
    updateState();
}

/**
 * Задать планировщик пути
 * @param pathFinder планировщик
 */
void SceneEditorDrawer::setPathFinder(const std::shared_ptr<bmpf::PathFinder> &pathFinder) {
    if (!pathFinder) {
        throw std::invalid_argument("BoxDrawer::setPathFinder() pathFinder is null");
    }
    if (!pathFinder->getScene()) {
        throw std::invalid_argument("BoxDrawer::setPathFinder() pathFinder scene is null");
    }
    _pathFinder = pathFinder;
    updateState();
}

