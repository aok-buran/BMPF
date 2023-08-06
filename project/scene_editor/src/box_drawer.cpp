#include "box_drawer.h"


/**
 * Задать масштабирование
 * @param scale масштаб
 */
void BoxDrawer::setScale(double scale) {
    _scale = scale;
}

/**
 * Обновить состояние
 * @param robotNum номер робота
 */
void BoxDrawer::updateState(int robotNum) {

    if (_lastState.empty())
        _lastState = _pathFinder->getScene()->getRandomState();

    std::vector<Eigen::Matrix4d> matrices = _pathFinder->getScene()->getTransformMatrices(_lastState);

    _pathFinder->updateCollider();

    if (_pathFinder->getCollider())
        if (_pathFinder->getScene()->getGroupedTranslation().empty())
            return;


    const std::vector<double> points = _pathFinder->getCollider()->getBoxPoints(robotNum, matrices);


    m_count = 0;
    mData.resize(static_cast<int>(points.size() * 6));

    unsigned long triCnt = points.size() / 6;

    QVector3D normal{0, 0, 1};

    for (unsigned int i = 0; i < triCnt; i++) {
        QVector3D point1{static_cast<float>(points.at(i * 6) * _scale),
                         static_cast<float>(points.at(i * 6 + 1) * _scale),
                         static_cast<float>(points.at(i * 6 + 2) * _scale)};
        QVector3D point2{static_cast<float>(points.at(i * 6 + 3) * _scale),
                         static_cast<float>(points.at(i * 6 + 4) * _scale),
                         static_cast<float>(points.at(i * 6 + 5) * _scale)};
        add(point1, normal);
        add(point2, normal);
    }
}

/**
 * Задать масштабирование
 * @param scale масштаб
 */
void BoxDrawer::setState(int robotNum, const std::vector<double> &state) {
    _lastState = state;
    updateState(robotNum);
}

/**
 * Задать планировщик пути
 * @param robotNum номер робота
 * @param pathFinder планировщик
 * @param state начальное состояние
 */
void BoxDrawer::setPathFinder(
        int robotNum, const std::shared_ptr<bmpf::PathFinder> &pathFinder, const std::vector<double> &state
) {
    if (!pathFinder) {
        throw std::invalid_argument("BoxDrawer::setPathFinder() pathFinder is null");
    }
    if (!pathFinder->getScene()) {
        throw std::invalid_argument("BoxDrawer::setPathFinder() pathFinder scene is null");
    }

    _pathFinder = pathFinder;
    setState(robotNum, state);
}

