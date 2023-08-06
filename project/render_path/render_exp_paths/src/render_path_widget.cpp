#include <render_path_widget.h>

#include <solid_collider.h>

#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <QtCore/QTimer>
#include <QtWidgets/QSlider>
#include <iostream>

/**
 * Конструктор виджета
 * @param parent родительский виджет
 */
RenderPathWidget::RenderPathWidget(QSlider *slider, QWidget *parent) :
        QTGLWidget(parent) {
    _flgPlay = false;
    _slider = slider;

    auto *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    _tm = 0;
    timer->start(_DELAY_MS);

    _expCnt = 0;
    _curExpNum = 0;
}

/**
 * Инициализация
 */
void RenderPathWidget::init() {
    // Создаём объект массива вершин. В OpenGL ES 2.0 и OpenGL 2.x
    // реализации это необязательно, и поддержка может отсутствовать
    // совсем. Тем не менее приведенный ниже код работает во всех случаях
    _vao.create();
    QOpenGLVertexArrayObject::Binder vaoBinder(&_vao);

    // настраиваем буфер вершин
    _sceneVertexBuffer.create();
    _sceneVertexBuffer.bind();
    _sceneVertexBuffer.allocate(_drawer.constData(), _drawer.count() * sizeof(GLfloat));

    _sceneVertexBuffer.bind();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();


    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1,
                             3,
                             GL_FLOAT,
                             GL_FALSE,
                             6 * sizeof(GLfloat),
                             reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    _sceneVertexBuffer.release();

}


void RenderPathWidget::paint() {

    QOpenGLVertexArrayObject::Binder vaoBinder(&_vao);
    _program->bind();
    _program->setUniformValue(_projMatrixLoc, _proj);
    _program->setUniformValue(_mvMatrixLoc, _camera * _worldTransform);
    QMatrix3x3 normalMatrix = _worldTransform.normalMatrix();
    _program->setUniformValue(_normalMatrixLoc, normalMatrix);

    glDrawArrays(GL_TRIANGLES, 0, _drawer.vertexCount());

    char buf[256];
    sprintf(buf, "ExpNum %lu: ", _curExpNum);
    _caption = buf;
    if (!_state.empty())
        _caption += _pathFinder->checkCollision(_state) ? "Collided" : "No collision";

}

void RenderPathWidget::resizeGL(int w, int h) {
    _proj.setToIdentity();
    _proj.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

/**
 * Задать планировщик пути
 * @param pathFinder планировщик пути
 */
void RenderPathWidget::setPathFinder(const std::shared_ptr<bmpf::PathFinder> &pr,
                                     std::vector<std::vector<std::vector<double>>> paths) {
    _drawer.setPathFinder(pr);

    if (!pr)
        throw std::invalid_argument("RenderPathWidget::setPathFinder() ERROR: path finder is null");

    if (!pr->getScene())
        throw std::invalid_argument("RenderPathWidget::setPathFinder() ERROR: scene is null");

    bmpf::infoMsg(pr->getScene()->getActiveRobotCnt(), " ",
                  pr->getScene()->getJointCnt());

    _pathFinder = pr;
    _expCnt = paths.size();
    _paths = paths;

    std::vector<double> state = pr->getScene()->getRandomState();

    std::vector<std::vector<double>> curPath = paths.at(0);
    auto statePR = bmpf::PathFinder::getPathStateFromTM(curPath, 0);
    _drawer.setState(statePR);
}

/**
 * Применить изменения
 */
void RenderPathWidget::_setChanges(double tm) {
    std::vector<std::vector<double>> curPath = _paths.at(_curExpNum);
    std::vector<double> actualState = bmpf::PathFinder::getPathStateFromTM(curPath, tm);

    _state = actualState;
    _drawer.setState(actualState);

    _sceneVertexBuffer.bind();
    _sceneVertexBuffer.allocate(_drawer.constData(), _drawer.count() * sizeof(GLfloat));
    _sceneVertexBuffer.release();

    update();
}

/**
 * Таймер
 */
void RenderPathWidget::onTimer() {
    if (_flgPlay) {
        assert(_pathFinder);

        _setChanges(_tm);

        _tm += 1.0 / _DELAY_MS;
        if (_tm >= _paths.at(_curExpNum).size())
            _tm = 0;

        _slider->setValue((int) (_tm * 1000));
        _slider->update();
    }
}

/**
 * Задать флаг прохождения пути
 * @param flgPlay флаг прохождения пути
 */
void RenderPathWidget::setFlgPlay(bool flgPlay) {
    _flgPlay = flgPlay;
}

/**
 * Предыдущий эксперимент
 */
void RenderPathWidget::prev() {
    if (_curExpNum == 0) {
        _curExpNum = _expCnt - 1;
    } else
        _curExpNum--;
    _makeChangeExpNum();
}
/**
 * Следующий эксперимент
 */
void RenderPathWidget::next() {
    _curExpNum++;
    if (_curExpNum >= _expCnt)
        _curExpNum = 0;
    _makeChangeExpNum();
}
/**
 * Обработка изменения номера эксперимента
 */
void RenderPathWidget::_makeChangeExpNum() {
    _tm = 0;
    _slider->setMinimum(0);
    _slider->setValue(0);
    _slider->setMaximum(_paths.at(_curExpNum).size() * 1000);

    _slider->update();

    std::vector<std::vector<double>> curPath = _paths.at(_curExpNum);
    std::vector<double> statePR = bmpf::PathFinder::getPathStateFromTM(curPath, 0);

    _drawer.setState(statePR);
    _setChanges(0);
}

void RenderPathWidget::setTime(int angle) {
    if ((double) angle / 1000 != _tm) {
        _tm = (double) angle / 1000;
        _setChanges(_tm);
        emit timeChanged(angle);
        update();
    }
}

void RenderPathWidget::setXRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != xAngle) {
        xAngle = angle;
        emit xRotationChanged(angle);
        update();
    }
}

void RenderPathWidget::setYRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != yAngle) {
        yAngle = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void RenderPathWidget::setZRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != zAngle) {
        zAngle = angle;
        emit zRotationChanged(angle);
        update();
    }
}

/**
 * Освободить используемые ресурсы
 */
void RenderPathWidget::cleanup() {
    bmpf::QTGLWidget::cleanup();
    _sceneVertexBuffer.destroy();
}
