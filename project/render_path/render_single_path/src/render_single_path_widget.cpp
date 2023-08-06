#include <render_single_path_widget.h>

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
RenderSinglePathWidget::RenderSinglePathWidget(QSlider *slider, QWidget *parent) :
        QTGLWidget(parent) {
    _flgPlay = false;
    _slider = slider;

    auto *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    _tm = 0;
    timer->start(_DELAY_MS);

}

/**
 * Инициализация
 */
void RenderSinglePathWidget::init() {
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


void RenderSinglePathWidget::paint() {

    QOpenGLVertexArrayObject::Binder vaoBinder(&_vao);
    _program->bind();
    _program->setUniformValue(_projMatrixLoc, _proj);
    _program->setUniformValue(_mvMatrixLoc, _camera * _worldTransform);
    QMatrix3x3 normalMatrix = _worldTransform.normalMatrix();
    _program->setUniformValue(_normalMatrixLoc, normalMatrix);

    glDrawArrays(GL_TRIANGLES, 0, _drawer.vertexCount());

}

void RenderSinglePathWidget::resizeGL(int w, int h) {
    _proj.setToIdentity();
    _proj.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

/**
 * Задать планировщик пути
 * @param pathFinder планировщик пути
 */
void RenderSinglePathWidget::setPathFinder(const std::shared_ptr<bmpf::PathFinder> &pr,
                                     std::vector<std::vector<double>> path) {
    _drawer.setPathFinder(pr);

    if (!pr)
        throw std::invalid_argument("RenderSinglePathWidget::setPathFinder() ERROR: path finder is null");

    if (!pr->getScene())
        throw std::invalid_argument("RenderSinglePathWidget::setPathFinder() ERROR: scene is null");

    bmpf::infoMsg(pr->getScene()->getActiveRobotCnt(), " ",
                  pr->getScene()->getJointCnt());

    _pathFinder = pr;
    _path = path;

    std::vector<double> state = pr->getScene()->getRandomState();

    auto statePR = bmpf::PathFinder::getPathStateFromTM(path, 0);
    _drawer.setState(statePR);
}

/**
 * Применить изменения
 */
void RenderSinglePathWidget::_setChanges(double tm) {
    std::vector<double> actualState = bmpf::PathFinder::getPathStateFromTM(_path, tm);

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
void RenderSinglePathWidget::onTimer() {
    if (_flgPlay) {
        assert(_pathFinder);

        _setChanges(_tm);

        _tm += 1.0 / _DELAY_MS;
        if (_tm >= _path.size())
            _tm = 0;

        _slider->setValue((int) (_tm * 1000));
        _slider->update();
    }
}

/**
 * Задать флаг прохождения пути
 * @param flgPlay флаг прохождения пути
 */
void RenderSinglePathWidget::setFlgPlay(bool flgPlay) {
    _flgPlay = flgPlay;
}

/**
 * Обработка изменения номера эксперимента
 */
void RenderSinglePathWidget::_makeChangeExpNum() {
    _tm = 0;
    _slider->setMinimum(0);
    _slider->setValue(0);
    _slider->setMaximum(_path.size() * 1000);

    _slider->update();

    std::vector<double> statePR = bmpf::PathFinder::getPathStateFromTM(_path, 0);

    _drawer.setState(statePR);
    _setChanges(0);
}

void RenderSinglePathWidget::setTime(int angle) {
    if ((double) angle / 1000 != _tm) {
        _tm = (double) angle / 1000;
        _setChanges(_tm);
        emit timeChanged(angle);
        update();
    }
}

void RenderSinglePathWidget::setXRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != xAngle) {
        xAngle = angle;
        emit xRotationChanged(angle);
        update();
    }
}

void RenderSinglePathWidget::setYRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != yAngle) {
        yAngle = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void RenderSinglePathWidget::setZRotation(int angle) {
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
void RenderSinglePathWidget::cleanup() {
    bmpf::QTGLWidget::cleanup();
    _sceneVertexBuffer.destroy();
}
