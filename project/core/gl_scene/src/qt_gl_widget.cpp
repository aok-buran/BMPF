#include <QCoreApplication>
#include "qt_gl_widget.h"
#include "log.h"

using namespace bmpf;

/**
 * Конструктор виджета
 * @param parent родительский виджет
 */
QTGLWidget::QTGLWidget(QWidget *parent) :
        QOpenGLWidget(parent),
        xAngle(0), yAngle(0), zAngle(0), _program(nullptr) {
    _parent = parent;

    // Проверяем, поддерживается ли прозрачность
    _isTransparent = QCoreApplication::arguments().contains(QStringLiteral("--transparent"));
    if (_isTransparent)
        // если поддерживается, то задействуем её
        setAttribute(Qt::WA_TranslucentBackground);

    // проверяем, используется ли профиль ядра
    _core = QCoreApplication::arguments().contains(QStringLiteral("--coreprofile"));
}


/**
 * Очистить виджет
 */
void QTGLWidget::cleanup() {
    makeCurrent();
    delete _program;
    _program = nullptr;
    doneCurrent();
}

/**
 * Инициализация OpenGL
 */
void QTGLWidget::initializeGL() {
    // Соответствующее окно верхнего уровня виджета может измениться
    // несколько раз в течение жизни виджета. Всякий раз, когда это происходит,
    // связанный с QOpenGLWidget контекст уничтожается и создается новый.
    // Поэтому мы должны быть готовы очистить ресурсы на
    // сигнал aboutToBeDestroyed() вместо деструктора. Эмиссия
    // сигнал будет сопровождаться вызовом initializeGL(), где мы
    // можем воссоздать все ресурсы.
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &QTGLWidget::cleanup);

    initializeOpenGLFunctions();

    glClearColor(0.06, 0.08, 0.09, _isTransparent ? 0 : 1);


    _program = new QOpenGLShaderProgram;
    _program->addShaderFromSourceCode(QOpenGLShader::Vertex, _core ? vertexShaderSourceCore : vertexShaderSource);
    _program->addShaderFromSourceCode(QOpenGLShader::Fragment, _core ? fragmentShaderSourceCore : fragmentShaderSource);

    _program->bindAttributeLocation("vertex", 0);
    _program->bindAttributeLocation("normal", 1);
    _program->link();

    _program->bind();


    _projMatrixLoc = _program->uniformLocation("projMatrix");
    _mvMatrixLoc = _program->uniformLocation("mvMatrix");
    _normalMatrixLoc = _program->uniformLocation("normalMatrix");
    _lightPosLoc = _program->uniformLocation("lightPos");

    init();

    // Our _camera never changes in this example.
    _camera.setToIdentity();
    _camera.translate(0, 0, -1);
    _camera.rotate(180, 0, 1, 0);

    // Light position is fixed.
    _program->setUniformValue(_lightPosLoc, QVector3D(0, 0, 70));

    _program->release();

}

/**
 * Рисование OPenGL
 */
void QTGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    _worldTransform.setToIdentity();
    _worldTransform.rotate(180.0f - (xAngle / 16.0f), 1, 0, 0);
    _worldTransform.rotate(yAngle / 16.0f, 0, 1, 0);
    _worldTransform.rotate(180.0f + zAngle / 16.0f, 0, 0, 1);

    paint();

    _parent->setWindowTitle(_caption.c_str());
    _parent->update();

    _program->release();
}

/**
 * Получить минимальный размер окна
 * @return минимальный размер окна
 */
QSize QTGLWidget::minimumSizeHint() const {
    return {50, 50};
}

/**
 * Получить максимальный размер окна
 * @return максимальный размер окна
 */
QSize QTGLWidget::sizeHint() const {
    return {400, 400};
}

/**
 * Обработчик нажатия мыши
 * @param event событие
 */
void QTGLWidget::mousePressEvent(QMouseEvent *event) {
    _lastMousePos = event->pos();
}

/**
 * Обработчик перемещения мыши
 * @param event событие
 */
void QTGLWidget::mouseMoveEvent(QMouseEvent *event) {
    int dx = event->x() - _lastMousePos.x();
    int dy = event->y() - _lastMousePos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xAngle + 8 * dy);
        setYRotation(yAngle + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xAngle + 8 * dy);
        setZRotation(zAngle + 8 * dx);
    }
    _lastMousePos = event->pos();
}
