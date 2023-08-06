#include <scene_editor_widget.h>

#include <utility>


/**
 * Задать слайдеры
 * @param sliders слайдеры
 */
void SceneEditorWidget::setSliders(std::vector<QSlider *> sliders) {
    _sliders = std::move(sliders);
}


/**
 * Конструктор виджета
 * @param parent родительский виджет
 */
SceneEditorWidget::SceneEditorWidget(QWidget *parent) : QTGLWidget(parent) {
    actualRobotNum = 0;

    for (unsigned int i = 0; i < 9; i++)
        _sliderValues.push_back(0);

    _sceneDrawer.setScale(_sceneScale);
    _boxDrawer.setScale(_sceneScale);
}

/**
 * Освободить используемые ресурсы
 */
void SceneEditorWidget::cleanup() {
    QTGLWidget::cleanup();
    _sceneVertexBuffer.destroy();
    boxVertexBuffer.destroy();
}

/**
 * Инициализация
 */
void SceneEditorWidget::init() {
    // Создаём объект массива вершин. В OpenGL ES 2.0 и OpenGL 2.x
    // реализации это необязательно, и поддержка может отсутствовать
    // совсем. Тем не менее приведенный ниже код работает во всех случаях
    _sceneVAO.create();
    QOpenGLVertexArrayObject::Binder vaoBinder(&_sceneVAO);

    // настраиваем буфер вершин
    _sceneVertexBuffer.create();
    _sceneVertexBuffer.bind();
    _sceneVertexBuffer.allocate(_sceneDrawer.constData(), _sceneDrawer.count() * sizeof(GLfloat));

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

    // Создаём объект массива вершин. В OpenGL ES 2.0 и OpenGL 2.x
    // реализации это необязательно, и поддержка может отсутствовать
    // совсем. Тем не менее приведенный ниже код работает во всех случаях
    _boxVAO.create();
    QOpenGLVertexArrayObject::Binder vaoBinder2(&_boxVAO);

    // настраиваем VO буфер
    boxVertexBuffer.create();
    boxVertexBuffer.bind();
    boxVertexBuffer.allocate(_boxDrawer.constData(), _boxDrawer.count() * sizeof(GLfloat));


    boxVertexBuffer.bind();
    QOpenGLFunctions *f2 = QOpenGLContext::currentContext()->functions();
    f2->glEnableVertexAttribArray(0);
    f2->glEnableVertexAttribArray(1);
    f2->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);
    f2->glVertexAttribPointer(1,
                              3,
                              GL_FLOAT,
                              GL_FALSE,
                              6 * sizeof(GLfloat),
                              reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    boxVertexBuffer.release();
}

/**
 * Рисование
 */
void SceneEditorWidget::paint() {

    QOpenGLVertexArrayObject::Binder vaoBinder(&_sceneVAO);
    _program->bind();
    _program->setUniformValue(_projMatrixLoc, _proj);
    _program->setUniformValue(_mvMatrixLoc, _camera * _worldTransform);
    QMatrix3x3 normalMatrix = _worldTransform.normalMatrix();
    _program->setUniformValue(_normalMatrixLoc, normalMatrix);

    glDrawArrays(GL_TRIANGLES, 0, _sceneDrawer.vertexCount());


    QOpenGLVertexArrayObject::Binder vaoBinder2(&_boxVAO);
    _program->bind();
    _program->setUniformValue(_projMatrixLoc, _proj);
    _program->setUniformValue(_mvMatrixLoc, _camera * _worldTransform);
    QMatrix3x3 normalMatrix2 = _worldTransform.normalMatrix();
    _program->setUniformValue(_normalMatrixLoc, normalMatrix2);


    glDrawArrays(GL_LINES, 0, _boxDrawer.vertexCount());

    if (_groupedStartTranslation.empty())
        return;

    char buf[256];
    sprintf(
            buf, "pos: %4.2f %4.2f %6.2f;    rot: %4.2f %4.2f %4.2f;    scale: %4.2f %4.2f %4.2f",
            _groupedStartTranslation.at(actualRobotNum).at(0),
            _groupedStartTranslation.at(actualRobotNum).at(1),
            _groupedStartTranslation.at(actualRobotNum).at(2),
            _groupedStartRotation.at(actualRobotNum).at(0),
            _groupedStartRotation.at(actualRobotNum).at(1),
            _groupedStartRotation.at(actualRobotNum).at(2),
            _groupedStartScale.at(actualRobotNum).at(0),
            _groupedStartScale.at(actualRobotNum).at(1),
            _groupedStartScale.at(actualRobotNum).at(2)
    );
    _caption = buf;
}

/**
 * Изменить размер окна
 * @param width ширина
 * @param height высота
 */
void SceneEditorWidget::resizeGL(int w, int h) {
    _proj.setToIdentity();
    _proj.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

/**
 * Задать планировщик пути
 * @param pathFinder планировщик пути
 */
void SceneEditorWidget::setPathFinder(const std::shared_ptr<bmpf::PathFinder>& pathFinder) {
    if(!pathFinder) {
        throw std::runtime_error("SceneEditorWidget::_setChanges() ERROR: pathFinder is empty");
    }

    _pathFinder = pathFinder;

    std::vector<double> state = pathFinder->getScene()->getRandomState();

    _sceneDrawer.setPathFinder(_pathFinder);

    _sceneDrawer.setState(state);

    _modifySliders();

    update();
}

/**
 * Применить изменения
 */
void SceneEditorWidget::_setChanges() {

    if(!_pathFinder) {
        throw std::runtime_error("SceneEditorWidget::_setChanges() ERROR: pathFinder is empty");
    }

    _sceneDrawer.setPathFinder(_pathFinder);
    _boxDrawer.setPathFinder(actualRobotNum, _pathFinder, _sceneDrawer.getLastState());

    _sceneVertexBuffer.bind();
    _sceneVertexBuffer.allocate(_sceneDrawer.constData(), _sceneDrawer.count() * sizeof(GLfloat));
    _sceneVertexBuffer.release();

    boxVertexBuffer.bind();
    boxVertexBuffer.allocate(_boxDrawer.constData(), _boxDrawer.count() * sizeof(GLfloat));
    boxVertexBuffer.release();

    update();

}

/**
 * Применить значения слайдеров
 */
void SceneEditorWidget::_applySliders() {

    if (_groupedStartTranslation.empty())
        return;


    for (unsigned int i = 0; i < 3; i++) {
        _groupedStartTranslation.at(actualRobotNum).at(i) =
                _sliderToTranslation(_sliderValues.at(i));
        _groupedStartRotation.at(actualRobotNum).at(i) =
                _sliderToRotation(_sliderValues.at(i + 3));
        _groupedStartScale.at(actualRobotNum).at(i) =
                _sliderToScale(_sliderValues.at(i + 6));
    }

    _pathFinder->getScene()->setGroupedTranslation(_groupedStartTranslation);
    _pathFinder->getScene()->setGroupedRotation(_groupedStartRotation);
    _pathFinder->getScene()->setGroupedScale(_groupedStartScale);

    _setChanges();

}

/**
 * Обновить слайдеры по имеющимся значениям
 */
void SceneEditorWidget::_modifySliders() {
    if (_pathFinder->getScene()->getActiveRobotCnt() > 0) {

        _groupedStartTranslation = _pathFinder->getScene()->getGroupedTranslation();
        _groupedStartRotation = _pathFinder->getScene()->getGroupedRotation();
        _groupedStartScale = _pathFinder->getScene()->getGroupedScale();

        for (unsigned int i = 0; i < 3; i++) {
            _sliderValues.at(i) = _translationToSlider(
                    _groupedStartTranslation.at(actualRobotNum).at(i)
            );
            _sliderValues.at(i + 3) = _rotationToSlider(
                    _groupedStartRotation.at(actualRobotNum).at(i)
            );
            _sliderValues.at(i + 6) = _scaleToSlider(
                    _groupedStartScale.at(actualRobotNum).at(i)
            );
        }

        for (unsigned int i = 0; i < 9; i++) {
            _sliders.at(i)->setValue(_sliderValues.at(i));
            _sliders.at(i)->update();
        }
    }
}

/**
 * Добавить объект на сцену
 */
void SceneEditorWidget::doAdd(std::string path) {
    _pathFinder->addObjectToScene(std::move(path));
    _sceneDrawer.setRandomState();

    _modifySliders();
    _setChanges();
}

/**
 * Удалить объект со сцены
 */
void SceneEditorWidget::doDelete() {
    if (_pathFinder->getScene()->getActiveRobotCnt() > 0) {
        _pathFinder->deleteObjectFromScene(actualRobotNum);
        if (actualRobotNum >= _pathFinder->getScene()->getActiveRobotCnt()) {
            actualRobotNum = 0;
        }
        _setChanges();
        _modifySliders();
    }
}

/**
 * Выделить следующий объект
 */
void SceneEditorWidget::doPrev() {
    if (_pathFinder->getScene()->getActiveRobotCnt() > 0) {
        if (actualRobotNum == 0)
            actualRobotNum = _pathFinder->getScene()->getActiveRobotCnt() - 1;
        else
            actualRobotNum--;

        _modifySliders();
    }
}
/**
 * Выделить предыдущий объект
 */
void SceneEditorWidget::doNext() {
    if (_pathFinder->getScene()->getActiveRobotCnt() > 0) {
        actualRobotNum++;
        if (actualRobotNum >= _pathFinder->getScene()->getActiveRobotCnt())
            actualRobotNum = 0;

        _modifySliders();
    }
}


void SceneEditorWidget::setScrollTranslationX(int angle) {
    _sliderValues.at(0) = angle;
    _applySliders();
    emit setScrollTranslationXChanged(angle);

}

void SceneEditorWidget::setScrollTranslationY(int angle) {
    _sliderValues.at(1) = angle;
    _applySliders();
    emit setScrollTranslationYChanged(angle);

}

void SceneEditorWidget::setScrollTranslationZ(int angle) {
    _sliderValues.at(2) = angle;
    _applySliders();
    emit setScrollTranslationZChanged(angle);
}

void SceneEditorWidget::setScrollRotationX(int angle) {
    _sliderValues.at(3) = angle;
    _applySliders();
    emit setScrollRotationXChanged(angle);

}

void SceneEditorWidget::setScrollRotationY(int angle) {
    _sliderValues.at(4) = angle;
    _applySliders();
    emit setScrollRotationYChanged(angle);

}

void SceneEditorWidget::setScrollRotationZ(int angle) {
    _sliderValues.at(5) = angle;
    _applySliders();
    emit setScrollRotationZChanged(angle);
}

void SceneEditorWidget::setScrollScaleX(int angle) {
    _sliderValues.at(6) = angle;
    _applySliders();
    emit setScrollScaleXChanged(angle);

}

void SceneEditorWidget::setScrollScaleY(int angle) {
    _sliderValues.at(7) = angle;
    _applySliders();
    emit setScrollScaleYChanged(angle);

}

void SceneEditorWidget::setScrollScaleZ(int angle) {
    _sliderValues.at(8) = angle;
    _applySliders();
    emit setScrollScaleZChanged(angle);
}


void SceneEditorWidget::setXRotation(int angle) {

    bmpf::qNormalizeAngle(angle);
    if (angle != xAngle) {
        xAngle = angle;
        emit xRotationChanged(angle);
        update();
    }

}

void SceneEditorWidget::setYRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != yAngle) {
        yAngle = angle;
        emit yRotationChanged(angle);
        update();
    }
}

void SceneEditorWidget::setZRotation(int angle) {
    bmpf::qNormalizeAngle(angle);
    if (angle != zAngle) {
        zAngle = angle;
        emit zRotationChanged(angle);
        update();
    }
}

