#pragma  once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <scene.h>
#include <QtWidgets/QSlider>
#include "scene_editor_drawer.h"
#include "base/collider.h"
#include "qt_gl_widget.h"
#include "box_drawer.h"
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <iostream>
#include <solid_collider.h>
#include <QtCore/QTimer>
#include <QtWidgets/QSlider>
#include <cmath>
#include <base/grid_path_finder.h>
#include <log.h>
#include <state.h>


QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

/**
 * Виджет редактора сцены
 */
class SceneEditorWidget : public bmpf::QTGLWidget {
Q_OBJECT

public:

    /**
     * Конструктор виджета
     * @param parent родительский виджет
     */
    explicit SceneEditorWidget(QWidget *parent = nullptr);

    /**
     * Задать слайдеры
     * @param sliders слайдеры
     */
    void setSliders(std::vector<QSlider *> sliders);

    /**
     * Задать планировщик пути
     * @param pathFinder планировщик пути
     */
    void setPathFinder(const std::shared_ptr<bmpf::PathFinder>& pathFinder);

    /**
     * Добавить объект на сцену
     */
    void doAdd(std::string);

    /**
     * Удалить объект со сцены
     */
    void doDelete();

    /**
     * Выделить следующий объект
     */
    void doPrev();

    /**
     * Выделить предыдущий объект
     */
    void doNext();

    /**
     * Освободить используемые ресурсы
     */
    void cleanup() override;

protected:

    /**
     * Рисование
     */
    void paint() override;

    /**
     * Изменить размер окна
     * @param width ширина
     * @param height высота
     */
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;

    /**
     * Инициализация
     */
    void init() override;

private:

    /**
     * Масштаб сцены
     */
    double _sceneScale = 0.3;
    /**
     * Объект для рисования сцены
     */
    SceneEditorDrawer _sceneDrawer;
    /**
     * Объект для рисования куба выделения
     */
    BoxDrawer _boxDrawer;
    /**
     * буфер вершин сцены
     */
    QOpenGLBuffer _sceneVertexBuffer;
    /**
     * буфер вершин куба выделения
     */
    QOpenGLBuffer boxVertexBuffer;
    /**
     * массив данных сцены
     */
    QOpenGLVertexArrayObject _sceneVAO;
    /**
     * массив данных куба выделения
     */
    QOpenGLVertexArrayObject _boxVAO;
    /**
     * Проекционная матрица
     */
    QMatrix4x4 _proj;
    /**
     * Планировщик пути
     */
    std::shared_ptr<bmpf::PathFinder> _pathFinder;
    /**
     * Номер выделенного робота
     */
    unsigned long actualRobotNum;
    /**
     * Список смещений каждого робота
     */
    std::vector<std::vector<double>> _groupedStartTranslation;
    /**
     * Список поворотов каждого робота
     */
    std::vector<std::vector<double>> _groupedStartRotation;
    /**
     * Список масштабов каждого робота
     */
    std::vector<std::vector<double>> _groupedStartScale;
    /**
     * Список слайдеров
     */
    std::vector<QSlider *> _sliders;
    /**
     * Значения слайдеров
     */
    std::vector<int> _sliderValues;

    /**
     * Применить изменения
     */
    void _setChanges();

    /**
     * Применить значения слайдеров
     */
    void _applySliders();

    /**
     * Обновить слайдеры по имеющимся значениям
     */
    void _modifySliders();

    /**
     * Преобразовать значение слайдера в смещение
     * @param sliderValue значение слайдера
     * @return смещение
     */
    static double _sliderToTranslation(int sliderValue) {
        return (double) sliderValue / 300;
    }

    /**
     * Преобразовать значение слайдера в поворот
     * @param sliderValue значение слайдера
     * @return поворот
     */
    static double _sliderToRotation(int sliderValue) {
        return (double) sliderValue / 180 * M_PI;
    }

    /**
     * Преобразовать значение слайдера в масштаб
     * @param sliderValue значение слайдера
     * @return масштаб
     */
    static double _sliderToScale(int sliderValue) {
        return (double) sliderValue / 3000;
    }

    /**
     * Преобразовать смещение в значение слайдера
     * @param translation смещение
     * @return значение слайдера
     */
    static int _translationToSlider(double translation) {
        return (int) (translation * 300);
    }

    /**
     * Преобразовать поворот в значение слайдера
     * @param rotation поворот
     * @return значение слайдера
     */
    static int _rotationToSlider(double rotation) {
        return (int) (rotation / 180 * M_PI);
    }

    /**
     * Преобразовать масштаб в значение слайдера
     * @param scale масштаб
     * @return значение слайдера
     */
    static int _scaleToSlider(double scale) {
        return (int) (scale * 3000);
    }


public slots:

    void setXRotation(int angle) override;

    void setYRotation(int angle) override;

    void setZRotation(int angle) override;

    void setScrollTranslationX(int angle);

    void setScrollTranslationY(int angle);

    void setScrollTranslationZ(int angle);

    void setScrollRotationX(int angle);

    void setScrollRotationY(int angle);

    void setScrollRotationZ(int angle);

    void setScrollScaleX(int angle);

    void setScrollScaleY(int angle);

    void setScrollScaleZ(int angle);

signals:

    void setScrollTranslationXChanged(int angle);

    void setScrollTranslationYChanged(int angle);

    void setScrollTranslationZChanged(int angle);

    void setScrollRotationXChanged(int angle);

    void setScrollRotationYChanged(int angle);

    void setScrollRotationZChanged(int angle);

    void setScrollScaleXChanged(int angle);

    void setScrollScaleYChanged(int angle);

    void setScrollScaleZChanged(int angle);

    void xRotationChanged(int angle);

    void yRotationChanged(int angle);

    void zRotationChanged(int angle);

    void timeChanged(int angle);


};
