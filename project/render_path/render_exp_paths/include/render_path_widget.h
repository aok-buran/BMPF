#pragma  once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <scene.h>
#include <QtWidgets/QSlider>
#include "render_path_drawer.h"
#include "qt_gl_widget.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)


/**
 * Виджет визуализатора пути сцены
 */
class RenderPathWidget : public bmpf::QTGLWidget {
Q_OBJECT


public:
    /**
     * Конструктор виджета
     * @param parent родительский виджет
     */
    explicit RenderPathWidget(QSlider *slider, QWidget *parent = nullptr);

    /**
     * Задать планировщик пути
     * @param pathFinder планировщик пути
     * @param paths пути
     */
    void
    setPathFinder(const std::shared_ptr<bmpf::PathFinder> &pr, std::vector<std::vector<std::vector<double>>> paths);

    /**
     * Задать флаг прохождения пути
     * @param flgPlay флаг прохождения пути
     */
    void setFlgPlay(bool flgPlay);

    /**
     * Следующий эксперимент
     */
    void next();

    /**
     * Предыдущий эксперимент
     */
    void prev();

public slots:

    void setTime(int angle);

    void setXRotation(int angle) override;

    void setYRotation(int angle) override;

    void setZRotation(int angle) override;

    /**
     * Таймер
     */
    void onTimer();

signals:

    void xRotationChanged(int angle);

    void yRotationChanged(int angle);

    void zRotationChanged(int angle);

    void timeChanged(int angle);

protected:

    void resizeGL(int width, int height) Q_DECL_OVERRIDE;

    void init() override;

    void paint() override;

    /**
     * Освободить используемые ресурсы
     */
    void cleanup() override;

private:
    /**
     * Обработка изменения номера эксперимента
     */
    void _makeChangeExpNum();

    /**
     * применить изменения по заданному времени
     * @param _tm время
     */
    void _setChanges(double _tm);

    /**
     * буфер вершин сцены
     */
    QOpenGLBuffer _sceneVertexBuffer;
    /**
     * Объект для рисования сцены
     */
    RenderPathDrawer _drawer;
    /**
     * массив данных вершин
     */
    QOpenGLVertexArrayObject _vao;
    /**
     * Проекционная матрица
     */
    QMatrix4x4 _proj;
    /**
     * Планировщик пути
     */
    std::shared_ptr<bmpf::PathFinder> _pathFinder;
    /**
     * время
     */
    double _tm;
    /**
     *  флаг прохождения пути
     */
    bool _flgPlay;
    /**
     * слайдер
     */
    QSlider *_slider;
    /**
     * Загруженные из файла пути
     */
    std::vector<std::vector<std::vector<double>>> _paths;
    /**
     * кол-во экспериментов
     */
    unsigned long _expCnt;
    /**
     * текущий номер эксперимента
     */
    unsigned long _curExpNum;
    /**
     * текущее состояние
     */
    std::vector<double> _state;
    /**
     * Задержка в мс между рисованием кадра
     */
    static const int _DELAY_MS = 20;

};
