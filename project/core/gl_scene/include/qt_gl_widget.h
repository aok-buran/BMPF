#pragma once

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <QtWidgets/QSlider>
#include <QOpenGLShaderProgram>

namespace bmpf {

    /**
     * @brief Базовый класс для QT виджетов и использованием `OpenGL`
     *
     * Базовый класс для QT виджетов и использованием `OpenGL`
     * почти все нужные методы уже прописаны.
     * Необходимо написать класс-потомок, у которого должны быть определены методы
     * инициализации `init()`, рисования `paint()` и поворота
     * объекта вдоль базовых осей: setXRotation(), setYRotation(), setZRotation().
     * Углы поворота предполагаются целыми для удобства связывания со слайдерами
     */
    class QTGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    public:

        /**
         * Конструктор виджета
         * @param parent родительский виджет
         */
        explicit QTGLWidget(QWidget *parent = nullptr);

        /**
         * Очистить виджет
         */
        virtual void cleanup();

        /**
         * Получить минимальный размер окна
         * @return минимальный размер окна
         */
        QSize minimumSizeHint() const Q_DECL_OVERRIDE;

        /**
         * Получить максимальный размер окна
         * @return максимальный размер окна
         */
        QSize sizeHint() const Q_DECL_OVERRIDE;

        /**
         * Задать поворот вдоль оси OX
         * @param angle угол
         */
        virtual void setXRotation(int angle) = 0;

        /**
         * Задать поворот вдоль оси OY
         * @param angle угол
         */
        virtual void setYRotation(int angle) = 0;

        /**
         * Задать поворот вдоль оси OZ
         * @param angle угол
         */
        virtual void setZRotation(int angle) = 0;

    protected:

        /**
         * Инициализация
         */
        virtual void init() = 0;

        /**
         * Рисование
         */
        virtual void paint() = 0;

        /**
         * Инициализация OpenGL
         */
        void initializeGL() Q_DECL_OVERRIDE;

        /**
         * Рисование OPenGL
         */
        void paintGL() Q_DECL_OVERRIDE;

        /**
         * Обработчик нажатия мыши
         * @param event событие
         */
        void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

        /**
         * Обработчик перемещения мыши
         * @param event событие
         */
        void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

        /**
         * угол поворота вдоль оси OX
         */
        int xAngle;
        /**
         * угол поворота вдоль оси OY
         */
        int yAngle;
        /**
         * угол поворота вдоль оси OZ
         */
        int zAngle;

        /**
         * Флаг, поддеживается ли прозрачность
         */
        bool _isTransparent;
        /**
         * флаг, используется ли профиль ядра
         */
        bool _core;

        /**
         * Последнее полоежние курсора мыши
         */
        QPoint _lastMousePos;
        /**
         * Матрица преобразования мира
         */
        QMatrix4x4 _worldTransform;
        /**
         * Шейдерная программа
         */
        QOpenGLShaderProgram *_program;
        /**
         * Указатель на родительский виджет
         */
        QWidget *_parent;
        /**
         * Матрица проекций
         */
        int _projMatrixLoc{};
        /**
         * Матрица ModelView
         */
        int _mvMatrixLoc{};
        /**
         * Матрица нормалей
         */
        int _normalMatrixLoc{};
        /**
         * Матрица положения источника света
         */
        int _lightPosLoc{};
        /**
         * Камера
         */
        QMatrix4x4 _camera;
        /**
         * Заголовок окна
         */
        std::string _caption;
    };


    /**
     * Ядро вершинного шейдера
     */
    static const char *vertexShaderSourceCore =
            "#version 150\n"
            "in vec4 vertex;\n"
            "in vec3 normal;\n"
            "out vec3 vert;\n"
            "out vec3 vertNormal;\n"
            "uniform mat4 projMatrix;\n"
            "uniform mat4 mvMatrix;\n"
            "uniform mat3 normalMatrix;\n"
            "void main() {\n"
            "   vert = vertex.xyz;\n"
            "   vertNormal = normalMatrix * normal;\n"
            "   gl_Position = projMatrix * mvMatrix * vertex;\n"
            "}\n";

    /**
     * Ядро шейдера фрагментов
     */
    static const char *fragmentShaderSourceCore =
            "#version 150\n"
            "in highp vec3 vert;\n"
            "in highp vec3 vertNormal;\n"
            "out highp vec4 fragColor;\n"
            "uniform highp vec3 lightPos;\n"
            "void main() {\n"
            "   highp vec3 L = normalize(lightPos - vert);\n"
            "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
            "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
            "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
            "   fragColor = vec4(col, 1.0);\n"
            "}\n";

    /**
     * Вершинный шейдер
     */
    static const char *vertexShaderSource =
            "attribute vec4 vertex;\n"
            "attribute vec3 normal;\n"
            "varying vec3 vert;\n"
            "varying vec3 vertNormal;\n"
            "uniform mat4 projMatrix;\n"
            "uniform mat4 mvMatrix;\n"
            "uniform mat3 normalMatrix;\n"
            "void main() {\n"
            "   vert = vertex.xyz;\n"
            "   vertNormal = normalMatrix * normal;\n"
            "   gl_Position = projMatrix * mvMatrix * vertex;\n"
            "}\n";

    /**
     * Шейдер фрагментов
     */
    static const char *fragmentShaderSource =
            "varying highp vec3 vert;\n"
            "varying highp vec3 vertNormal;\n"
            "uniform highp vec3 lightPos;\n"
            "void main() {\n"
            "   highp vec3 L = normalize(lightPos - vert);\n"
            "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
            "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
            "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
            "   gl_FragColor = vec4(col, 1.0);\n"
            "}\n";


    /**
     * Ограничить значение угла
     * @param angle реальное значение угла
     */
    static void qNormalizeAngle(int &angle) {
        while (angle < 0)
            angle += 360 * 16;
        while (angle > 360 * 16)
            angle -= 360 * 16;
    }

}