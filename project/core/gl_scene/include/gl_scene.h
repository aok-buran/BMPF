#pragma once

#include <GL/glut.h>
#include <string>
#include <utility>
#include "camera.h"

namespace bmpf {

    /**
     * @brief Базовый класс GL сцены для упрощения работы с OpenGL
     * Базовый класс GL сцены для упрощения работы с OpenGL, почти все нужные методы уже прописаны,
     * необходимо написать класс-потомок, у которого должны быть определены методы
     * инициализации `init()`, рисования `render()` и изменения состояния
     * сцены `changeState()`
     *
     * Шаблон приложения находится в файле `demo_gl_scene.cpp`
     *
     * Управление состояниями парами (первая кнопка увеличивает, вторая уменьшает):
     * ['1', '2'], ['3', '4'], ['5', '6'], ['7', '8'], ['9', '0'], ['-', '='],
     * ['q', 'w'], ['e', 'r'], ['t', 'y']
     *
     * [z, x] - изменение цели (еслои роботов несколько)
     *
     * [c, v] - изменение номера текущего состояния (если реализован просмотр истории)
     *
     * Работает только на английской раскладке
     *
     * По пробелу меняется флаг `_flgPlay`, чтобы реализовать паузу/плей, обращайтесь
     * из класса-потомка к этому полю
     *
     */
    class GLScene {
    public:

        /**
         * Конструктор GL сцены
         * @param clientWidth ширина окна
         * @param clientHeight высота окна
         * @param caption заголовок
         */
        GLScene(int clientWidth, int clientHeight, const char *caption) :
                _clientWidth(clientWidth), _clientHeight(clientHeight), _caption(caption), _flgPlay(false) {}

        /**
         * инициализация OpenGL
         */
        virtual void initGL();

        /**
         *  рисование OpenGL
         */
        virtual void renderGL();

        /**
         * обработчик мыши
         * @param x координата X
         * @param y координата Y
         */
        virtual void motionFunc(int x, int y);

        /**
         * обработчик клавиатуры
         * @param key введённый символ
         */
        virtual void myKeyboard(unsigned char key);

        /**
         * обработчик изменения размеров окна
         * @param width ширина окна
         * @param height высота окна
         */
        virtual void myReshape(int width, int height);

    protected:
        /**
         * инициализация, определённая в потомке
         */
        virtual void init() = 0;

        /**
         * рисование, определённое в потомке
         */
        virtual void render() = 0;

        /**
         * изменение состояния сцены, num - номер сочленения, delta - направление
         * изменения (только 1 или -1)
         * @param num
         * @param delta
         */
        virtual void changeState(int num, int delta) = 0;

        /**
         * перейти к следующему состоянию
         */
        virtual void incActualState() {};

        /**
         * перейти к предыдущему состоянию
         */
        virtual void decActualState() {};

        /**
         * перейти к следующей цели
         */
        virtual void incTarget() {};

        /**
         * перейти к предыдущей цели
         */
        virtual void decTarget() {};

        /**
         * флаг режима Play
         */
        bool _flgPlay;

    private:
        /**
         * ширина окна
         */
        int _clientWidth;
        /**
         * высота окна
         */
        int _clientHeight;
        /**
         * Заголовок окна
         */
        const char *_caption;
        /**
         * камера
         */
        Camera _camera;

        /**
         * Очень большая координата для начального значения положения
         * мыши, чтобы поймать первое событие перемещения мыши и не
         * обрабатывать его
         */
        static const int VERY_BIG_MOUSE_COORDS = 10000;
        /**
         * Предыдущее положение мыши вдоль оси OX
         */
        int _prevPosX = VERY_BIG_MOUSE_COORDS;
        /**
         * Предыдущее положение мыши вдоль оси OY
         */
        int _prevPosY = VERY_BIG_MOUSE_COORDS;

    };

}