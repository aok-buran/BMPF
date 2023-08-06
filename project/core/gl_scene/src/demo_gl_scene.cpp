#include "gl_scene.h"

/**
 * Класс для упрощения работы с OpenGL, почти все нужные методы уже прописаны,
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
 *
 * По пробелу меняется флаг `_flgPlay`, чтобы реализовать паузу/плей, обращайтесь
 * из класса-потомка к этому полю
 *
 */
class TemplateGLScene : public bmpf::GLScene {

public:
    TemplateGLScene(int clientWidth, int clientHeight, const char *caption) :
            GLScene(clientWidth, clientHeight, caption) {}

protected:
    // инициализация, определённая в потомке
    void init() override {
    }

    // рисование, определённое в потомке
    void render() override {
    }

    // изменение состояния сцены, num - номер сочленения, delta - направление
    // изменения (только 1 или -1)
    void changeState(int num, int delta) override {
    }
};

TemplateGLScene templateGlScene(1280, 720, "Demo Scene");

void myReshape(int w, int h) {
    templateGlScene.myReshape(w, h);
}

void myKeyboard(unsigned char key, int x, int y) {
    templateGlScene.myKeyboard(key);
}

void display() {
    templateGlScene.renderGL();
}

void motionFunc(int x, int y) {
    templateGlScene.motionFunc(x, y);
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    templateGlScene.initGL();

    glutKeyboardFunc(myKeyboard);
    glutReshapeFunc(myReshape);

    glutDisplayFunc(display);
    glutPassiveMotionFunc(motionFunc);
    glutMainLoop();

    return 0;
}
