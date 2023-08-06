#include "gl_scene.h"

#include "solid_collider.h"
#include "base/robot.h"
#include "dh_robot.h"
#include <memory>


std::shared_ptr<bmpf::SolidCollider> collider;
std::shared_ptr<bmpf::BaseRobot> robot;

std::vector<double> actualState = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


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
class DHRobotScene : public bmpf::GLScene {

public:
    /**
     * Конструктор
     * @param clientWidth ширина окна
     * @param clientHeight  высота окна
     * @param caption заголовок
     */
    DHRobotScene(int clientWidth, int clientHeight, const char *caption) :
            GLScene(clientWidth, clientHeight, caption) {}

protected:
    // инициализация, определённая в потомке
    void init() override {
        robot = std::make_shared<bmpf::DHRobot>();
        robot->loadFromFile("../../../../config/dh/kuka_six.json");

        collider = std::make_shared<bmpf::SolidCollider>();
        collider->init({robot->getModelPaths()}, false);
    }

    // рисование, определённое в потомке
    void render() override {
        glPushMatrix();
        glScalef(2, 2, 2);
        glDisable(GL_LIGHTING);
        glColor4f(0, 1, 0, 0.4);
        collider->paint(robot->getTransformMatrices(actualState), false);
        glEnable(GL_LIGHTING);
        glPopMatrix();
    }

    // изменение состояния сцены, num - номер сочленения, delta - направление
    // изменения (только 1 или -1)
    void changeState(int num, int delta) override {
        if (0 <= num && num <= 5) {
            actualState.at(num) += 0.1 * delta;
        }
    }
};

DHRobotScene dhRobotScene(1280, 720, "Demo Scene");

void myReshape(int w, int h) {
    dhRobotScene.myReshape(w, h);
}

void myKeyboard(unsigned char key, int x, int y) {
    dhRobotScene.myKeyboard(key);
}

void display() {
    dhRobotScene.renderGL();
}

void motionFunc(int x, int y) {
    dhRobotScene.motionFunc(x, y);
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    dhRobotScene.initGL();

    glutKeyboardFunc(myKeyboard);
    glutReshapeFunc(myReshape);

    glutDisplayFunc(display);
    glutPassiveMotionFunc(motionFunc);
    glutMainLoop();

    return 0;
}
