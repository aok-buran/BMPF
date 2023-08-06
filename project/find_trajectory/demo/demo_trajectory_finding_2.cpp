#include <urdf_robot.h>
#include <solid_collider.h>
#include <base/path_finder.h>
#include <one_direction_path_finder.h>

#include "gl_scene.h"
#include "one_direction_ordered_path_finder.h"
#include "multirobot_path_finder.h"
#include "monotone_trajectory_finder.h"

std::shared_ptr<bmpf::MonotoneTrajectoryFinder> trajectoryFinder;
std::vector<double> actualState;

std::vector<double> start
        {-2.372, -2.251, 1.977, 0.031, 1.885, 5.093, -2.043, -0.717, -0.893, 0.307, 0.687, -0.148, 0.723, 0.667,
         -1.421, -2.498, 1.934, -4.705, -2.144, -2.477, 1.529, 0.919, 1.333, 2.003};

std::vector<double> end
        {0.262, -3.238, 1.314, 2.603, -0.827, -3.604, -1.641, -0.440, 1.958, 1.606, 1.474, -4.645, -2.421, -0.583,
         0.134, -0.834, 2.049, -4.375, -2.353, -2.529, 0.148, -0.707, 0.145, -2.702};



double timeScale = 20.0;

long currenRobotNum = 0;

unsigned int actualStatePos = 0;

bool flgPathFound = false;

double tm = 0;

long maxRobotNum = 0;
bool ready = false;

std::vector<std::pair<long, long>> actuatorIndexesRange;

// Тестовая сцена GL
class TemplateGLScene : public bmpf::GLScene {

public:
    TemplateGLScene(int clientWidth, int clientHeight, const char *caption) :
            GLScene(clientWidth, clientHeight, caption) {}

protected:
    // инициализация, определённая в потомке
    void init() override {
        std::shared_ptr<bmpf::Scene> scene = std::make_shared<bmpf::Scene>();
        scene->loadFromFile("../../../config/murdf/4robots.json");

        trajectoryFinder = std::make_shared<bmpf::MonotoneTrajectoryFinder>();
        trajectoryFinder->init(scene, 0.1, false, 1000, 10, 3000, 100, 1);

        actuatorIndexesRange = scene->getJointIndexRanges();
        int errorCode;
        trajectoryFinder->prepareTrajectory(start, end, errorCode);

        maxRobotNum = actuatorIndexesRange.size() - 1;

        actualState.clear();

        for (int i = 0; i < scene->getJointCnt(); i++) {
            actualState.push_back(0.0);
        }
        bmpf::infoMsg("inited");
        ready = true;
        glClearColor(0.06, 0.08, 0.09, 1.0);
    }

    // рисование, определённое в потомке
    void render() override {
        glDisable(GL_LIGHTING);
        if (ready) {


            char buf[256];
            sprintf(buf, "robot num:%ld , state pos%d ", currenRobotNum, actualStatePos);
            std::string caption = buf;

            glPushMatrix();
            glScalef(2, 2, 2);

            glColor4f(0.8, 0.04, 0.09, 0.4);
            trajectoryFinder->getPF()->paint(start, true);
            glColor4f(0.04, 0.9, 0.09, 0.4);
            trajectoryFinder->getPF()->paint(end, true);
            glColor4f(0.09, 0.04, 0.8, 0.4);

            std::string logMsg;
            if (_flgPlay) {
                if (tm < trajectoryFinder->getWholeDuration() * timeScale) {
                    auto trPos = trajectoryFinder->getTrajectoryPosition(tm / timeScale);
                    actualState = std::vector<double>(trPos.begin() + 1,
                                                      trPos.begin() + 1 + trajectoryFinder->getScene()->getJointCnt());
                } else
                    tm = 0;
                tm += 0.1;
            }

            trajectoryFinder->getPF()->paint(actualState, true);
            trajectoryFinder->getPF()->getCollider()->paint(
                    trajectoryFinder->getScene()->getTransformMatrices(actualState), false
            );
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);
    }

    // изменение состояния сцены, num - номер сочленения, delta - направление
    // изменения (только 1 или -1)
    void changeState(int num, int delta) override {
        if (0 <= num && num <= 5) {
            unsigned long startPos = actuatorIndexesRange.at(currenRobotNum).first;
            actualState.at(startPos + num) += 0.1 * delta;
        }
    }

    // перейти к следующему состоянию
    void incActualState() override {
        actualStatePos++;
        if (actualStatePos >= actualState.size())
            actualStatePos = actualState.size() - 1;
    }

    // перейти к предыдущему состоянию
    void decActualState() override {
        actualStatePos--;
        if (actualStatePos < 0)
            actualStatePos = 0;
    }

    // перейти к следующей цели
    void incTarget() override {
        currenRobotNum++;
        if (currenRobotNum > maxRobotNum)
            currenRobotNum = 0;
    }

    // перейти к предыдущей цели
    void decTarget() override {
        currenRobotNum--;
        if (currenRobotNum < 0)
            currenRobotNum = maxRobotNum;
    }
};

TemplateGLScene testGlScene(1280, 720, "Collision Visualisator");


void myReshape(int w, int h) {
    testGlScene.myReshape(w, h);
}

void myKeyboard(unsigned char key, int x, int y) {
    testGlScene.myKeyboard(key);
}

void display() {
    testGlScene.renderGL();
}

void motionFunc(int x, int y) {
    testGlScene.motionFunc(x, y);
}

void Loop(int i) {
    glutPostRedisplay();
    glutTimerFunc(50, Loop, 0);
};

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    testGlScene.initGL();

    glutKeyboardFunc(myKeyboard);
    glutReshapeFunc(myReshape);

    glutTimerFunc(50, Loop, 0);

    glutDisplayFunc(display);
    glutPassiveMotionFunc(motionFunc);
    glutMainLoop();

    return 0;
}