#include <urdf_robot.h>
#include <solid_collider.h>
#include <base/path_finder.h>
#include <one_direction_path_finder.h>

#include "gl_scene.h"
#include "one_direction_ordered_path_finder.h"

std::shared_ptr<bmpf::PathFinder> pathFinder;
std::vector<double> actualState;

std::vector<double> start
        {2.891, 0.101, -1.610, -0.915, 0.241, 3.377, 0.982, -1.895, -1.725, 1.885, 1.530, -2.532, -2.037, -2.228, 0.126,
         -1.387, -1.135, -4.978, -2.876, -0.069, 1.063, -2.898, -1.186, 5.017};

std::vector<double> end
        {-1.169, -3.184, 0.133, -1.435, -1.304, 2.631, -0.411, -2.594, 0.547, 0.203, 0.143, -4.815, -1.140, -2.497,
         0.085, -0.745, 2.059, -2.227, 1.052, -2.708, 0.714, -2.336, -0.279, 3.811};


long currenRobotNum = 0;

unsigned int actualStatePos = 0;

bool flgPathFound = false;

double tm = 0;

long maxRobotNum = 0;

std::vector<std::pair<long, long>> actuatorIndexesRange;

// Тестовая сцена GL
class TemplateGLScene : public bmpf::GLScene {

public:
    TemplateGLScene(int clientWidth, int clientHeight, const char *caption) :
            GLScene(clientWidth, clientHeight, caption) {}

protected:
    // инициализация OpenGL
    void init() override {
        std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
        sceneWrapper->loadFromFile("../../../../config/murdf/4robots.json");

        pathFinder = std::make_shared<bmpf::OneDirectionOrderedPathFinder>(sceneWrapper, false, 1000, 15, 6000, 5, 1);

        actuatorIndexesRange = pathFinder->getScene()->getJointIndexRanges();
        pathFinder->prepare(start, end);

        maxRobotNum = actuatorIndexesRange.size() - 1;

        actualState.clear();

        for (int i = 0; i < pathFinder->getScene()->getJointCnt(); i++) {
            actualState.push_back(0.0);
        }
        glClearColor(0.06, 0.08, 0.09, 1.0);
    }

    // рисование OpenGL
    void render() override {
        glDisable(GL_LIGHTING);
        if (pathFinder->isReady()) {

            char buf[256];
            sprintf(buf, "robot num:%ld , state pos%d ", currenRobotNum, actualStatePos);
            std::string caption = buf;

            glPushMatrix();
            glScalef(2, 2, 2);


            glColor4f(0.8, 0.04, 0.09, 0.4);
            pathFinder->paint(start, true);
            glColor4f(0.04, 0.9, 0.09, 0.4);
            pathFinder->paint(end, true);
            glColor4f(0.09, 0.04, 0.8, 0.4);

            if (_flgPlay) {
                //   infoMsg("flgPlay");
                if (!flgPathFound) {
                    if (pathFinder->findTick(actualState)) {
                        flgPathFound = true;
                        pathFinder->buildPath();
                    } else {
                        //glutSetWindowTitle(logMsg.c_str());
                    }
                } else {
                    if (tm < pathFinder->getBuildedPath().size())
                        actualState = pathFinder->getPathStateFromTM(tm);
                    else {
                        tm = 0;
                        flgPathFound = false;
                        pathFinder->prepare(start, end);
                    }
                    tm += 0.1;
                }
            }
            //Scene::infoState(actualState, "actual state");
            pathFinder->paint(
                    actualState, false
            );
            //Scene::infoState(actualState, "state");
            if (pathFinder->checkCollision(actualState)) {
                //infoMsg("collision!");
            }
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
