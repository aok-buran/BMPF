#include <solid_collider.h>
#include <base/path_finder.h>

#include "gl_scene.h"
#include "multirobot_path_finder.h"

std::shared_ptr<bmpf::PathFinder> pathFinder;
std::vector<double> actualState;

std::vector<double> start
        {-2.967, -0.855, 0.314, -1.937, -1.676, -3.665};

std::vector<double> end
        {1.187, -0.035, -1.131, 0.000, 0.419, 3.665};

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
    // инициализация, определённая в потомке
    void init() override {
        std::shared_ptr<bmpf::Scene> sceneWrapper = std::make_shared<bmpf::Scene>();
        sceneWrapper->loadFromFile("../../../../config/murdf/demo_scene2.json");

        pathFinder = std::make_shared<MultiRobotPathFinder>(sceneWrapper, false, 1000, 15, 6000);

        actuatorIndexesRange = pathFinder->getScene()->getJointIndexRanges();
        pathFinder->prepare(start, end);

        maxRobotNum = actuatorIndexesRange.size() - 1;

        actualState.clear();

        for (int i = 0; i < pathFinder->getScene()->getJointCnt(); i++) {
            actualState.push_back(0.0);
        }

        glClearColor(0.06, 0.08, 0.09, 1.0);
    }

    // рисование, определённое в потомке
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

            std::string logMsg;
            if (_flgPlay) {
                //  infoMsg("flgPlay");
                if (!flgPathFound) {
                    if (pathFinder->findTick(actualState)) {
                        flgPathFound = true;
                        pathFinder->buildPath();
                    } else {
                        //    glutSetWindowTitle(logMsg.c_str());
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

            pathFinder->paint(actualState, true);
            pathFinder->getCollider()->paint(
                    pathFinder->getScene()->getTransformMatrices(actualState), false
            );
            //Scene::infoState(actualState, "state");
            if (pathFinder->checkCollision(actualState)) {
                // infoMsg("collision!");
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
