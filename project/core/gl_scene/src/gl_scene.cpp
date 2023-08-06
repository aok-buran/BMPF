#include <Eigen/Core>
#include "gl_scene.h"

using namespace bmpf;

/**
 * инициализация OpenGL
 */
void GLScene::initGL() {

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(40, 40);
    glutInitWindowSize(_clientWidth, _clientHeight);
    glutCreateWindow(_caption);

    // настраиваем свет
    GLfloat light_ambient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

    GLfloat light_position0[] = {1.0f, 1.0f, 1.0f, 0.0f};
    GLfloat light_position1[] = {-1.0f, -1.0f, -1.0f, 0.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (float) _clientWidth / (float) _clientHeight, 1.0f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_DITHER);


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // инициализация, определённая в потомке
    init();

    _camera = Camera(Eigen::Vector3d(0.0, 0.0, 0.0),
                     Eigen::Vector3d(0.0, 0.0, 5.0),
                     Eigen::Vector3d(0.0, 1.0, 0.0));

    glutIdleFunc(nullptr);
}


/**
 * обработчик мыши
 * @param x координата X
 * @param y координата Y
 */
void GLScene::motionFunc(int x, int y) {
    if (_prevPosX == VERY_BIG_MOUSE_COORDS) _prevPosX = x;
    if (_prevPosY == VERY_BIG_MOUSE_COORDS) _prevPosY = y;

    double dX = x - _prevPosX;
    double dY = y - _prevPosY;

    _prevPosX = x;
    _prevPosY = y;

    _camera.rotateX(-dX / 50);
    _camera.rotateY(-dY / 50);

    // перерисовываем сцену OpenGL
    glutPostRedisplay();
}

/**
 *  рисование OpenGL
 */
void GLScene::renderGL() {
    // очищаем буферы
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    // делаем единичной модельную матрицу
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // применяем камеру к состояниям OpenGL (gluLookAt)
    _camera.applyLookAt();

    // вызов рисования, определённого в потомке
    render();

    // "проталкиваем" оставшиеся данные в буферах
    glFlush();
    // меняем местами буферы экрана
    glutSwapBuffers();
}

/**
 * обработчик изменения размеров окна
 * @param width ширина окна
 * @param height высота окна
 */
void GLScene::myReshape(int width, int height) {
    // задаём новые размеры для viewPort
    glViewport(0, 0, width, height);
    // делаем матрицу проекций единичной
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // задаём параметры перспективы
    gluPerspective(52.0f, (GLdouble) width / (GLdouble) height, 1.0f, 1000.0f);
    // делаем модельную матрицу единичной
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/**
 * обработчик клавиатуры
 * @param key введённый символ
 */
void GLScene::myKeyboard(unsigned char key) {
    switch (key) {
        case '1':
            changeState(0, 1);
            break;
        case '2':
            changeState(0, -1);
            break;
        case '3':
            changeState(1, 1);
            break;
        case '4':
            changeState(1, -1);
            break;
        case '5':
            changeState(2, 1);
            break;
        case '6':
            changeState(2, -1);
            break;
        case '7':
            changeState(3, 1);
            break;
        case '8':
            changeState(3, -1);
            break;
        case '9':
            changeState(4, 1);
            break;
        case '0':
            changeState(4, -1);
            break;
        case '-':
            changeState(5, 1);
            break;
        case '=':
            changeState(5, -1);
            break;
        case 'q':
            changeState(6, 1);
            break;
        case 'w':
            changeState(6, -1);
            break;
        case 'e':
            changeState(7, 1);
            break;
        case 'r':
            changeState(7, -1);
            break;
        case 't':
            changeState(8, 1);
            break;
        case 'y':
            changeState(8, -1);
            break;
        case 'z':
            incTarget();
            break;
        case 'x':
            decTarget();
            break;
        case 'c':
            incActualState();
            break;
        case 'v':
            decActualState();
            break;
        case ' ':
            _flgPlay = !_flgPlay;
            break;
        case 27:
            exit(0);
    }
    renderGL();
}