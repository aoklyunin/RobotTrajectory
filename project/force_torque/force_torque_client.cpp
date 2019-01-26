#include <urdf_scene_description.h>
#include <solid_collider.h>
#include "scene_wrapper.h"
#include "camera.h"
#include "force_torque.h"

std::vector<double> start
        {-2.249, -0.468, -0.594, -2.520, 0.834, 1.116};

/*
 * keys from '1' to '+' rotate joints
 * keys from 'q' to 'y' move joint object at pos {%OFFSET_POS%} in local chordinate system in X, Y and Z directions
 */

std::vector<double> actualState;

std::vector<double> steps;

std::chrono::time_point<std::chrono::system_clock> _curTime;

double tm = 0;

bool flgPlay = false;

bool flgPathFinded = false;

std::shared_ptr<ForceTorque> forceTorque;

unsigned int actualStatePos = 0;

// pos of moving offset
const int OFFSET_POS = 6;

// offsets of objects
std::vector<std::vector<double>> cartesianOffsets;

// main window width and height
static const int clientWidth = 1280;

static const int clientHeight = 720;

static const int VERY_BIG_MOUSE_COORDS = 10000;

// this two values are designed for calculating relative mouse moving
// initial values are so big, because we need to set first mouse choors
// after creating window. Each time they are different
int prevPosX = VERY_BIG_MOUSE_COORDS;

int prevPosY = VERY_BIG_MOUSE_COORDS;

// Object for moving in 3D scene
Camera camera;

SceneWrapper sceneWrapper;

std::vector<std::vector<unsigned long>> actuatorIndexesRange;

long currenRobotNum = 0;

long maxRobotNum = 0;

bool flgTick = true;

// process mouse moving
void motionFunc(int x, int y) {
    if (prevPosX == VERY_BIG_MOUSE_COORDS) {
        prevPosX = x;
    }
    if (prevPosY == VERY_BIG_MOUSE_COORDS) {
        prevPosY = y;
    }

    double dX = x - prevPosX;
    double dY = y - prevPosY;

    prevPosX = x;
    prevPosY = y;

    camera.rotateX(-dX / 50);
    camera.rotateY(-dY / 50);

    // redraw 3D scene
    flgTick = false;
    glutPostRedisplay();
}

void display(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Eigen::Vector3d center = camera.getCenter();
    Eigen::Vector3d eye = camera.getEye();
    Eigen::Vector3d up = camera.getUp();

    gluLookAt(eye(0), eye(1), eye(2),
              center(0), center(1), center(2),
              up(0), up(1), up(2));

    glDisable(GL_LIGHTING);
    if (forceTorque->isReady()) {
        char buf[256];
        sprintf(buf, "robot num:%ld , state pos%d ", currenRobotNum, actualStatePos);
        std::string caption = buf;

        glPushMatrix();
        glScalef(2, 2, 2);

        glColor4f(0, 0, 1, 0.7);
        forceTorque->setFlgPlay(flgPlay);
        auto newTime = std::chrono::high_resolution_clock::now();
        double dt = (double)std::chrono::duration_cast<std::chrono::milliseconds>(newTime - _curTime).count() / 1000;
        _curTime = newTime;
       // info_msg(dt);
        forceTorque->tick( dt);

        forceTorque->paint();

        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
    glFlush();
    glutSwapBuffers();
}

void setCamera();

void newPlacements() {}

void toggleIdle() {
    static bool idle = true;
    if (idle) {
        glutIdleFunc(newPlacements);
        idle = false;
    } else {
        glutIdleFunc(0);
        idle = true;
    }
}

void setCamera() {
    glLoadIdentity();
    gluLookAt(0, 0, 1,
              0.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f);
    display();
}

void myReshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (float) w / (float) h, 1.0f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
}

void goodbye(void) {
    forceTorque->writeReport("../../../reports/1.csv");
    std::cout << "goodbye ..." << std::endl;
    exit(0);
}

void myKeyboard(unsigned char key, int x, int y) {
   // info_msg("myKeyboard");
    unsigned long startPos = actuatorIndexesRange.at(currenRobotNum).at(0);
    float delta = 0.1f;
    switch (key) {
        case '`':
            actualState = forceTorque->getSceneWrapper()->getRandomState();

            forceTorque->setState(actualState);
     //       info_msg("tilda");
            break;

        case '1':
            actualState.at(startPos) += delta;

            forceTorque->setState(actualState);
            break;
        case '2':
            actualState.at(startPos) -= delta;
            break;
        case '3':
            actualState.at(startPos + 1) += delta;
            break;
        case '4':
            actualState.at(startPos + 1) -= delta;
            break;
        case '5':
            actualState.at(startPos + 2) += delta;
            break;
        case '6':
            actualState.at(startPos + 2) -= delta;
            break;
        case '7':
            actualState.at(startPos + 3) += delta;
            break;
        case '8':
            actualState.at(startPos + 3) -= delta;
            break;
        case '9':
            actualState.at(startPos + 4) += delta;
            break;
        case '0':
            actualState.at(startPos + 4) -= delta;
            break;
        case '-':
            actualState.at(startPos + 5) += delta;
            break;
        case '=':
            actualState.at(startPos + 5) -= delta;
            break;
        case 27:
            goodbye();
            break;
        case 'q':
            cartesianOffsets.at(OFFSET_POS).at(0) += delta;
            break;
        case 'w':
            cartesianOffsets.at(OFFSET_POS).at(0) -= delta;
            break;
        case 'e':
            cartesianOffsets.at(OFFSET_POS).at(1) += delta;
            break;
        case 'r':
            cartesianOffsets.at(OFFSET_POS).at(1) -= delta;
            break;
        case 't':
            cartesianOffsets.at(OFFSET_POS).at(2) += delta;
            break;
        case 'y':
            cartesianOffsets.at(OFFSET_POS).at(2) -= delta;
            break;
        case 'z':
            currenRobotNum++;
            if (currenRobotNum > maxRobotNum)
                currenRobotNum = 0;
            break;
        case 'x':
            currenRobotNum--;
            if (currenRobotNum < 0)
                currenRobotNum = maxRobotNum;
            break;

        case 'c':
            actualStatePos++;
//            if (actualStatePos >= actualStates.size())
//                actualStatePos = 0;
            break;
        case 'v':
            if (actualStatePos == 0) {
//                assert(actualStates.size() > 0);
//                actualStatePos = actualStates.size() - 1;
            } else
                actualStatePos--;
            break;
        case ' ':
            flgPlay = !flgPlay;
            break;
        default:
            break;
    }

    display();
}

void menu(int choice) {

    static int fullScreen = 0;
    static int px, py, sx, sy;

    switch (choice) {
        case 1:
            if (fullScreen == 1) {
                glutPositionWindow(px, py);
                glutReshapeWindow(sx, sy);
                glutChangeToMenuEntry(1, "Full Screen", 1);
                fullScreen = 0;
            } else {
                px = glutGet((GLenum) GLUT_WINDOW_X);
                py = glutGet((GLenum) GLUT_WINDOW_Y);
                sx = glutGet((GLenum) GLUT_WINDOW_WIDTH);
                sy = glutGet((GLenum) GLUT_WINDOW_HEIGHT);
                glutFullScreen();
                glutChangeToMenuEntry(1, "Close Full Screen", 1);
                fullScreen = 1;
            }
            break;
        case 2:
            toggleIdle();
            break;
        case 3:
            goodbye();
            break;
        default:
            break;
    }
}

void init(void) {
    GLfloat light_ambient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

    /*	light_position is NOT default value	*/
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
    gluPerspective(60.0f, (float) clientWidth / (float) clientHeight, 1.0f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_DITHER);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    camera = Camera(Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(0.0, 0.0, 5.0),
                    Eigen::Vector3d(0.0, 1.0, 0.0));

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/ur10.json");

    forceTorque = std::make_shared<ForceTorque>(sceneWrapper);
    forceTorque->prepareTick();

    actuatorIndexesRange = forceTorque->getSceneWrapper()->getActuratorIndexRanges();

    maxRobotNum = actuatorIndexesRange.size() - 1;

    actualState.clear();
    cartesianOffsets.clear();

    actualState = forceTorque->getSceneWrapper()->getRandomState();

    SceneWrapper::dispState(actualState, "actual state in init");

}

void createMenu() {
    glutCreateMenu(menu);
    glutAddMenuEntry("Full Screen", 1);
    glutAddMenuEntry("Toggle Idle (Start/Stop)", 2);
    glutAddMenuEntry("Quit", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void Loop(int i) {
    glutPostRedisplay();
    flgTick = true;
    glutTimerFunc(50, Loop, 0);
};

int main(int argc, char **argv) {


    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(40, 40);
    glutInitWindowSize(clientWidth, clientHeight);
    glutCreateWindow("Collision Visualisator");

    init();
    glutKeyboardFunc(myKeyboard);
    glutReshapeFunc(myReshape);
    createMenu();

    glutTimerFunc(50, Loop, 0);

    display();
    setCamera();
    glutDisplayFunc(display);
    glutPassiveMotionFunc(motionFunc);

    _curTime =  std::chrono::high_resolution_clock::now();
    glutMainLoop();


    return 0;

}


