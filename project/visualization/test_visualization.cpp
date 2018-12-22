#include <urdf_scene_description.h>
#include <solid_collider.h>
#include <path_finder.h>
#include <tra_star_path_finder.h>
#include "scene_wrapper.h"
#include "camera.h"


/*
 * keys from '1' to '+' rotate joints
 * keys from 'q' to 'y' move joint object at pos {%OFFSET_POS%} in local chordinate system in X, Y and Z directions
 */

std::vector<std::vector<double>> actualStates;

std::shared_ptr<PathFinder> pathFinder;

unsigned int actualStatePos = 0;

// pos of moving offset
const int OFFSET_POS = 6;

// offsets of objects
std::vector<std::vector<double>> cartesianOffsets;

// main window width and height
static const int clientWidth = 1280;

static const int clientHeight = 720;

//state of robot
std::vector<double> actualState;

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

// process mouse moving
void motionFunc(int x, int y)
{
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
    glutPostRedisplay();
}

void display(void)
{

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
    if (pathFinder->isReady()) {
        char buf[256];
        sprintf(buf, "robot num:%ld , state pos%d ", currenRobotNum, actualStatePos);
        std::string caption = buf;
        // sceneWrapper.setCartesianOffsets(cartesianOffsets);
        if (pathFinder->checkCollision(actualStates.at(actualStatePos))) {
            glutSetWindowTitle((caption + "Collision").c_str());
        }
        else {
            glutSetWindowTitle((caption + "No Collision").c_str());
        }
        glPushMatrix();
        glScalef(2, 2, 2);
        pathFinder->paint(actualStates.at(actualStatePos),false);
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
    glFlush();
    glutSwapBuffers();
}

void setCamera();

void newPlacements()
{}

void toggleIdle()
{
    static bool idle = true;
    if (idle) {
        glutIdleFunc(newPlacements);
        idle = false;
    }
    else {
        glutIdleFunc(0);
        idle = true;
    }
}

void setCamera()
{
    glLoadIdentity();
    gluLookAt(0, 0, 1,
              0.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f);
    display();
}

void myReshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (float) w / (float) h, 1.0f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
}

void goodbye(void)
{
    std::cout << "goodbye ..." << std::endl;
    exit(0);
}

void myKeyboard(unsigned char key, int x, int y)
{

    unsigned long startPos = actuatorIndexesRange.at(currenRobotNum).at(0);
    float delta = 0.1f;
    switch (key) {
        case '1': actualStates.at(actualStatePos).at(startPos) += delta;
            break;
        case '2': actualStates.at(actualStatePos).at(startPos) -= delta;
            break;
        case '3': actualStates.at(actualStatePos).at(startPos + 1) += delta;
            break;
        case '4': actualStates.at(actualStatePos).at(startPos + 1) -= delta;
            break;
        case '5': actualStates.at(actualStatePos).at(startPos + 2) += delta;
            break;
        case '6': actualStates.at(actualStatePos).at(startPos + 2) -= delta;
            break;
        case '7': actualStates.at(actualStatePos).at(startPos + 3) += delta;
            break;
        case '8': actualStates.at(actualStatePos).at(startPos + 3) -= delta;
            break;
        case '9': actualStates.at(actualStatePos).at(startPos + 4) += delta;
            break;
        case '0': actualStates.at(actualStatePos).at(startPos + 4) -= delta;
            break;
        case '-': actualStates.at(actualStatePos).at(startPos + 5) += delta;
            break;
        case '=': actualStates.at(actualStatePos).at(startPos + 5) -= delta;
            break;
        case 27:goodbye();
            break;
        case 'q':cartesianOffsets.at(OFFSET_POS).at(0) += delta;
            break;
        case 'w':cartesianOffsets.at(OFFSET_POS).at(0) -= delta;
            break;
        case 'e':cartesianOffsets.at(OFFSET_POS).at(1) += delta;
            break;
        case 'r':cartesianOffsets.at(OFFSET_POS).at(1) -= delta;
            break;
        case 't':cartesianOffsets.at(OFFSET_POS).at(2) += delta;
            break;
        case 'y':cartesianOffsets.at(OFFSET_POS).at(2) -= delta;
            break;
        case 'z':currenRobotNum++;
            if (currenRobotNum > maxRobotNum)
                currenRobotNum = 0;
            break;
        case 'x':currenRobotNum--;
            if (currenRobotNum < 0)
                currenRobotNum = maxRobotNum;
            break;

        case 'c':actualStatePos++;
            if (actualStatePos >= actualStates.size())
                actualStatePos = 0;
            break;
        case 'v':
            if (actualStatePos == 0) {
                assert(actualStates.size() > 0);
                actualStatePos = actualStates.size() - 1;
            }
            else
                actualStatePos--;
            break;


        default:break;
    }
    SceneWrapper::dispState(actualStates.at(actualStatePos), "actual state");
    display();
}

void menu(int choice)
{

    static int fullScreen = 0;
    static int px, py, sx, sy;

    switch (choice) {
        case 1:
            if (fullScreen == 1) {
                glutPositionWindow(px, py);
                glutReshapeWindow(sx, sy);
                glutChangeToMenuEntry(1, "Full Screen", 1);
                fullScreen = 0;
            }
            else {
                px = glutGet((GLenum) GLUT_WINDOW_X);
                py = glutGet((GLenum) GLUT_WINDOW_Y);
                sx = glutGet((GLenum) GLUT_WINDOW_WIDTH);
                sy = glutGet((GLenum) GLUT_WINDOW_HEIGHT);
                glutFullScreen();
                glutChangeToMenuEntry(1, "Close Full Screen", 1);
                fullScreen = 1;
            }
            break;
        case 2:toggleIdle();
            break;
        case 3:goodbye();
            break;
        default:break;
    }
}

void init(void)
{
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

    camera = Camera(Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(0.0, 0.0, 5.0),
                    Eigen::Vector3d(0.0, 1.0, 0.0));

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/demo_scene2.json");


    pathFinder = std::make_shared<AStarPathFinder>(sceneWrapper, true, 100, 5, 2000, 25,1,0,0,1);

    actuatorIndexesRange = pathFinder->getSceneWrapper()->getActuratorIndexRanges();

    maxRobotNum = actuatorIndexesRange.size() - 1;


    for (int i = 0; i < pathFinder->getSceneWrapper()->getActuatorCnt(); i++) {
        cartesianOffsets.emplace_back(std::vector<double>{0, 0, 0});
    }

    for (int i = 0; i < pathFinder->getSceneWrapper()->getActuatorCnt(); i++) {
        actualState.push_back(0.0);
    }

    actualStates =
        std::vector<std::vector<double>>{
            {2.967 ,-3.316 ,-1.613 ,-0.969 ,-0.419 ,3.665}
        };

//    actualState =
//        std::vector<double>{-2.374 ,-2.906 ,-0.168 ,1.292 ,1.676 ,4.887 ,0.000 ,-2.086 ,1.278 ,-1.292 ,-1.257 ,-3.665 ,0.000 ,-2.496 ,-1.613 ,1.937 ,-1.676 ,2.443 ,-2.374 ,-0.445 ,-2.094 ,2.583 ,-2.094 ,0.000};


}

void createMenu()
{
    glutCreateMenu(menu);
    glutAddMenuEntry("Full Screen", 1);
    glutAddMenuEntry("Toggle Idle (Start/Stop)", 2);
    glutAddMenuEntry("Quit", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

int main(int argc, char **argv)
{

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(40, 40);
    glutInitWindowSize(clientWidth, clientHeight);
    glutCreateWindow("Collision Visualisator");

    init();
    glutKeyboardFunc(myKeyboard);
    glutReshapeFunc(myReshape);
    createMenu();

    glutIdleFunc(0);
    display();
    setCamera();
    glutDisplayFunc(display);
    glutPassiveMotionFunc(motionFunc);
    glutMainLoop();

    return 0;

}
