#include <urdf_scene_description.h>
#include <solid_collider.h>
#include <path_finder.h>
#include <tra_star_path_finder.h>
#include <ordered_tra_star_path_finder.h>
#include <weight_a_star_path_finder.h>
#include "scene_wrapper.h"
#include "camera.h"

std::vector<double> start
    {2.891, 0.101, -1.610, -0.915, 0.241, 3.377, 0.982, -1.895, -1.725, 1.885, 1.530, -2.532, -2.037, -2.228, 0.126,
     -1.387, -1.135, -4.978, -2.876, -0.069, 1.063, -2.898, -1.186, 5.017};

std::vector<double> end
    {-1.169, -3.184, 0.133, -1.435, -1.304, 2.631, -0.411, -2.594, 0.547, 0.203, 0.143, -4.815, -1.140, -2.497,
     0.085, -0.745, 2.059, -2.227, 1.052, -2.708, 0.714, -2.336, -0.279, 3.811};

/*
 * keys from '1' to '+' rotate joints
 * keys from 'q' to 'y' move joint object at pos {%OFFSET_POS%} in local chordinate system in X, Y and Z directions
 */

std::vector<double> actualState;

std::vector<double> steps;

double tm = 0;

bool flgPlay = false;

bool flgPathFinded = false;

std::shared_ptr<PathFinder> pathFinder;

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
    flgTick = false;
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
//        if (pathFinder->checkCollision(actualStates.at(actualStatePos))) {
//            glutSetWindowTitle((caption + "Collision").c_str());
//        }
//        else {
//            glutSetWindowTitle((caption + "No Collision").c_str());
//        }
        glPushMatrix();
        glScalef(2, 2, 2);

        glColor4f(1, 0, 0, 0.4);
        pathFinder->paint(start, true);
        glColor4f(0, 1, 0, 0.4);
        pathFinder->paint(end, true);
        glColor4f(0, 0, 1, 0.4);

        std::string logMsg;
        if (flgPlay) {
         //   info_msg("flgPlay");
            if (!flgPathFinded) {
                if (pathFinder->tick(actualState, logMsg)) {
                    flgPathFinded = true;
                    pathFinder->buildPath();
                }
                else {
                    glutSetWindowTitle(logMsg.c_str());
                }
            }
            else {
                if (tm <= pathFinder->getPathTimeDuraion())
                    actualState = pathFinder->getStateFromPath(tm);
                else {
                    tm = 0;
                    flgPathFinded = false;
                    pathFinder->prepareTick(start, end);
                }
                tm += 0.1;
            }
        }
        //SceneWrapper::dispState(actualState, "actual state");
        pathFinder->paint(
            actualState, false
        );
        //SceneWrapper::dispState(actualState, "state");
        if (pathFinder->checkCollision(actualState)) {
            info_msg("collision!");
        }
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
        case '1': actualState.at(startPos) += delta;
            break;
        case '2': actualState.at(startPos) -= delta;
            break;
        case '3': actualState.at(startPos + 1) += delta;
            break;
        case '4': actualState.at(startPos + 1) -= delta;
            break;
        case '5': actualState.at(startPos + 2) += delta;
            break;
        case '6': actualState.at(startPos + 2) -= delta;
            break;
        case '7': actualState.at(startPos + 3) += delta;
            break;
        case '8': actualState.at(startPos + 3) -= delta;
            break;
        case '9': actualState.at(startPos + 4) += delta;
            break;
        case '0': actualState.at(startPos + 4) -= delta;
            break;
        case '-': actualState.at(startPos + 5) += delta;
            break;
        case '=': actualState.at(startPos + 5) -= delta;
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
//            if (actualStatePos >= actualStates.size())
//                actualStatePos = 0;
            break;
        case 'v':
            if (actualStatePos == 0) {
//                assert(actualStates.size() > 0);
//                actualStatePos = actualStates.size() - 1;
            }
            else
                actualStatePos--;
            break;
        case ' ':flgPlay = !flgPlay;
            break;
        default:break;
    }
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

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    camera = Camera(Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(0.0, 0.0, 5.0),
                    Eigen::Vector3d(0.0, 1.0, 0.0));

    std::shared_ptr<SceneWrapper> sceneWrapper = std::make_shared<SceneWrapper>();
    sceneWrapper->buildFromFile("../../../config/murdf/4robots.json");

    pathFinder = std::make_shared<OrderedAStarPathFinder>(sceneWrapper, false, 1000, 15, 6000, 25, 5, 0, 0, 1);

    actuatorIndexesRange = pathFinder->getSceneWrapper()->getActuratorIndexRanges();
    pathFinder->prepareTick(start, end);

    maxRobotNum = actuatorIndexesRange.size() - 1;

    actualState.clear();
    cartesianOffsets.clear();

    for (int i = 0; i < pathFinder->getSceneWrapper()->getActuatorCnt(); i++) {
        cartesianOffsets.emplace_back(std::vector<double>{0, 0, 0});
        actualState.push_back(0.0);
    }

    SceneWrapper::dispState(actualState, "actual state in init");

}

void createMenu()
{
    glutCreateMenu(menu);
    glutAddMenuEntry("Full Screen", 1);
    glutAddMenuEntry("Toggle Idle (Start/Stop)", 2);
    glutAddMenuEntry("Quit", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void Loop(int i)
{
    glutPostRedisplay();
    flgTick = true;
    glutTimerFunc(50, Loop, 0);
};

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

    glutTimerFunc(50, Loop, 0);

    display();
    setCamera();
    glutDisplayFunc(display);
    glutPassiveMotionFunc(motionFunc);
    glutMainLoop();


    return 0;

}


