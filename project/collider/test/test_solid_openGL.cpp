#include <solid_collider.h>
#include <collider.h>


std::shared_ptr<Collider> sc;

// main window width and height
static const int clientWidth = 1600;

static const int clientHeight = 1080;

std::vector<Eigen::Matrix4d> genmatrices()
{

    Eigen::Matrix4d m1;
    m1 << 0.00079696, -0.000795692, -0.999999, 0,
        -0.000795692, 0.999999, -0.000796326, 0,
        0.999999, 0.000796326, 0.000796327, 0,
        0, 0, 0, 1;
    Eigen::Matrix4d m2;
    m2 << -0.000798227, 0.000794422, -0.999999, -0.4,
        0.00238834, -0.999997, -0.000796326, -0.000318531,
        -0.999997, -0.00238898, 0.000796327, 0.000318531,
        0, 0, 0, 1;
    Eigen::Matrix4d m3;
    m3 << -0.000798227, -0.00238707, 0.999997, -0.40002,
        0.00238834, 0.999994, 0.00238897, -0.000258822,
        -0.999997, 0.00239024, -0.000792521, -0.0246814,
        0, 0, 0, 1;

    Eigen::Matrix4d m4;
    m4 << -0.999999, 1.90089e-06, -0.00159075, -0.400467,
        -0.00159075, -0.00238961, 0.999996, 0.00107865,
        -1.90039e-06, 0.999997, 0.00238961, -0.58468,
        0, 0, 0, 1;
    Eigen::Matrix4d m5;
    m5 << -0.00079696, 0.00159075, -0.999998, -0.435467,
        0.00159202, -0.999997, -0.00159202, 0.00102297,
        -0.999998, -0.00159329, 0.000794425, -0.58468,
        0, 0, 0, 1;

    Eigen::Matrix4d m6;
    m6 << -0.000798227, -0.999998, -0.00159075, -0.435467,
        0.00238834, -0.00159265, 0.999996, 0.00184352,
        -0.999997, 0.000794424, 0.00238961, -1.09968,
        0, 0, 0, 1;

    Eigen::Matrix4d m7;
    m7 << -0.000793158, -0.00159075, -0.999998, -0.435388,
        -0.0031872, 0.999994, -0.00158821, 0.00216224,
        0.999995, 0.00318593, -0.000798223, -1.19968,
        0, 0, 0, 1;

    return std::vector<Eigen::Matrix4d>{m1, m2, m3, m4, m5, m6, m7};


}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    gluLookAt(0.0, 0.0, 5.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0);

    glPushMatrix();
    glRotatef(0.4, 1, 0, 0);
    glRotatef(0.5, 0, 1, 0);
    glScalef(2, 2, 2);
    sc->paint(genmatrices(),false);
    glPopMatrix();

    glFlush();
    glutSwapBuffers();
}

void init(void)
{

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();
    gluPerspective(60.0f, (float) clientWidth / (float) clientHeight, 1.0f, 200.0f);

    display();

}

int main(int argc, char **argv)
{

    std::vector<std::vector<std::string>> paths{{
                                                    "../../../models/kuka_six/base_link.stl",
                                                    "../../../models/kuka_six/link_1.stl",
                                                    "../../../models/kuka_six/link_2.stl",
                                                    "../../../models/kuka_six/link_3.stl",
                                                    "../../../models/kuka_six/link_4.stl",
                                                    "../../../models/kuka_six/link_5.stl",
                                                    "../../../models/kuka_six/link_6.stl"
                                                }};


    sc = std::make_shared<SolidCollider>();
    sc->init(paths);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(clientWidth, clientHeight);
    glutCreateWindow("Solid3 OpenGL path_finding");

    init();

    glutIdleFunc(0);
    display();
    glutDisplayFunc(display);
    glutMainLoop();

    return 0;
}