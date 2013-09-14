#include "MyWindow.h"
#include "yui/GLFuncs.h"
#include "Particle.h"
#include <iostream>

using namespace Eigen;

void MyWindow::displayTimer(int _val) {
	double normalizedX = mMouseX / (double)mWinWidth;
	double normalizedY = (mWinHeight - mMouseY) / (double)mWinHeight;
	normalizedX = normalizedX * 1.07 - 0.535;
	normalizedY = normalizedY * 0.8 - 0.4;
	mWorld->simulate(normalizedX, normalizedY);
    glutPostRedisplay();
    mFrame++;
    if(mPlaying)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
	mWorld->draw(mRI);

    //Draw a circle
    glDisable(GL_LIGHTING);
    mRI->setPenColor(Vector4d(0.3, 0.3, 0.3, 1.0));
    mRI->pushMatrix();
    glBegin(GL_LINE_LOOP);
    double rad = 3.14 / 180.0;
    double radius = 0.2;
    for (int i = 0; i < 360; i++) {
        double angle = i * rad;
        glVertex3d(radius * cos(angle), radius * sin(angle), 0.0);
    }
    glEnd();
    mRI->popMatrix();
    glEnable(GL_LIGHTING);

    // Display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    std::string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
	yui::drawStringOnScreen(0.02f,0.28f,"Controls: a Toggle spring mode. If enabled, press left mouse to add and move spring.");
	yui::drawStringOnScreen(0.02f,0.18f,"Controls: q/w Enable/disable gravity, e/r: Enable/disable damping");
	yui::drawStringOnScreen(0.02f,0.12f,"Controls: Press anywhere to place a bead. Then, right click near existing bead to add constraint.");
	yui::drawStringOnScreen(0.02f,0.06f,"Controls: Press 1 to resume. Add bead near wire to register it to wire");
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y) {
    switch(key){
    case ' ': // Use space key to play or stop the motion
        mPlaying = !mPlaying;
        if(mPlaying)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
	case '1':
		mWorld->resumeSimulation();
		break;
	case 'a' :
		mWorld->toggleSpringMode();
		break;
	case 'q':
		mWorld->enableGravity();
		break;
	case 'w':
		mWorld->disableGravity();
		break;
	case 'e':
		mWorld->enableDamping();
		break;
	case 'r':
		mWorld->disableDamping();
		break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void MyWindow::click(int button, int state, int x, int y) {
    mMouseDown = !mMouseDown;
    if(mMouseDown){
		double normalizedX = x / (double)mWinWidth;
		double normalizedY = (mWinHeight - y) / (double)mWinHeight;
		normalizedX = normalizedX * 1.07 - 0.535;
		normalizedY = normalizedY * 0.8 - 0.4;
		if (button == GLUT_LEFT_BUTTON)
		{
            std::cout << "Left Click" << std::endl;
			mWorld->onLeftMouseClick(normalizedX, normalizedY);
		}
        else if (button == GLUT_RIGHT_BUTTON || button == GLUT_MIDDLE_BUTTON)
		{
            std::cout << "RIGHT Click" << std::endl;
			mWorld->onRightMouseClick(normalizedX, normalizedY);
		}
        
        mMouseX = x;
        mMouseY = y;
    }
	else {
		if (button == GLUT_LEFT_BUTTON)
		{
			mWorld->onLeftMouseReleased();
		}
	}
    glutPostRedisplay();
}

void MyWindow::drag(int x, int y) {
    double deltaX = x - mMouseX;
    double deltaY = y - mMouseY;

    mMouseX = x;
    mMouseY = y;
    std::cout << "Drag by (" << deltaX << ", " << deltaY << ")" << std::endl;

    glutPostRedisplay();
}
