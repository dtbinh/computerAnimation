#include "MyWindow.h"
#include "yui/GLFuncs.h"
#include "Particle.h"
#include <iostream>
#include "GL\glut.h"

using namespace Eigen;

void MyWindow::displayTimer(int _val) {
    mWorld->simulate();
    glutPostRedisplay();
    mFrame++;
    if(mPlaying)
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
    // Draw particles
    for (int i = 0; i < mWorld->getNumParticles(); i++)
        mWorld->getParticle(i)->draw(mRI);

    //Draw a cube centered at mWorld->getCubePosition()
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    mRI->setPenColor(Vector4d(0.3, 0.3, 0.3, 1.0));
    mRI->pushMatrix();
    mRI->translate(mWorld->getCubePosition());
	glPushMatrix();
		glRotatef( 90, 1, 0, 0 );
		glutWireSphere( 0.5, 10, 10 );
	glPopMatrix();
    mRI->popMatrix();
    glEnable(GL_LIGHTING);

    // Display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
	char message[] = "Controls: a - Enable wind, b - Disable wind";
    std::string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
	yui::drawStringOnScreen(0.02f,0.08f,message);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y) {
    switch(key){
    case ' ': // Use space key to play or stop the motion
        mPlaying = !mPlaying;
        if(mPlaying)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
	case 'a': mWorld->enableWind();
		break;
	case 'b': mWorld->enableWind();
		break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void MyWindow::click(int button, int state, int x, int y) {
    // TODO: replace this code to "shake the cube"
    mMouseDown = !mMouseDown;
    if(mMouseDown){
        if (button == GLUT_LEFT_BUTTON)
            std::cout << "Left Click" << std::endl;
        else if (button == GLUT_RIGHT_BUTTON || button == GLUT_MIDDLE_BUTTON)
            std::cout << "RIGHT Click" << std::endl;
        
        mMouseX = x;
        mMouseY = y;
    }
    glutPostRedisplay();
}

void MyWindow::drag(int x, int y) {
    // TODO: replace this code to "shake the cube"
    double deltaX = x - mMouseX;
    double deltaY = y - mMouseY;

	double normalizedDeltaX = deltaX / (double)mWinWidth;
	double normalizedDeltaY = deltaY / (double)mWinHeight;
	normalizedDeltaX = normalizedDeltaX * 0.84;
	normalizedDeltaY = -normalizedDeltaY * 0.66;

	mMouseX = x;
    mMouseY = y;
	mWorld->moveContainer( normalizedDeltaX, normalizedDeltaY );
	std::cout << "Drag by (" << normalizedDeltaX << ", " << normalizedDeltaY << ")" << std::endl;

    glutPostRedisplay();
}
