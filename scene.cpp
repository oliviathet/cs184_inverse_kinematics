#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <limits>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <bitset>
#include <algorithm>
#include <stdlib.h>     //for using the function sleep

#include "Eigen/Geometry"

#include "Joint.h"
#include "JointSystem.h"

inline float sqr(float x) { return x*x; }

using namespace std;

const float EPSILON = 0.01;

//****************************************************
// Some Classes
//****************************************************

class Viewport;

class Viewport {
public:
	int w, h; // width and height
};



//****************************************************
// Global Variables
//****************************************************
Viewport	viewport;
bool debug;
int displayMode;
JointSystem jointSystem;
vector<Eigen::Vector3f> goals;



//****************************************************
// Transformation functions
//****************************************************
static Eigen::Vector3f rotateX(Eigen::Vector3f point, float theta) {
	Eigen::Matrix3f rotateX;
	rotateX <<
			1, 0, 0,
			0, cos(theta), -sin(theta),
			0, sin(theta), cos(theta);
	return rotateX * point;
}

static Eigen::Vector3f rotateY(Eigen::Vector3f point, float theta) {
	Eigen::Matrix3f rotateY;
	rotateY <<
			cos(theta), 0, sin(theta),
			0, 1, 0,
			-sin(theta), 0, cos(theta);

	return rotateY * point;
}

static Eigen::Vector3f rotateZ(Eigen::Vector3f point, float theta) {
	Eigen::Matrix3f rotateZ;
	rotateZ <<
		cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;
	return rotateZ * point;
}

static Eigen::Vector3f translate(Eigen::Vector3f source, Eigen::Vector3f goal) {
	return goal - source;
}



//****************************************************
// Find system Jacobian
//***************************************************
Eigen::MatrixXf findSystemJacobian() {
	Eigen::MatrixXf systemJacobian(3, 3 * jointSystem.joints.size());

	Eigen::Vector3f goalPosition = jointSystem.joints[jointSystem.joints.size() - 1].endingPosition;

	// Calculate a 3x3 Jacobian for each joint, starting from the LEFT side of our larger systemJacobian matrix
	for (std::vector<Joint>::size_type i = 0; i < jointSystem.joints.size(); i++) {
		Joint currentJoint = jointSystem.joints[i];

		Eigen::Vector3f originalVector = translate(currentJoint.startingPosition, goalPosition);

		// First column (x-rotation)
		Eigen::Vector3f rotatedXVector = rotateX(originalVector, EPSILON);
		Eigen::Vector3f differenceXVector = (rotatedXVector - originalVector) / EPSILON;

		// Second column (y-rotation)
		Eigen::Vector3f rotatedYVector = rotateY(originalVector, EPSILON);
		Eigen::Vector3f differenceYVector = (rotatedYVector - originalVector) / EPSILON;

		// Third column (z-rotation)
		Eigen::Vector3f rotatedZVector = rotateZ(originalVector, EPSILON);
		Eigen::Vector3f differenceZVector = (rotatedZVector - originalVector) / EPSILON;

		// Set appropriate columns in Jacobian matrix
		// (First column)
		systemJacobian(0, i * 3) = differenceXVector[0];
		systemJacobian(1, i * 3) = differenceXVector[1];
		systemJacobian(2, i * 3) = differenceXVector[2];
		// (Second column)
		systemJacobian(0, i * 3 + 1) = differenceYVector[0];
		systemJacobian(1, i * 3 + 1) = differenceYVector[1];
		systemJacobian(2, i * 3 + 1) = differenceYVector[2];
		// (Third column)
		systemJacobian(0, i * 3 + 2) = differenceZVector[0];
		systemJacobian(1, i * 3 + 2) = differenceZVector[1];
		systemJacobian(2, i * 3 + 2) = differenceZVector[2];
	}
	return systemJacobian;
}

//****************************************************
// Find psuedo-inverse of system jacobian
//***************************************************
static Eigen::VectorXf findChangeInTheta(Eigen::MatrixXf systemJacobian, Eigen::Vector3f goal) {
	double jointsLength = 0.0;

	Eigen::Vector3f myGoal = goal;

	for (int i = 0; i < jointSystem.joints.size(); i++) {
		jointsLength += jointSystem.joints[i].length;
	}

	if (myGoal.norm() > jointsLength) {
		myGoal = myGoal.normalized() * jointsLength;
	}
	Eigen::Vector3f goalVector = myGoal - jointSystem.joints[jointSystem.joints.size() - 1].endingPosition;
	return systemJacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(goalVector);
}


//****************************************************
// Updates rotations of our vectors
//***************************************************
void updateRotations(Eigen::VectorXf changeInRotations) {
	// changeInRotations is a vector of length m, where m = 3 * n (n = number of joints)
	// Each joint has a 3-d vector that represents the rotation that needs to be applied to it

	Eigen::Vector3f originalVector(0.0, 0.0, 0.0);

	for (int i = 0; i < jointSystem.joints.size(); i++) {

		originalVector = Eigen::Vector3f(jointSystem.joints[i].length, 0.0, 0.0);

		// Perform all 3 rotations, per joint
		jointSystem.joints[i].endingPosition = rotateX(originalVector, changeInRotations[3 * i] + jointSystem.joints[i].theta[0]);
		jointSystem.joints[i].endingPosition = rotateY(jointSystem.joints[i].endingPosition, changeInRotations[3 * i + 1] + jointSystem.joints[i].theta[1]);
		jointSystem.joints[i].endingPosition = rotateZ(jointSystem.joints[i].endingPosition, changeInRotations[3 * i + 2] + jointSystem.joints[i].theta[2]);

		jointSystem.joints[i].theta += Eigen::Vector3f(changeInRotations[3 * i], changeInRotations[3 * i + 1], changeInRotations[3 * i + 2]);

		if (i != 0) {
			jointSystem.joints[i].endingPosition = translate(-jointSystem.joints[i - 1].endingPosition, jointSystem.joints[i].endingPosition);
			jointSystem.joints[i].startingPosition = jointSystem.joints[i - 1].endingPosition;

		}
	}
}

//****************************************************
// Determine (based on error term) whether we've reached the given goal
//***************************************************
bool haveReachedGoal(Eigen::Vector3f goal) {

	Eigen::Vector3f differenceVector = (jointSystem.joints[jointSystem.joints.size() - 1].endingPosition - goal);
	float errorTerm = differenceVector.norm();
	if (debug) {
		cout << "Error term of \n" << jointSystem.joints[jointSystem.joints.size() - 1].endingPosition << "\nis " << errorTerm << "\n";
	}
	if (errorTerm < EPSILON) {
		// We are done!
		return true;
	} else {
		return false;
	}
}


//****************************************************
// Modifies joints to reach one goal
//***************************************************
void reachGoal(Eigen::Vector3f goal) {
	if (debug) {
		cout << "\n\n\nTrying to reach goal:\n" << goal << "\n";
	}

	// Do steps 3-5 if we haven't reached our goal
	if (!haveReachedGoal(goal)) {

		// TODO: Find system jacobian
		Eigen::MatrixXf systemJacobian = findSystemJacobian();

		// TODO: Find Jacobian psuedo inverse
		Eigen::VectorXf changeInTheta = findChangeInTheta(systemJacobian, goal);

		// TODO: Update rotations and render system
		updateRotations(changeInTheta);

		glutPostRedisplay();

		// TODO: How to stop this function if it is impossible to reach our goal?
	}

}


//****************************************************
// Processes all goals and tries to reach each one
//***************************************************
void reachGoals() {
	for (std::vector<Eigen::Vector3f>::size_type i = 0; i < goals.size(); i++) {
		Eigen::Vector3f currentGoal = goals[i];
		reachGoal(currentGoal);
	}
}



//****************************************************
// Initialize list of joints pointing in same direction (y-axis)
//***************************************************
void initializeJoints() {
	Joint joint1 = Joint(Eigen::Vector3f(0.0, 0.0, 0.0), 1);
	Joint joint2 = Joint(joint1.endingPosition, 1);
	Joint joint3 = Joint(joint2.endingPosition, 1.5);
	Joint joint4 = Joint(joint3.endingPosition, 1.5);
	jointSystem.addJoint(joint1);
	jointSystem.addJoint(joint2);
	jointSystem.addJoint(joint3);
	jointSystem.addJoint(joint4);
}


//****************************************************
// Initialize list of goals (just a list of points)
//***************************************************
void initializeGoals() {
	Eigen::Vector3f goal1 = Eigen::Vector3f(-0.5, 2.5, 0.0);
	Eigen::Vector3f goal2 = Eigen::Vector3f(3.0, 1.0, 0.0);
	goals.push_back(goal1);
	goals.push_back(goal2);
}


void resetAndAddGoal(Eigen::Vector3f goal) {
	goals.clear();
	goals.push_back(goal);
}


//****************************************************
// Simple init function
//****************************************************
void initScene(){
	initializeJoints();
	initializeGoals();
}


//****************************************************
// reshape viewport if the window is resized
//***************************************************
void myReshape(int w, int h) {
	viewport.w = w;
	viewport.h = h;

	glViewport (0,0,viewport.w,viewport.h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-w / 100.0, w / 100.0, -h / 100.0, h / 100.0, 20, -20);
}



//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	glShadeModel(GL_FLAT);

	glLoadIdentity();				        // make sure transformation is "zero'd"

	double colors[] = { 0.0, 0.0, 1.0,
			0.0, 1.0, 0.0,
			1.0, 0.0, 0.0,
			1.0, 1.0, 0.0 };

	reachGoals();

	// Iterate through joints and render them
	for (std::vector<Joint>::size_type i = 0; i < jointSystem.joints.size(); i++) {
		glColor3d(colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]);
		jointSystem.joints[i].renderJoint(displayMode);
	}

	glFlush();
	glutSwapBuffers();					// swap buffers (we earlier set double buffer)
}



//****************************************************
// function that assists with exit of program on space bar press
//***************************************************
void exitOnSpaceBarPress( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'd':
		displayMode += 1;
		break;

		// Space bar
	case ' ':
		exit(1);
	}

	glutPostRedisplay();
}


//****************************************************
// function that prints all of our command line option variables
//***************************************************
void printCommandLineOptionVariables( )
{
	if (debug)
	{

	}
}


//****************************************************
// function that parses command line options,
// given number of command line arguments (argc)
// and the argument array (argv)
//***************************************************
void parseCommandLineOptions(int argc, char *argv[])
{
	string flag;

	int i = 1;
	while (i <= argc - 1) {

		flag = argv[i];

		// Advance to next flag, if one exists
		i++;
	}
}



//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {

	displayMode = 1;

	// Turns debug mode ON or OFF
	debug = true;

	// This initializes glut
	glutInit(&argc, argv);

	// Parse command line options
	parseCommandLineOptions(argc, argv);
	printCommandLineOptionVariables();

	//This tells glut to use a double-buffered window with red, green, and blue channels
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// Initalize theviewport size
	viewport.w = 1000;
	viewport.h = 1000;

	//The size and position of the window
	glutInitWindowSize(viewport.w, viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow(argv[0]);

	initScene();							// quick function to set up scene

	glutDisplayFunc(myDisplay);				// function to run when its time to draw something
	glutReshapeFunc(myReshape);				// function to run when the window gets resized

	// Program exits if space bar is pressed
	glutKeyboardFunc( exitOnSpaceBarPress );

	glutMainLoop();							// infinite loop that will keep drawing and resizing
	// and whatever else

	return 0;
}








