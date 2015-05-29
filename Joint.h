/*
 * Joint.h
 *
 *  Created on: Apr 29, 2015
 *      Author: ryanyu
 */

#ifndef JOINT_H_
#define JOINT_H_

class Joint {
	public:
		float length;
		Eigen::Vector3f theta, startingPosition, endingPosition;

	Joint() {

	}

	Joint(Eigen::Vector3f startingPosition, float length) {
		this->length = length;
		this->startingPosition = startingPosition;
		endingPosition = startingPosition + Eigen::Vector3f(length, 0.0, 0.0);
		theta = Eigen::Vector3f(0.0, 0.0, 0.0);
	}

	// Renders this joint with OpenGL commands
	void renderJoint(int displayMode) {
		if (displayMode % 3 == 0) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glBegin(GL_LINES);
			glVertex3f(startingPosition.x(), startingPosition.y(), startingPosition.z());
			glVertex3f(endingPosition.x(), endingPosition.y(), endingPosition.z());
			glEnd();
		} else if (displayMode % 3 == 1) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			float d = length / 3;
			Eigen::Vector3f first = startingPosition + Eigen::Vector3f(-d, d, -d);
			Eigen::Vector3f second = startingPosition + Eigen::Vector3f(d, d, -d);
			Eigen::Vector3f third = startingPosition + Eigen::Vector3f(d, d, d);
			Eigen::Vector3f fourth = startingPosition + Eigen::Vector3f(-d, d, d);



			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(second[0], second[1], second[2]);
			glVertex3f(third[0], third[1], third[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(third[0], third[1], third[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glVertex3f(first[0], first[1], first[2]);
			glEnd();

			//second
			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(second[0], second[1], second[2]);
			glVertex3f(third[0], third[1], third[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(third[0], third[1], third[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glVertex3f(first[0], first[1], first[2]);
			glEnd();
		} else if (displayMode % 3 == 2) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			float d = length / 3;
			Eigen::Vector3f first = startingPosition + Eigen::Vector3f(-d, d, -d);
			Eigen::Vector3f second = startingPosition + Eigen::Vector3f(d, d, -d);
			Eigen::Vector3f third = startingPosition + Eigen::Vector3f(d, d, d);
			Eigen::Vector3f fourth = startingPosition + Eigen::Vector3f(-d, d, d);

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(second[0], second[1], second[2]);
			glVertex3f(third[0], third[1], third[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(third[0], third[1], third[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(startingPosition[0], startingPosition[1], startingPosition[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glVertex3f(first[0], first[1], first[2]);
			glEnd();

			//second
			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(first[0], first[1], first[2]);
			glVertex3f(second[0], second[1], second[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(second[0], second[1], second[2]);
			glVertex3f(third[0], third[1], third[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(third[0], third[1], third[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glEnd();

			glBegin(GL_POLYGON);
			glVertex3f(endingPosition[0], endingPosition[1], endingPosition[2]);
			glVertex3f(fourth[0], fourth[1], fourth[2]);
			glVertex3f(first[0], first[1], first[2]);
			glEnd();
		}


	}

};



#endif /* JOINT_H_ */
