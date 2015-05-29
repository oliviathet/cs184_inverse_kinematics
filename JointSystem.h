/*
 * JointSystem.h
 *
 *  Created on: Apr 29, 2015
 *      Author: ryanyu
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

class JointSystem {
	public:
		std::vector<Joint> joints;


	JointSystem() {

	}

	JointSystem(std::vector<Joint> joints) {
		this->joints = joints;
	}

	void addJoint(Joint joint) {
		joints.push_back(joint);
	}
};



#endif /* SYSTEM_H_ */
