#pragma once

#include <QVector>
#include "SurfaceMeshHelper.h"
#include "../Math/mathlib.h"

const int BoneMap[19][2] =
{
	// torso
	{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 0, 12 }, { 0, 16 }, { 2, 4 }, { 2, 8 },
	//left arm
	{ 4, 5 }, { 5, 6 }, { 6, 7 },
	//right arm
	{ 8, 9 }, { 9, 10 }, { 10, 11 },
	//left leg
	{ 12, 13 }, { 13, 14 }, { 14, 15 },
	//right leg
	{ 16, 17 }, { 17, 18 }, { 18, 19 }
};

class Skeleton
{
public:

	/*
    		3	
	        |
	   4----2----8
	  /	    |     \
	 5	    1      9
	/	    |       \
   6	    0        10
  /        / \         \
 7        12  16        11
         /     \
		13      17
	   /         \
      14          18
     /             \
    15              19
		
		*/

	enum JOINT_NAME {
		HIP_CENTER = 0,
		SPINE,
		SHOULDER_CENTER,
		HEAD,
		SHOULDER_LEFT,
		ELBOW_LEFT,
		WRIST_LEFT,
		HAND_LEFT,
		SHOULDER_RIGHT,
		ELBOW_RIGHT,
		WRIST_RIGHT,
		HAND_RIGHT,
		HIP_LEFT,
		KNEE_LEFT,
		ANKLE_LEFT,
		FOOT_LEFT,
		HIP_RIGHT,
		KNEE_RIGHT,
		ANKLE_RIGHT,
		FOOT_RIGHT
	};

	Skeleton();
	~Skeleton();

	Skeleton(QVector<Eigen::Vector4d> pts);
	Skeleton(std::vector<MathLib::Vector3> pts);

	void draw();
	void drawJoint(int id, QColor c = QColor(255,0,0));
	void drawBone(int id, QColor c = QColor(0,255,0));

	int jointNum() { return m_joints.size(); };
	MathLib::Vector3 getJoint(int id);
	std::vector<MathLib::Vector3> getJoints();
	std::vector<SurfaceMesh::Vector3> getSurfaceMeshJoints();
	std::vector<MathLib::Vector3> getNormalizedJoints();

	std::vector<MathLib::Vector3> getTransformedJoints(double newHipX, double newHipY, double thetaZ, MathLib::Vector3 upright);

	std::vector<int> states; // whether a joint interact with object

private:
	QVector<Eigen::Vector4d> m_joints;
};

