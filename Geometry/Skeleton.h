#pragma once

#include <QVector>
#include "SurfaceMeshHelper.h"
#include "../Math/mathlib.h"
#include "OBB.h"

class CScene;
class SuppPlane;

const double ReachDistThreshold = 0.6;
const double SampleGridSize = 0.4;
const double AroundBodyDistThreshold = 0.3;

const double HeightReachThreshold = 0.5;

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

const int SampleLocationNum = 5;
const int MaxIterNum = 100;

struct AcessPlane
{
	std::vector<double> corners;
	double zVal;
	int modelID;
	int suppPlaneID; // which support plane in the model
};

struct ObjLocation
{
	MathLib::Vector3 pos;
	int modelID;
	int suppPlaneID;
};

// potential object target locations
struct ObjPotentialPlacement
{
	std::vector<SuppPlane*> potentialSuppPlanesToPlace; // supp planes from other model
	std::vector<AcessPlane> accessPlanesToPlace; // intersection between skeleton access rectangle and other model's supp planes
	
	std::vector<ObjLocation> sampledLocations;
	std::vector<ObjLocation> predictedLocations;

	void samplePotentialLocation();
	bool randomSampleOnAccessPlanes(ObjLocation &samplePos);
	bool isPosValid();

	bool hasAccessiblePlanes() { return !accessPlanesToPlace.empty(); };
	void setScene(CScene *s){ m_scene = s; };

	void drawAccessPlanes(QColor c);
	void drawSampledLocations(QColor c);

private:
	CScene *m_scene;

};

class Skeleton
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
		SHOULDER_RIGHT,
		ELBOW_RIGHT,
		WRIST_RIGHT,
		HAND_RIGHT,
		SHOULDER_LEFT,
		ELBOW_LEFT,
		WRIST_LEFT,
		HAND_LEFT,	
		HIP_RIGHT,
		KNEE_RIGHT,
		ANKLE_RIGHT,
		FOOT_RIGHT,
		HIP_LEFT,
		KNEE_LEFT,
		ANKLE_LEFT,
		FOOT_LEFT
	};

	Skeleton();
	~Skeleton();

	Skeleton(QVector<Eigen::Vector4d> pts);
	Skeleton(std::vector<MathLib::Vector3> pts);

	void draw(int drawMode = 0);
	void drawJoint(int id, QColor c = QColor(255,0,0));
	void drawBone(int id, QColor c = QColor(0,255,0));

	void AlignToOrigin();

	int jointNum() { return m_joints.size(); };
	MathLib::Vector3 getJoint(int id);
	std::vector<MathLib::Vector3> getJoints();
	std::vector<SurfaceMesh::Vector3> getSurfaceMeshJoints();
	std::vector<MathLib::Vector3> getNormalizedJoints();
	
	void computeOrientation();
	MathLib::Vector3 getShoulderDir() { return m_shoulderDir; };
	MathLib::Vector3 getTorsoNormal() { return m_torsoNormal; };

	void computeOBB();
	void drawOBB();

	std::vector<MathLib::Vector3> getTransformedJoints(double transX, double transY, double transZ, double thetaZ, MathLib::Vector3 upright);

	std::vector<int> states; // whether a joint interact with object

	void setSampledPos(const MathLib::Vector3 &pos) { m_sampledPos = pos; };
	MathLib::Vector3 getSamplePos() { return m_sampledPos; };

	void setScene(CScene *s) { m_scene = s; };
	void computePlacementRegion();
	void drawPotentialPlacement();

	void sampleObjPlacementPosNearby();
	bool hasAccessiblePlanes() { return m_potentialObjPlacement.hasAccessiblePlanes(); };

	int getSampledLocationNum() { return m_potentialObjPlacement.sampledLocations.size(); };
	ObjLocation getSampledLocation(int id) { return m_potentialObjPlacement.sampledLocations[id]; };

	int getPredictedLocationNum() { return m_potentialObjPlacement.predictedLocations.size(); };
	ObjLocation getPredictedLocation(int id) { return m_potentialObjPlacement.predictedLocations[id]; };

	void setObjPredictedLocation(std::vector<ObjLocation> locations) { m_potentialObjPlacement.predictedLocations = locations; };

	void setPicked(bool s) { m_isPicked = s; };

private:
	QVector<Eigen::Vector4d> m_joints;
	MathLib::Vector3 m_shoulderDir;
	MathLib::Vector3 m_torsoDir;
	MathLib::Vector3 m_torsoNormal;	

	COBB m_OBB;
	
	MathLib::Vector3 m_sampledPos; // postion of "foot center"

	bool m_isPicked;

	CScene *m_scene;
	ObjPotentialPlacement m_potentialObjPlacement;
};
