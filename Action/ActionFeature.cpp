#include "ActionFeature.h"

const double Interact_Dist = 0.1;

ActionFeature::ActionFeature()
{

}

ActionFeature::ActionFeature(ActionLearner *learner):
m_actionLearner(learner)
{
	m_scene = m_actionLearner->getScene();
	m_repSkeletons.resize(ACTION_PHASE_NUM);
}

ActionFeature::ActionFeature(CScene *scene):
m_scene(scene)
{

}

ActionFeature::~ActionFeature()
{

}

void ActionFeature::setActionInstance(int id)
{
	m_instanceID = id;

	m_actionInstance = m_actionLearner->getActionInstance(id);
}

void ActionFeature::extractFeature()
{
	double FrameRate = 20;
	double samplingTimeIntv = 0.1;
	
	int FrameIntv = FrameRate*samplingTimeIntv;

	int totalFrameNum = m_actionInstance.endFrameID - m_actionInstance.startFrameID;
	double lastTimeLength = totalFrameNum / FrameRate;

	std::vector<double> actionFeature;
	
	computeActionFeatureAt(0, actionFeature, ActionPhase::StartAction);
	m_startActionFeatures[0] = actionFeature;

	if (lastTimeLength > 2)
	{
		int sampleNum = 1 / samplingTimeIntv;

		for (int i = 0; i < sampleNum; i++)
		{
			int currFrameID = m_actionInstance.startFrameID + i*FrameIntv;
			computeActionFeatureAt(currFrameID, actionFeature, ActionPhase::StartAction);
			m_startActionFeatures[currFrameID] = actionFeature;
		}

		for (int i = 0; i < sampleNum; i++)
		{
			int currFrameID = m_actionInstance.endFrameID - i*FrameIntv;
			computeActionFeatureAt(currFrameID, actionFeature, ActionPhase::EndAction);
			m_endActionFeatures[currFrameID] = actionFeature;
		}
	}

	
	computeActionFeatureAt(m_actionInstance.startFrameID, actionFeature, ActionPhase::FullAction);
	m_actionFeatures[m_actionInstance.startFrameID] = actionFeature;
	
	computeActionFeatureAt(m_actionInstance.endFrameID, actionFeature, ActionPhase::FullAction);
	m_actionFeatures[m_actionInstance.endFrameID] = actionFeature;

	m_featureDim = actionFeature.size();
}

void ActionFeature::computeActionFeatureAt(int frame_id, std::vector<double> &actionFeature, ActionPhase actionPhaseType)
{	
	Skeleton *skeleton = m_actionLearner->getSkeleton(frame_id);

	m_repSkeletons[actionPhaseType].push_back(skeleton);	

	QVector<QPair<int, Eigen::Matrix4d>> modelTrackMats = m_actionLearner->getModelTrackMat(frame_id);

	CModel *model = m_scene->getModel(m_actionInstance.modelID);
	CSceneRG &rg = m_scene->getSceneRG();

	std::vector<double> skeletonShapeFeature, objectGeoFeature, objectStructFeature, skeletonObjectFeature;

	// option: update scene graph


	// compute feature using transformed scene
	computeSkeletonShapeFeature(skeleton, skeletonShapeFeature);
	computeObjectGeoFeatures(model, objectGeoFeature);
	computeObjectStructFeature(model, objectStructFeature);
	computeSkeletonObjInterFeatures(skeleton, model, skeletonObjectFeature);

	//
	actionFeature.clear();
	actionFeature.insert(actionFeature.end(), skeletonShapeFeature.begin(), skeletonShapeFeature.end());
	actionFeature.insert(actionFeature.end(), objectGeoFeature.begin(), objectGeoFeature.end());
	actionFeature.insert(actionFeature.end(), objectStructFeature.begin(), objectStructFeature.end());
	actionFeature.insert(actionFeature.end(), skeletonObjectFeature.begin(), skeletonObjectFeature.end());
}

// 11dim
void ActionFeature::computeSkeletonShapeFeature(Skeleton *skeleton, std::vector<double> &skeletonShapeFeature)
{

	// upper body feature from Koppula et al. IJRR13
	// compute upper body to head distance
	std::vector<MathLib::Vector3> joints = skeleton->getJoints();
	for (int i = 0; i < 3; i++)
	{
		MathLib::Vector3 to_head = joints[i] - joints[Skeleton::HEAD];
		skeletonShapeFeature.push_back(to_head.magnitude());
	}

	for (int i = 4; i < 12; i++)
	{
		MathLib::Vector3 to_head = joints[i] - joints[Skeleton::HEAD];
		skeletonShapeFeature.push_back(to_head.magnitude());
	}

	// body pose feature similar to Sung et al. ICRA12
	// compute relative distance from hand to head
	MathLib::Vector3 shoulderDirection = joints[Skeleton::SHOULDER_RIGHT] - joints[Skeleton::SHOULDER_LEFT];
	MathLib::Vector3 torsoDirection = joints[Skeleton::SHOULDER_CENTER] - joints[Skeleton::HIP_CENTER];

	shoulderDirection.normalize();
	torsoDirection.normalize();
	MathLib::Vector3 torsoNormal = shoulderDirection.cross(torsoDirection);

	MathLib::Vector3 headToHandVec;
	int handID[2] = { Skeleton::HAND_LEFT, Skeleton::HAND_RIGHT };
	for (int i = 0; i < 2; i++)
	{
		headToHandVec = joints[handID[i]] - joints[Skeleton::HEAD];
		skeletonShapeFeature.push_back(headToHandVec.dot(shoulderDirection));
		skeletonShapeFeature.push_back(headToHandVec.dot(torsoDirection));
		skeletonShapeFeature.push_back(headToHandVec.dot(torsoNormal));
	}

	// compute relative distance from foot to hip center
	MathLib::Vector3 hipToFootVec;
	int footID[2] = { Skeleton::FOOT_LEFT, Skeleton::FOOT_RIGHT };
	for (int i = 0; i < 2; i++)
	{
		hipToFootVec = joints[footID[i]] - joints[Skeleton::HIP_CENTER];
		skeletonShapeFeature.push_back(headToHandVec.dot(shoulderDirection));
		skeletonShapeFeature.push_back(headToHandVec.dot(torsoDirection));
		skeletonShapeFeature.push_back(headToHandVec.dot(torsoNormal));
	}

	//// compute hand position features
	//MathLib::Vector3 to_head = joints[Skeleton::HAND_LEFT] - joints[Skeleton::HEAD];
	//skeletonShapeFeature.push_back(to_head.magnitude());

	//to_head = joints[Skeleton::HAND_RIGHT] - joints[Skeleton::HEAD];
	//skeletonShapeFeature.push_back(to_head.magnitude());
}

// 3 + 3*8 = 27dim
void ActionFeature::computeObjectGeoFeatures(CModel *m, std::vector<double> &objectGeoFeature)
{
	SurfaceMesh::Vector3 center = m->getTransformedOBBCenter();
	QVector<SurfaceMesh::Vector3> vps = m->getTransformedOBBVertices();

	//objectGeoFeature.resize(3 * (vps.size() + 1));

	//for (int i = 0; i < 3; i++)
	//{
	//	objectGeoFeature[i] = center[i];
	//}

	//for (int i = 0; i < vps.size(); i++)
	//{
	//	for (int j = 0; j < 3; j++)
	//	{
	//		objectGeoFeature[3 * (i + 1) + j] = vps[i][j];
	//	}
	//}

	std::vector<double> objectOBBSize = m->getOBBSize();
	objectGeoFeature.insert(objectGeoFeature.end(), objectOBBSize.begin(), objectOBBSize.end());
}

// 2dim
void ActionFeature::computeObjectStructFeature(CModel *m, std::vector<double> &objectStructFeature)
{
	int modelID = m->getID();

	double supportLevel;
	double supportChindrenNum;
	double isFixed;

	double ratioBetweenChildrenAndSupporter; //  size ratio: sum of children size / supporter size 

	supportLevel = m->supportLevel;
	supportChindrenNum = m->suppChindrenList.size();


	objectStructFeature.push_back(m->getOBBBottomHeight(MathLib::Vector3(0,0,1)));
	objectStructFeature.push_back(supportLevel);
	objectStructFeature.push_back(supportChindrenNum);
	
	// surrounding objects
}

// 20dim
void ActionFeature::computeSkeletonObjInterFeatures(Skeleton *skeleton, CModel *m, std::vector<double> &skeletonObjectFeature)
{
	SurfaceMesh::Vector3 center = m->getTransformedOBBCenter();

	std::vector<MathLib::Vector3> joints = skeleton->getJoints();

	//for (int i = 0; i < joints.size(); i++)
	//{
	//	double dist = (joints[i].x - center[0])*(joints[i].x - center[0]) +
	//		(joints[i].y - center[1])*(joints[i].y - center[1]) +
	//		(joints[i].z - center[2])*(joints[i].z - center[2]);
	//	dist = std::sqrt(dist);
	//	skeletonObjectFeature.push_back(dist);
	//}

	// joint interacting state, 20-dim binary, from SceneGrok
	std::vector<double> interStateVec(joints.size(), 0);
	for (int i = 0; i < joints.size(); i++)
	{
		SurfaceMesh::Vector3 pt(joints[i][0], joints[i][1], joints[i][2]);
		if (m->getClosestDistToVoxel(pt) < Interact_Dist)
		{
			interStateVec[i] = 1;
		}
	}

	skeletonObjectFeature.insert(skeletonObjectFeature.end(), interStateVec.begin(), interStateVec.end());

	// orientation
	MathLib::Vector3 shoulderDirection = joints[Skeleton::SHOULDER_RIGHT] - joints[Skeleton::SHOULDER_LEFT];
	MathLib::Vector3 torsoDirection = joints[Skeleton::SHOULDER_CENTER] - joints[Skeleton::HIP_CENTER];

	shoulderDirection.normalize();
	torsoDirection.normalize();
	MathLib::Vector3 torsoNormal = shoulderDirection.cross(torsoDirection);

	MathLib::Vector3 shoulderCenterToObjCenterVec = MathLib::Vector3(center[0], center[1], center[2]) - joints[Skeleton::SHOULDER_CENTER];

	//// horizontal angle diff
	//double thetaInXY = std::atan2(shoulderCenterToObjCenterVec[1], shoulderCenterToObjCenterVec[0]) - std::atan2(torsoNormal[1], torsoNormal[0]);

	//// vertical angle diff
	//double thetaInYZ = std::atan2(shoulderCenterToObjCenterVec[2], shoulderCenterToObjCenterVec[1]) - std::atan2(torsoNormal[2], torsoNormal[1]);

	//skeletonObjectFeature.push_back(thetaInXY);
	//skeletonObjectFeature.push_back(thetaInYZ);

	skeletonObjectFeature.push_back(shoulderCenterToObjCenterVec.dot(shoulderDirection));
	skeletonObjectFeature.push_back(shoulderCenterToObjCenterVec.dot(torsoDirection));
	skeletonObjectFeature.push_back(shoulderCenterToObjCenterVec.dot(torsoNormal));

}

void ActionFeature::computeActionFeatureForSkel(Skeleton *skeleton, int model_id, std::vector<double> &actionFeature)
{
	CModel *model = m_scene->getModel(model_id);

	std::vector<double> skeletonShapeFeature, objectGeoFeature, objectStructFeature, skeletonObjectFeature;

	// option: update scene graph


	// compute feature using transformed scene
	computeSkeletonShapeFeature(skeleton, skeletonShapeFeature);
	computeObjectGeoFeatures(model, objectGeoFeature);
	computeObjectStructFeature(model, objectStructFeature);
	computeSkeletonObjInterFeatures(skeleton, model, skeletonObjectFeature);

	//
	actionFeature.clear();
	actionFeature.insert(actionFeature.end(), skeletonShapeFeature.begin(), skeletonShapeFeature.end());
	actionFeature.insert(actionFeature.end(), objectGeoFeature.begin(), objectGeoFeature.end());
	actionFeature.insert(actionFeature.end(), objectStructFeature.begin(), objectStructFeature.end());
	actionFeature.insert(actionFeature.end(), skeletonObjectFeature.begin(), skeletonObjectFeature.end());
}

std::map<int, std::vector<double>>& ActionFeature::getFeatureVector(ActionPhase actionFeatureType)
{
	std::map<int, std::vector<double>> tempFeatures;

	if (actionFeatureType == ActionPhase::StartAction)
	{
		return m_startActionFeatures;
	}

	else if (actionFeatureType == ActionPhase::EndAction)
	{
		return m_endActionFeatures;
	}
	
	else if (actionFeatureType == ActionPhase::FullAction)
	{
		return m_actionFeatures;
	}

	else
	{
		return tempFeatures;
	}
	
}
