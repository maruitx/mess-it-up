#include "ActionFeature.h"

ActionFeature::ActionFeature()
{

}

ActionFeature::ActionFeature(ActionLearner *learner):
m_actionLearner(learner)
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
	CScene *scene = m_actionLearner->getScene();
	scene->buildSupportHierarchy();

	std::vector<double> actionFeature;
	computeActionFeatureAt(m_actionInstance.startFrameID, actionFeature);
	m_actionFeatures[m_actionInstance.startFrameID] = actionFeature;
	
	computeActionFeatureAt(m_actionInstance.endFrameID, actionFeature);
	m_actionFeatures[m_actionInstance.endFrameID] = actionFeature;

	m_featureDim = actionFeature.size();
}

void ActionFeature::computeActionFeatureAt(int frame_id, std::vector<double> &actionFeature)
{
	CScene *scene = m_actionLearner->getScene();

	Skeleton *skeleton = m_actionLearner->getSkeleton(frame_id);
	m_skeletonPool.push_back(skeleton);

	QVector<QPair<int, Eigen::Matrix4d>> modelTrackMats = m_actionLearner->getModelTrackMat(frame_id);

	CModel *model = scene->getModel(m_actionInstance.modelID);
	CSceneRG &rg = scene->getSceneRG();

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

void ActionFeature::computeSkeletonShapeFeature(Skeleton *skeleton, std::vector<double> &skeletonShapeFeature)
{
	// compute upper body to head distance features
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

	// compute hand position features
	MathLib::Vector3 to_head = joints[Skeleton::HAND_LEFT] - joints[Skeleton::HEAD];
	skeletonShapeFeature.push_back(to_head.magnitude());

	to_head = joints[Skeleton::HAND_RIGHT] - joints[Skeleton::HEAD];
	skeletonShapeFeature.push_back(to_head.magnitude());
}

void ActionFeature::computeObjectGeoFeatures(CModel *m, std::vector<double> &objectGeoFeature)
{
	SurfaceMesh::Vector3 center = m->getTransformedOBBCenter();
	QVector<SurfaceMesh::Vector3> vps = m->getTransformedOBBVertices();

	objectGeoFeature.resize(3 * (vps.size() + 1));

	for (int i = 0; i < 3; i++)
	{
		objectGeoFeature[i] = center[i];
	}

	for (int i = 0; i < vps.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			objectGeoFeature[3 * (i + 1) + j] = vps[i][j];
		}
	}
}

void ActionFeature::computeSkeletonObjInterFeatures(Skeleton *skeleton, CModel *m, std::vector<double> &skeletonObjectFeature)
{
	SurfaceMesh::Vector3 center = m->getTransformedOBBCenter();

	std::vector<MathLib::Vector3> joints = skeleton->getJoints();

	for (int i = 0; i < joints.size(); i++)
	{
		double dist = (joints[i].x - center[0])*(joints[i].x - center[0]) +
			(joints[i].y - center[1])*(joints[i].y - center[1]) +
			(joints[i].z - center[2])*(joints[i].z - center[2]);
		dist = std::sqrt(dist);
		skeletonObjectFeature.push_back(dist);
	}
}

int ActionFeature::actionID()
{
	return m_actionInstance.actionID;
}

void ActionFeature::computeObjectStructFeature(CModel *m, std::vector<double> &objectStructFeature)
{
	int modelID = m->getID();

	double supportLevel;
	double supportChindrenNum;
	double isFixed;

	double ratioBetweenChildrenAndSupporter; //  size ratio: sum of children size / supporter size 

	supportLevel = m->supportLevel;
	supportChindrenNum = m->suppChindrenList.size();

	objectStructFeature.push_back(supportLevel);
	objectStructFeature.push_back(supportChindrenNum);
}
