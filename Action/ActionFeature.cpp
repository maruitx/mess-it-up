#include "ActionFeature.h"

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
	m_scene->buildSupportHierarchy();

	double FrameRate = 20;
	double samplingTimeIntv = 0.25;
	
	int FrameIntv = FrameRate*samplingTimeIntv;

	int totalFrameNum = m_actionInstance.endFrameID - m_actionInstance.startFrameID;
	double lastTimeLength = totalFrameNum / FrameRate;

	std::vector<double> actionFeature;

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
