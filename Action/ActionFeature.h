#pragma once
#include <vector>
#include <QVector>
#include <QString>
#include <Eigen/Dense>

#include "ActionLearner.h"
#include "../Geometry/Scene.h"
#include "../Geometry/Skeleton.h"

// each action feature corresponds to an action instance
// each action feature contains a sequence of skeletons instead one static skeleton pose

// learning step: each action label correspond to a local structure, and one center object
// testing step: choose an object as a center object, and sample the skeleton around the center object, the prediction still base on the local structure

// for every frame which the action starts, create a feature; there may be multiple features for one frame since it may contain several labels
// each action may last for many frames, and it will be represented by one feature; problem: how to represent the skeletons, object structures within these frames; naive solution: random sampling

class CScene;
class ActionLearner;
struct ActionInstance;

class ActionFeature
{
public:
	enum ActionPhase
	{
		StartAction,
		EndAction,
		InBetweenAction,
		FullAction
	};

	ActionFeature();
	~ActionFeature();

	ActionFeature(ActionLearner *learner);
	ActionFeature(CScene *scene);
	void setActionInstance(int id);

	void extractFeature();
	std::map<int, std::vector<double>>& getFeatureVector(ActionPhase actionFeatureType);

	void computeActionFeatureAt(int frame_id, std::vector<double> &actionFeature, ActionPhase actionPhaseType);

	// skeleton shape
	void computeSkeletonShapeFeature(Skeleton *skeleton, std::vector<double> &skeletonShapeFeature);

	// object shape + structure
	void computeObjectGeoFeatures(CModel *m, std::vector<double> &objectGeoFeature);
	void computeObjectStructFeature(CModel *m, std::vector<double> &objectStructFeature);

	// skeleton + object
	void computeSkeletonObjInterFeatures(Skeleton *skeleton, CModel *m, std::vector<double> &skeletonObjectFeature);

	// for test stage, also could be used for learning synthetic skeletons
	// simply compute feature for one skeleton at sampled location
	void computeActionFeatureForSkel(Skeleton *skeleton, int model_id, std::vector<double> &actionFeature);

	int centerModelID() { return m_actionInstance.modelID; };
	int actionID(){ return m_actionInstance.actionID; };
	int featureDim() { return m_featureDim; };

	// return representative skeletons of current action instance
	// for now, start and end frames only
	// future work, frames in the middle + key frame + random sampled frames
	std::vector<Skeleton*> getActionRepSkeletons(ActionPhase actionPhaseType) { return m_repSkeletons[actionPhaseType]; };

private:
	ActionLearner *m_actionLearner;
	CScene *m_scene;

	ActionInstance m_actionInstance;
	int m_instanceID;
	
	int m_featureDim;

	// map->first: frame id
	// each feature is corresponding to a frame
	std::map<int, std::vector<double>> m_actionFeatures;
	std::map<int, std::vector<double>> m_startActionFeatures;
	std::map<int, std::vector<double>> m_endActionFeatures;

	// representative skeletons
	// 1st layer: for each phase of current action
	std::vector<std::vector<Skeleton*>> m_repSkeletons;

	std::vector<double> m_objectSpatioFeature;
	std::vector<double> m_objectStructureFeature;
};

