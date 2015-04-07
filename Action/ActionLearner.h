#ifndef ACTIONLEARNER_H
#define ACTIONLEARNER_H

#include "../Utilities/utility.h"
#include "../mess_mode.h"

class CScene;
class Skeleton;
class SkeletonSampler;
class ActionFeature;

struct ActionInstance
{
	int actionID;
	int modelID;
	QString actionLabel;
	int startFrameID;
	int endFrameID;
	bool isCompleted;
};


typedef std::vector<Skeleton*> SkeletonPtrList;

// 1st layer: for each phase of current action
// 2nd layer: for each action type: move, sit etc.
typedef std::vector<std::vector<SkeletonPtrList>> MultiPhaseMultiTypeSkeletonList;

// 1st layer: for some model 
// 2nd layer: for each phase of current action
// 3rd layer: for each action type: move, sit etc.
// SkeletonPtrList: each phase may correspond several skeletons
typedef std::map<int, MultiPhaseMultiTypeSkeletonList> ModelRelatedSkeletonList;

// 1st layer: feature for each phase of current action
// 2nd layer: feature for each action type: move, sit etc.
typedef std::vector<std::vector<ActionFeature>> MultiTypeMultiInstanceFeatures;

typedef std::vector<double> FeatureVector;
typedef std::vector<FeatureVector> FeatureVectorList;

class ActionLearner : public QObject
{
	Q_OBJECT

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ActionLearner(QObject *parent);
	~ActionLearner();

	void init(mess_mode *m);
	
	void alignSceneToScan();
	void loadScanToSceneTransMat();

	bool loadJob(const QString &filename);
	bool hasJob() { return m_hasJob; };
	bool hasSynthJob() { return m_hasSynthJob; };
	void setJobStatus(bool s) { m_hasJob = s; };


	void setSkeletonStream();
	void setTrackingObj();

	void drawSkeleton();

	void syncWithScan();
	void updateFrameId();
	void updateSkeletonInteract();
	void updateModelTransMat();

	// for action learning
	ActionInstance getActionInstance(int inst_id) { return m_actionInstances[inst_id]; };
	Skeleton* getSkeleton(int frame_id) { return m_skeletonStream[frame_id]; };
	CScene* getScene() { return m_scene; };
	QVector<QPair<int, Eigen::Matrix4d>> getModelTrackMat(int frame_id);

	void extractActionInstances();    // extract action instances from frame labels
	void collectFeatureVecsFromLabeledData();
	
	// features is not constraint to scenes
	// same feature used for both training and testing
	//void saveExtractedFeatures(int phaseID);
	//void saveExtractedFeatures();
	void collectFeaturesFromAllScenes();

	/* in training: skeleton is only meaningful when giving the scene and its corresponding center model
	// in test: all skeletons representing one action are used for prediction, and not constraint to scene and center model
	// save both kinds of skeleton
	*/
	void saveActionRepSkels(int phaseID);
	void saveActionRepSkels();
	void collectSkeletonsFromAllScenes();

	bool loadSyntheticJob(const QString &filename);
	void loadSynthActionSkels();
	void computeFeaturesForSyntheticData();

	void saveCollectedFeatures();
	void collectClassLabelForWeka();
	void saveCollectedFeaturesForWeka();
	bool isPhaseConsidered(int phaseID);
	
	//					       z  y
	//				  	       | /
	// scan coord frame:  x -- o
                  
	//					   z
	//				  	   |
	// scene coord frame:  o -- x
	//                    /
	//					 y

	Eigen::Matrix4d scanToSceneAxisTransMat;

public slots:
	void startLearning();

	void resetAlignView();
	void updateDrawArea();

private:
	RgbdViewer *m_rgbdViewer;
	CScene *m_scene;
	Starlab::DrawArea *m_drawArea;

	bool m_hasJob;
	bool m_hasSynthJob; // good data are synthesized just for testing algorithm

	QString m_jobFilePath;
	QString m_sceneFileName;
	QString m_scanFileName;

	Eigen::Matrix4d m_scanToSceneMat;
	Eigen::Matrix4d m_sceneToScanMat;
	Eigen::Vector4d m_alignViewDir;
	double m_alignedFrameMat[16];

	std::vector<Skeleton*> m_skeletonStream;
	SkeletonSampler *m_skeletonSampler;

	/*
	// pre-processed skeletons 
	// to replace the sampled skeletons from recorded skeleton stream
	// load skeleton built for different scenes
	*/
	ModelRelatedSkeletonList m_loadedSkeletonsForTrain;

	std::vector<int> m_trackingObjID;

	int m_currFrameId;

	// actions instances from labeled data
	std::vector<ActionInstance> m_actionInstances;

	// features from different scenes
	MultiTypeMultiInstanceFeatures m_actionFeatures;
	std::vector<std::vector<FeatureVectorList>> m_collectedFeatureVecs;
	std::vector<std::vector<QString>> m_collectedFeatureLabels;

};

#endif // ACTIONLEARNER_H
