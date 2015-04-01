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
	
	void saveExtractedFeatures(int currentActionPhase);
	void saveExtractedFeatures();

	// skeleton is only meaningful when giving the scene and its corresponding center model
	void saveActionRepSkels(int currentActionPhase);
	void saveActionRepSkels();

	//void loadActionRepSkels();

	void loadSavedActionSkels();
	
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
	QString m_jobFilePath;
	QString m_sceneFileName;
	QString m_scanFileName;

	Eigen::Matrix4d m_scanToSceneMat;
	Eigen::Matrix4d m_sceneToScanMat;
	Eigen::Vector4d m_alignViewDir;
	double m_alignedFrameMat[16];

	std::vector<Skeleton*> m_skeletonStream;
	SkeletonSampler *m_skeletonSampler;

	// pre-processed skeletons 
	// to replace the sampled skeletons from recorded skeleton stream
	// 1st layer: for some model 
	// 2nd layer: for each phase of current action
	// 3rd layer: for each action type: move, sit etc.
	// SkeletonPtrList: each phase may correspond several skeletons
	std::map<int, std::vector<std::vector<SkeletonPtrList>>> m_savedSkeletons;

	std::vector<int> m_trackingObjID;

	int m_currFrameId;

	// actions
	std::vector<ActionInstance> m_actionInstances;
	// each type of action has a set of instances/features
	std::vector<std::vector<ActionFeature>> m_actionFeatures;

	//std::vector<SkeletonPtrList> m_actionRepSkeletons;
};

#endif // ACTIONLEARNER_H
