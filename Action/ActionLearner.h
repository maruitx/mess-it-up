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

	//void init(RgbdViewer *viewer, CScene *s, Starlab::DrawArea *area);
	void init(mess_mode *m);
	
	void alignSceneToScan();
	void loadScanToSceneTransMat();

	bool loadJob(const QString &filename);
	void setSkeletonStream();

	void setTrackingObj();

	void drawSkeleton();

	bool hasJob() { return m_hasJob; };

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
	void saveExtractedFeatures();
	void saveActionRepSkels();
	void loadActionRepSkels();
	
	void genRandomSkeletonList(int num);
	void drawSampledSkeletons(int modelID, int actionID);
	bool isShowSampledSkeletons() { return m_showSampledSkeleton; };
	bool isFinishPredict() { return m_finishPredict; };

	
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
	void startPredicting();

	void resetAlignView();

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

	std::vector<int> m_trackingObjID;

	int m_currFrameId;


	// actions
	std::vector<ActionInstance> m_actionInstances;
	std::vector<std::vector<ActionFeature>> m_actionFeatures;

	std::vector<SkeletonPtrList> m_actionRepSkeletons;
	std::map<int, std::vector<SkeletonPtrList>> m_sampledSkeletonsForActions;
	std::map<int, std::vector<std::vector<int>>> m_randomSkeletonIdList;

	bool m_showSampledSkeleton;
	bool m_finishPredict;
};

#endif // ACTIONLEARNER_H
