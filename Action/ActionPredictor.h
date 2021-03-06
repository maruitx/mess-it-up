#ifndef ACTIONPREDICTOR_H
#define ACTIONPREDICTOR_H

#include <QObject>
#include "../Utilities/utility.h"
#include "../mess_mode.h"
#include "ActionLearner.h"
#include "../Math/mathlib.h"

const double LABEL_DIFF_TH = 0.1;

typedef std::map<int, std::vector<std::vector<std::vector<int>>>> ModelRelatedSkeletonIdList;

class ActionPredictor : public QObject
{
	Q_OBJECT

public:
	ActionPredictor(QObject *parent);
	~ActionPredictor();

	void init(mess_mode *m);

	bool loadTestJob(const QString &filename);

	void loadActionRepSkels();
	void loadActionRepSkels(int phaseID);

	void sampleSkeletons();
	void sampleSkeletonsForActionPhrase(int phaseID);

	void loadClassifiers();
	bool isPhaseConsidered(int phaseID);

	bool classifyTestForSkeletons(int modelID, int phaseID, int actionID, Skeleton *skel);
	bool classifyTestForSkeletonsFuzzy(int modelID, int phaseID, int actionID, Skeleton *skel);
	bool classifyStartSkeletonFuzzy(int modelID, int actionID, Skeleton *skel);
	bool classifyEndSkeletonFuzzy(int modelID, int actionID, Skeleton *skel);
	
	void repredicting(double prob, int showSkelNum);

	void genRandomSkeletonListForDisplay(int num);
	void resampleSkeletonForDisplay(int num);
	int getSampledSkelNum(int modelID, int actionID);
	int getPredictedSkelNum(int modelID, int actionID);
	
	std::vector<int> getRandomPredictedSkelList(int modelID, int actionID);
	int getRandomPredictedSkelListSize(int modelID, int actionID);
	int getSkelIDFromRandomPredictedSkelList(int modelID, int actionID, int idInList);
	
	int getNewLocationNumOfSkel(int modelID, int actionID, int skelID);
	MathLib::Vector3 getNewLocation(int modelID, int actionID, int skelID, int locationID);

	void setSkeletonPicked(int modelID, int actionID, int skelID, bool state);

	void drawSampledSkeletons(int modelID, int phaseID, int actionID);
	void drawSampleRange(int modelID);
	void drawPredictedSkeletons(int modelID, int phaseID, int actionID);

	bool isShowSampledSkeletons() { return m_showSampledSkeleton; };
	bool isFinishPredict() { return m_finishPredict; };

	bool isShowStartPose() { return m_showStartPose; };
	bool isShowEndPose() { return m_showEndPose; };

	CScene* getScene() { return m_scene; };


public slots:
	void loadTrainingResult();
	void startPredicting();

	void setShowStartPose(int state);
	void setShowEndPose(int state);
	void setDrawSampleRegionStatus(int s);

	void updateDrawArea();

signals:
	void finishPrediction();


private:
	CScene *m_scene;
	Starlab::DrawArea *m_drawArea;

	QString m_jobFilePath;
	QString m_sceneFileName;
	QString m_useFeatureType;

	SkeletonSampler *m_skeletonSampler;

	std::vector<std::vector<OpenCVClassifier*>> m_classifiers;
	double m_classProbThreshold;

	/* in training: skeleton is only meaningful when giving the scene and its corresponding center model
	// in test: all skeletons representing one action are used for prediction, and not constraint to scene and center model
	// 1st layer: for each phase of current action
	// 2nd layer: for each action type: move, sit etc.
	*/
	MultiPhaseMultiTypeSkeletonList m_loadedSkeletonsForTest;
	
	/*
	// 1st layer: for some model 
	// 2nd layer: for each phase of current action
	// 3rd layer: for each action type: move, sit etc.
	// SkeletonPtrList: each phase may correspond several skeletons
	*/
	ModelRelatedSkeletonList m_sampledSkeletonsForActions;
	ModelRelatedSkeletonList m_predictedSkeletonsForActions;

	ModelRelatedSkeletonIdList m_randomSampledSkeletonIdList;
	ModelRelatedSkeletonIdList m_randomPredictedSkeletonIdList;

	bool m_showSampledSkeleton;
	bool m_showSampeRegion;

	bool m_showStartPose;
	bool m_showEndPose;

	bool m_finishPredict;
};

#endif // ACTIONPREDICTOR_H
