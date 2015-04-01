#ifndef ACTIONPREDICTOR_H
#define ACTIONPREDICTOR_H

#include <QObject>
#include "../Utilities/utility.h"
#include "../mess_mode.h"
#include "ActionLearner.h"

class ActionPredictor : public QObject
{
	Q_OBJECT

public:
	ActionPredictor(QObject *parent);
	~ActionPredictor();

	void init(mess_mode *m);

	bool loadTestScene(const QString &filename);

	void loadActionRepSkels();
	void loadActionRepSkels(int currentActionPhase);

	void sampleSkeletons();
	void sampleSkeletonsForActionPhrase(int currentActionPhase);
	
	void genRandomSkeletonList(int num);
	void resampleSkeleton(int num);
	int getSampledSkelNum(int modelID, int actionID);

	void drawSampledSkeletons(int modelID, int phaseID, int actionID);
	void drawSampleRange(int modelID);

	bool isShowSampledSkeletons() { return m_showSampledSkeleton; };
	bool isFinishPredict() { return m_finishPredict; };

	bool isShowStartPose() { return m_showStartPose; };
	bool isShowEndPose() { return m_showEndPose; };



	CScene* getScene() { return m_scene; };

	void updateDrawArea();

public slots:
	void loadTrainingResult();
	void startPredicting();

	void setShowStartPose(int state);
	void setShowEndPose(int state);


private:
	CScene *m_scene;
	Starlab::DrawArea *m_drawArea;

	QString m_jobFilePath;
	QString m_sceneFileName;

	SkeletonSampler *m_skeletonSampler;

	// 1st layer: for each phase of current action
	// 2nd layer: for each action type: move, sit etc.
	std::vector<std::vector<SkeletonPtrList>> m_loadedActionSkeletons;
	
	// 1st layer: for some model 
	// 2nd layer: for each phase of current action
	// 3rd layer: for each action type: move, sit etc.
	// SkeletonPtrList: each phase may correspond several skeletons
	std::map<int, std::vector<std::vector<SkeletonPtrList>>> m_sampledSkeletonsForActions;
	std::map<int, std::vector<std::vector<std::vector<int>>>> m_randomSkeletonIdList;

	bool m_showSampledSkeleton;
	bool m_showSampeRegion;

	bool m_showStartPose;
	bool m_showEndPose;

	bool m_finishPredict;
};

#endif // ACTIONPREDICTOR_H
