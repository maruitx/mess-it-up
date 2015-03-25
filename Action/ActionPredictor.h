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

	void genRandomSkeletonList(int num);
	void drawSampledSkeletons(int modelID, int actionID);
	bool isShowSampledSkeletons() { return m_showSampledSkeleton; };
	bool isFinishPredict() { return m_finishPredict; };

	CScene* getScene() { return m_scene; };

	void updateDrawArea();

public slots:
	void loadTrainingResult();
	void startPredicting();


private:
	CScene *m_scene;
	Starlab::DrawArea *m_drawArea;

	QString m_jobFilePath;
	QString m_sceneFileName;

	SkeletonSampler *m_skeletonSampler;

	std::vector<SkeletonPtrList> m_actionRepSkeletons;
	std::map<int, std::vector<SkeletonPtrList>> m_sampledSkeletonsForActions;
	std::map<int, std::vector<std::vector<int>>> m_randomSkeletonIdList;

	bool m_showSampledSkeleton;
	bool m_finishPredict;
};

#endif // ACTIONPREDICTOR_H
