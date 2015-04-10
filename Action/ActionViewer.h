#pragma once

#include "../Utilities/utility.h"

class ActionViewerWidget;
class ActionLearner;
class ActionPredictor;
class CScene; 

class ActionViewer : public QObject
{
	Q_OBJECT

public:
	ActionViewer(ActionLearner *actionLearner);
	ActionViewer(ActionPredictor *actionPredictor);
	~ActionViewer();

	void createWidget();
	
	int getSelectModelID();
	int getSelectActionID();

	void setWigetStatus(bool s) { m_hasWidget = s; };
	bool hasWidget() { return m_hasWidget; };

	bool isShowSampledSkeletons();
	bool isShowPredictedSkeletons();

public slots:
	void setShowModelVoxel(int state);
	void setCenterModelID();

public:
	QVector<QString> allModelNameList;
	ActionPredictor *actionPredictor;

private:
	ActionLearner *m_actionLearner;
	CScene *m_scene;

	ActionViewerWidget *m_widget;

	int m_selectModelID;
	int m_selectActionID;

	bool m_hasWidget;
};

