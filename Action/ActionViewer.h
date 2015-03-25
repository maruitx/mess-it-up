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

	bool hasWidget() { return m_hasWidget; };

public slots:
	void updateDisplayedSkels();

public:
	QVector<QString> allModelNameList;

private:
	ActionLearner *m_actionLearner;
	ActionPredictor *m_actionPredictor;
	CScene *m_scene;

	ActionViewerWidget *m_widget;

	int m_selectModelID;
	int m_selectActionID;

	bool m_hasWidget;
};

