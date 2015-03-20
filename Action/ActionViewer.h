#pragma once

#include "../Utilities/utility.h"

class ActionViewerWidget;
class ActionLearner;
class CScene; 

class ActionViewer
{
public:
	ActionViewer(ActionLearner *actionLearner);
	~ActionViewer();

	void createWidget();
	
	int getSelectModelID();
	int getSelectActionID();

	bool hasWidget() { return m_hasWidget; };

public:
	QVector<QString> allModelNameList;

private:
	ActionLearner *m_actionLearner;
	CScene *m_scene;

	ActionViewerWidget *m_widget;

	int m_selectModelID;
	int m_selectActionID;

	bool m_hasWidget;
};

