#include "ActionViewer.h"
#include "ActionLearner.h"
#include "ActionPredictor.h"
#include "ActionViewerWidget.h"
#include "../Geometry/Scene.h"


ActionViewer::ActionViewer(ActionLearner *actionLearner):
m_actionLearner(actionLearner)
{
	m_scene = m_actionLearner->getScene();

	allModelNameList = m_scene->getModelNameList();

	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		if (m_scene->isModelFixed(modelID))
		{
			allModelNameList[modelID] = allModelNameList[modelID] + "(fixed)";
		}
	}

	m_hasWidget = false;
}

ActionViewer::ActionViewer(ActionPredictor *actionPredictor):
m_actionPredictor(actionPredictor)
{
	m_scene = m_actionPredictor->getScene();

	allModelNameList = m_scene->getModelNameList();

	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		if (m_scene->isModelFixed(modelID))
		{
			allModelNameList[modelID] = allModelNameList[modelID] + "(fixed)";
		}
	}

	m_hasWidget = false;
}


ActionViewer::~ActionViewer()
{
	m_hasWidget = false;
}

void ActionViewer::createWidget()
{
	m_widget = new ActionViewerWidget(this);
	m_widget->show();

	m_hasWidget = true;
}

int ActionViewer::getSelectModelID()
{
	m_selectModelID = m_widget->getCurrSelectModelID();
	return m_widget->getCurrSelectModelID();
}

int ActionViewer::getSelectActionID()
{
	m_selectActionID = m_widget->getCurrSelectActionID();
	return m_widget->getCurrSelectActionID();
}

void ActionViewer::updateDisplayedSkels()
{
	//m_actionLearner->updateDrawArea();

	m_actionPredictor->updateDrawArea();
}
