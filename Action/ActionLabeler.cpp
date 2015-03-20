#include "ActionLabeler.h"
#include "ActionLabelerWidget.h"
#include "Kinect/RgbdViewer.h"
#include "../Geometry/Scene.h"
#include "Utilities/utility.h"

ActionLabeler::ActionLabeler(DepthSensor *depthSensor, CScene *scene, QObject *parent) :
m_depthSensor(depthSensor), m_scene(scene), m_hasWidget(false)
{
	connect(m_depthSensor, SIGNAL(currFrameChanged()), this, SLOT(updateFrameInfo()));

	allModelNameList = m_scene->getModelNameList();
}

ActionLabeler::~ActionLabeler()
{
	m_scene->setPickModelMode(false);

	for (int i = 0; i < allModelNameList.size(); i++)
	{
		m_scene->setModelPicked(i, false);
	}

	m_hasWidget = false;
}

void ActionLabeler::createWidget()
{
	if (m_depthSensor->getLoadFrameNum()==0)
	{
		Simple_Message_Box("Please load scan first");
	}
	else
	{
		m_widget = new ActionLabelerWidget(this);
		m_widget->show();

		m_widget->setCurrFrameID(0);
		m_widget->setCurrFrameLabel(m_depthSensor->getCurrFrameLabel());

		m_widget->ui->frameBrowseSlider->setMinimum(1);
		m_widget->ui->frameBrowseSlider->setMaximum(m_depthSensor->getLoadFrameNum());

		m_hasWidget = true;
	}
}

void ActionLabeler::updateFrameInfo()
{
	m_widget->setCurrFrameID(m_depthSensor->getCurrFrameID());

	QSet<FrameLabel> frameLabels = m_depthSensor->getCurrFrameLabel();
	m_widget->setCurrFrameLabel(frameLabels);

	// collect frame labels
	QSet<int> actionList, selectModelIDSet;
	foreach(FrameLabel label, frameLabels)
	{
		actionList.insert(label.first);
		selectModelIDSet.insert(label.second);
	}

	// update the action widget
	m_widget->ui->actionListWidget->clearSelection();
	foreach(int id, actionList)
	{
		m_widget->ui->actionListWidget->setCurrentRow(id, QItemSelectionModel::Select);
	}

	// update the model widget
	QVector<int> selectModelIDs;
	foreach(int id, selectModelIDSet)
	{
		selectModelIDs.push_back(id);
	}
	updateSelectModelInWidget(selectModelIDs);

	// update the select models in the scene according to the widget
	updateSelectModelInScene();
}

void ActionLabeler::addLabelToCurrFrame()
{
	int selectActionID = m_widget->getCurrSelectActionID();
	
	selectModelIDs = getSelectedModelIDList(m_widget->getSelectModelNames());

	for (int i = 0; i < selectModelIDs.size(); i++)
	{
		m_depthSensor->addLabelToCurrFrame(selectActionID, selectModelIDs[i]);
	}

	updateFrameInfo();
}

void ActionLabeler::clearLabelOfCurrFrame()
{
	m_depthSensor->clearLabelOfCurrFrame();
	updateFrameInfo();
}

void ActionLabeler::changeToFrame(int i)
{
	m_depthSensor->setCurrFrameAs(i - 1);
}

void ActionLabeler::addLabelToFrames()
{
	int fromID = m_widget->ui->fromFrameNumEdit->text().toInt();
	int toID = m_widget->ui->toFrameNumEdit->text().toInt();

	int selectActionID = m_widget->getCurrSelectActionID();
	selectModelIDs = getSelectedModelIDList(m_widget->getSelectModelNames());

	for (int i = 0; i < selectModelIDs.size(); i++)
	{
		m_depthSensor->addLabelToFrames(fromID, toID, selectActionID, selectModelIDs[i]);
	}

	m_depthSensor->setCurrFrameAs(fromID);
	updateFrameInfo();
}

void ActionLabeler::clearLabelOfFrames()
{
	int fromID = m_widget->ui->fromFrameNumEdit->text().toInt();
	int toID = m_widget->ui->toFrameNumEdit->text().toInt();

	m_depthSensor->clearLabelOfFrames(fromID, toID);

	m_depthSensor->setCurrFrameAs(fromID);
	updateFrameInfo();
}

void ActionLabeler::saveLabelFile()
{
	m_depthSensor->saveActionLabelFile();
}

void ActionLabeler::clearAllLabels()
{
	m_depthSensor->clearAllLabels();
}

QVector<int> ActionLabeler::getSelectedModelIDList(const QVector<QString> &nameList)
{
	QVector<int> idList(nameList.size());

	for (int i = 0; i < nameList.size(); i++)
	{
		idList[i] = m_scene->getModelIdByName(nameList[i]);
	}

	return idList;
}

void ActionLabeler::updateSelectModelInScene()
{
	selectModelIDs = getSelectedModelIDList(m_widget->getSelectModelNames());

	m_scene->unPickAllModels();

	for (int i = 0; i < selectModelIDs.size(); i++)
	{
		m_scene->setModelPicked(selectModelIDs[i], true);
	}

	m_scene->updateDrawArea();
}

void ActionLabeler::updateSelectModelInWidget(QVector<int> selectIDs)
{
	for (int i = 0; i < allModelNameList.size(); i++)
	{
		if (std::find(selectIDs.begin(),selectIDs.end(),i) != selectIDs.end())
		{
			m_widget->ui->modelNameListWidget->setCurrentRow(i, QItemSelectionModel::Select);
		}

		else
		{
			m_widget->ui->modelNameListWidget->setCurrentRow(i, QItemSelectionModel::Deselect);
		}
	}
}

void ActionLabeler::clearSelectedLabels()
{
	m_widget->ui->actionListWidget->clearSelection();
	m_widget->ui->modelNameListWidget->clearSelection();
}
