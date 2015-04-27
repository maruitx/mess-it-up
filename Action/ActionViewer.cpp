#include "ActionViewer.h"
#include "ActionViewerWidget.h"
#include "ActionLearner.h"
#include "ActionPredictor.h"
#include "../Geometry/Scene.h"

ActionViewer::ActionViewer(ActionPredictor *actPredictor):
actionPredictor(actPredictor)
{
	m_scene = actionPredictor->getScene();

	allModelNameList = m_scene->getModelNameList();

	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		if (m_scene->isModelFixed(modelID))
		{
			allModelNameList[modelID] = allModelNameList[modelID] + "(fixed)";
		}
	}

	m_hasWidget = false;

	connect(actionPredictor, SIGNAL(finishPrediction()), this, SLOT(updatePredictedSkelAndLocationList()));
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

	updatePredictedSkelAndLocationList();
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

void ActionViewer::setCenterModelID()
{
	m_selectModelID = m_widget->getCurrSelectModelID();
	m_scene->setCenterModelID(m_selectModelID);
}

void ActionViewer::setShowModelVoxel(int state)
{
	m_scene->setShowModelVoxel(state);
	actionPredictor->updateDrawArea();
	
}

void ActionViewer::setShowVoxelOctree(int state)
{
	m_scene->setShowVoxelOctree(state);
	actionPredictor->updateDrawArea();
}


bool ActionViewer::isShowSampledSkeletons()
{
	return m_widget->ui.showSampledSkelButton->isChecked();
}

bool ActionViewer::isShowPredictedSkeletons()
{
	return m_widget->ui.showPredictedSkelButton->isChecked();
}

void ActionViewer::updatePredictedSkelAndLocationList()
{
	m_displayedSkelIDList.clear();

	if (m_widget)
	{
		int selectModelID = m_widget->getCurrSelectModelID();
		int selectActionID = m_widget->getCurrSelectActionID();
		int skeletonListSize = actionPredictor->getRandomPredictedSkelListSize(selectModelID, selectActionID);
		m_displayedSkelIDList = actionPredictor->getRandomPredictedSkelList(selectModelID, selectActionID);

		if (skeletonListSize > 0)
		{
			int locationListSize = actionPredictor->getNewLocationNumOfSkel(selectModelID, selectActionID, m_displayedSkelIDList[0]);

			m_widget->setPredictSkelListItem(skeletonListSize);
			m_widget->setLocationListItem(locationListSize);

			selectSkelAndUpdateLocationList();
			viewModelInNewLocation();
		}

		// restore other model's location
		Eigen::Matrix4d tempTransMat = Eigen::Matrix4d::Identity();
		for (int model_id = 0; model_id < m_scene->getModelNum(); model_id++)
		{
			CModel *model = m_scene->getModel(model_id);

			if (model_id != selectModelID)
			{
				model->setTempDisplayTransMat(tempTransMat);
			}
		}
	}

	actionPredictor->updateDrawArea();
}

void ActionViewer::selectSkelAndUpdateLocationList()
{
	int selectModelID = m_widget->getCurrSelectModelID();
	int selectActionID = m_widget->getCurrSelectActionID();
	int selectSkelID = m_widget->getCurrSelectSkelID();

	// show selected skel with obb
	for (int i = 0; i < m_displayedSkelIDList.size(); i++)
	{
		if (i == selectSkelID)
		{
			actionPredictor->setSkeletonPicked(selectModelID, selectActionID, m_displayedSkelIDList[i], true);
		}

		else
		{
			actionPredictor->setSkeletonPicked(selectModelID, selectActionID, m_displayedSkelIDList[i], false);
		}
	}
	
	// update location list
	int locationListSize = actionPredictor->getNewLocationNumOfSkel(selectModelID, selectActionID, m_displayedSkelIDList[selectSkelID]);
	m_widget->setLocationListItem(locationListSize);

	actionPredictor->updateDrawArea();
}

void ActionViewer::viewModelInNewLocation()
{
	int selectModelID = m_widget->getCurrSelectModelID();
	int selectActionID = m_widget->getCurrSelectActionID();
	int selectSkelID = m_widget->getCurrSelectSkelID();

	int selectLocationID = m_widget->getCurrSelectLocationID();

	Eigen::Matrix4d tempTransMat = Eigen::Matrix4d::Identity();
	CModel *model = m_scene->getModel(selectModelID);

	if (selectLocationID != 0)
	{		
		MathLib::Vector3 currLocation = model->getCurrentLocation();
		MathLib::Vector3 newLocation = actionPredictor->getNewLocation(selectModelID, selectActionID, m_displayedSkelIDList[selectSkelID], selectLocationID-1);

		MathLib::Vector3 translationVec = newLocation - currLocation;
		Eigen::Affine3d tempTransform(Eigen::Translation3d(Eigen::Vector3d(translationVec.x, translationVec.y, translationVec.z)));

		tempTransMat = tempTransform.matrix();
	}

	model->setTempDisplayTransMat(tempTransMat);

	actionPredictor->updateDrawArea();
}

