#include "ActionViewerWidget.h"
#include "ActionViewer.h"
#include "ActionPredictor.h"

ActionViewerWidget::ActionViewerWidget(ActionViewer *viewer, QWidget *parent)
	: m_viewer(viewer), QWidget(parent)
{
	ui.setupUi(this);



	for (int i = 0; i < m_viewer->allModelNameList.size(); i++)
	{
		ui.modelNameListWidget->addItem(m_viewer->allModelNameList[i]);
	}

	for (int i = 0; i < ACTION_NUM; i++)
	{
		ui.actionListWidget->addItem(QString(Action_Labels[i]));
	}

	ui.modelNameListWidget->setCurrentRow(0);
	ui.actionListWidget->setCurrentRow(0);
	m_showSampledSkelNum = ui.showSkelNumSlider->value();
	updateShowSkelNumLabel();

	connect(ui.modelNameListWidget, SIGNAL(itemSelectionChanged()), m_viewer->actionPredictor, SLOT(updateDrawArea()));
	connect(ui.actionListWidget, SIGNAL(itemSelectionChanged()), m_viewer->actionPredictor, SLOT(updateDrawArea()));
	connect(ui.modelNameListWidget, SIGNAL(itemSelectionChanged()), m_viewer, SLOT(setCenterModelID()));

	connect(ui.modelNameListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(updateShowSkelNumLabel()));
	connect(ui.actionListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(updateShowSkelNumLabel()));

	connect(ui.showSkelNumSlider, SIGNAL(valueChanged(int)), this, SLOT(updateShowSkelNumLabel(int)));
	connect(ui.resampleSkelButton, SIGNAL(clicked()), this, SLOT(resampleSkeleton()));

	connect(ui.showCenterModelVoxelBox, SIGNAL(stateChanged(int)), m_viewer, SLOT(setShowModelVoxel(int)));	
}

ActionViewerWidget::~ActionViewerWidget()
{

}

void ActionViewerWidget::updateShowSkelNumLabel()
{
	int modelID = ui.modelNameListWidget->currentRow();
	int actionID = ui.actionListWidget->currentRow();

	int totalSkelNum = m_viewer->actionPredictor->getSampledSkelNum(modelID, actionID);

	// update label
	QString str("Skel Num: ");
	str += QString("%1 of %2 skels").arg(m_showSampledSkelNum).arg(totalSkelNum);

	ui.showSkelNumLabel->setText(str);

	// reset slider postion
	ui.showSkelNumSlider->setValue(m_showSampledSkelNum);
}

// only to indicate current slider value, don't actually change sample num
void ActionViewerWidget::updateShowSkelNumLabel(int displayValue)
{
	int modelID = ui.modelNameListWidget->currentRow();
	int actionID = ui.actionListWidget->currentRow();

	int totalSkelNum = m_viewer->actionPredictor->getSampledSkelNum(modelID, actionID);

	// update label
	QString str("Skel Num: ");
	str += QString("%1 of %2 skels").arg(displayValue).arg(totalSkelNum);

	ui.showSkelNumLabel->setText(str);
}

void ActionViewerWidget::resampleSkeleton()
{
	int newSkelNum = ui.showSkelNumSlider->value();

	m_viewer->actionPredictor->resampleSkeleton(newSkelNum);

	m_showSampledSkelNum = newSkelNum;
	updateShowSkelNumLabel();

}
