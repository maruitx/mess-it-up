#include "ActionViewerWidget.h"
#include "ActionViewer.h"

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


	connect(ui.modelNameListWidget, SIGNAL(itemSelectionChanged()), m_viewer, SLOT(updateDisplayedSkels()));
	connect(ui.actionListWidget, SIGNAL(itemSelectionChanged()), m_viewer, SLOT(updateDisplayedSkels()));

	ui.modelNameListWidget->setCurrentRow(0);
	ui.actionListWidget->setCurrentRow(0);
}

ActionViewerWidget::~ActionViewerWidget()
{

}
