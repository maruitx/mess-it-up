#ifndef ACTIONVIEWERWIDGET_H
#define ACTIONVIEWERWIDGET_H

#include <QWidget>
#include "ui_ActionViewerWidget.h"

class ActionViewer;

class ActionViewerWidget : public QWidget
{
	Q_OBJECT

public:
	ActionViewerWidget(ActionViewer *viewer, QWidget *parent = 0);
	~ActionViewerWidget();

	int getCurrSelectModelID() { return ui.modelNameListWidget->currentRow(); };
	int getCurrSelectActionID() { return ui.actionListWidget->currentRow(); };


	Ui::ActionViewerWidget ui;

private:
	ActionViewer *m_viewer;

};

#endif // ACTIONVIEWERWIDGET_H