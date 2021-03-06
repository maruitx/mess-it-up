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
	int getCurrSelectSkelID() { return ui.predictEndPhaseListWidget->currentRow(); };
	int getCurrSelectLocationID() { return ui.prediectNewLocationListWidget->currentRow(); };

	void setPredictSkelListItem(int skelNum);
	void setLocationListItem(int locationNum);

	Ui::ActionViewerWidget ui;

public slots:
	void updateShowSkelNumLabel();
	void updateShowSkelNumLabel(int displayNum);
	void refreshSkeleton();
	void repredicting();

private:
	ActionViewer *m_viewer;
	int m_showSampledSkelNum;

};

#endif // ACTIONVIEWERWIDGET_H
