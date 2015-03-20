#ifndef ACTIONLABELERWIDGET_H
#define ACTIONLABELERWIDGET_H

#include <QWidget>
#include "ui_ActionLabelerWidget.h"

class ActionLabeler;
typedef QPair<int, int> FrameLabel;

class ActionLabelerWidget : public QWidget
{
	Q_OBJECT

public:
	ActionLabelerWidget(ActionLabeler *labeler, QWidget *parent = 0);
	~ActionLabelerWidget();

	void setCurrFrameID(int i);
	void setCurrFrameLabel(QSet<int> labels);
	void setCurrFrameLabel(QSet<FrameLabel> labels);

	int getCurrSelectActionID() { return ui->actionListWidget->currentRow(); };
	QVector<QString> getSelectModelNames();

	Ui::ActionLabelerWidget *ui;
	QVector<QString> selectModelNames;

private:
	ActionLabeler *m_labeler;
	
};

#endif // ACTIONLABELERWIDGET_H
