#ifndef KINECTVIEWERWIDGET_H
#define KINECTVIEWERWIDGET_H

#include <QWidget>
#include <QString>
#include "ui_RgbdViewerWidget.h"

#include <opencv2/core/core.hpp>

class RgbdViewer;

class RgbdViewerWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RgbdViewerWidget(RgbdViewer *v, QWidget *parent = 0);
	~RgbdViewerWidget();

	QLabel* colorLabel() {return ui->colorLabel; };
	QLabel* depthLabel() {return ui->depthLabel; };

	QString loadScanName();
	QString saveRecordName();

	QString savePointCloudName();

	Ui::RgbdViewerWidget *ui;

private:
	RgbdViewer *m_viewer;
};

#endif // KINECTVIEWERWIDGET_H
