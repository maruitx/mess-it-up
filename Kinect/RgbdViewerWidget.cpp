#include "RgbdViewerWidget.h"
#include "RgbdViewer.h"
#include "MS_OpenCVFrameHelper.h"

#include <QFileDialog>

RgbdViewerWidget::RgbdViewerWidget(RgbdViewer *v, QWidget *parent /*= 0*/)
	: QWidget(parent), ui(new Ui::RgbdViewerWidget), m_viewer(v)
{
	ui->setupUi(this);

	// io options
	connect(ui->startScanButton, SIGNAL(clicked()), m_viewer, SLOT(startScan()));
	connect(ui->stopScanButton, SIGNAL(clicked()), m_viewer, SLOT(pauseScan()));
	connect(ui->fpsSlider, SIGNAL(valueChanged(int)), m_viewer, SLOT(setFPS(int)));

	connect(ui->startRecordButton, SIGNAL(clicked()), m_viewer, SLOT(startRecord()));
	connect(ui->finishRecordButton, SIGNAL(clicked()), m_viewer, SLOT(finishRecord()));

	connect(ui->loadScanButton, SIGNAL(clicked()), m_viewer, SLOT(loadScan()));
	connect(ui->playLoadScanButton, SIGNAL(toggled(bool)), m_viewer, SLOT(playLoadScan(bool)));

	// stream control
	connect(ui->flipXYBox, SIGNAL(stateChanged(int)), m_viewer, SLOT(changeFlipXYMode(int)));
	connect(ui->depthNearModeBox, SIGNAL(stateChanged(int)), m_viewer, SLOT(changeDepthNearMode(int)));
	connect(ui->skeletonModeBox, SIGNAL(stateChanged(int)), m_viewer, SLOT(changeSkeletonSeatedMode(int)));

	// frame browse
	connect(ui->frameBrowseSlider, SIGNAL(valueChanged(int)), m_viewer, SLOT(changeToFrame(int)));

	// point cloud
	connect(ui->showPointCloudButton, SIGNAL(clicked()), m_viewer->depthSensor, SLOT(showDepthCloud()));
	connect(ui->savePointCloudButton, SIGNAL(clicked()), m_viewer, SLOT(saveCurrPointCloud()));

	connect(ui->resetCloudViewerButton, SIGNAL(clicked()), m_viewer->depthSensor, SLOT(resetDepthCloudView()));
	connect(ui->startTrackingButton, SIGNAL(toggled(bool)), m_viewer->depthSensor, SLOT(startTracking(bool)));

	connect(ui->doOfflineTrackingButton, SIGNAL(clicked()), m_viewer->depthSensor, SLOT(doOfflineTracking()));
	connect(ui->showOfflineResultBox, SIGNAL(stateChanged(int)), m_viewer->depthSensor, SLOT(setShowOfflineTrackingResult(int)));

	// initial settings
	ui->finishRecordButton->setDisabled(true);
	ui->frameBrowseSlider->setDisabled(true);
}

RgbdViewerWidget::~RgbdViewerWidget()
{
	m_viewer->onWidgetClosed();
	delete ui;
}

QString RgbdViewerWidget::loadScanName()
{
	QString filename = QFileDialog::getOpenFileName(0, tr("Load Scan"),
		m_viewer->parentMainWin->settings()->getString("lastUsedDirectory"),
		tr("Scan File (*.txt)"));

	if (filename.isEmpty()) return "";

	// Keep folder active
	QFileInfo fileInfo(filename);
	m_viewer->parentMainWin->settings()->set("lastUsedDirectory", fileInfo.absolutePath());

	return filename;
}

QString RgbdViewerWidget::saveRecordName()
{
	QString filename = QFileDialog::getSaveFileName(0, tr("Save Scan as"),
		m_viewer->parentMainWin->settings()->getString("lastUsedDirectory"),
		tr("Scan File (*.txt)"));

	// Keep folder active
	QFileInfo fileInfo(filename);
	m_viewer->parentMainWin->settings()->set("lastUsedDirectory", fileInfo.absolutePath());

	return filename;
}

QString RgbdViewerWidget::savePointCloudName()
{
	QString filename = QFileDialog::getSaveFileName(0, tr("Save Point Cloud as Ply file"),
		m_viewer->parentMainWin->settings()->getString("lastUsedDirectory"),
		tr("Point Cloud File (*.ply)"));

	// Keep folder active
	QFileInfo fileInfo(filename);
	m_viewer->parentMainWin->settings()->set("lastUsedDirectory", fileInfo.absolutePath());

	return filename;
}
