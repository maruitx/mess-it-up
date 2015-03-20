#include <QMainWindow>
#include <QThread>

#include "RgbdViewer.h"
#include "RgbdViewerWidget.h"

#include "../Geometry/Scene.h"

RgbdViewer::RgbdViewer(QObject *parent)
	: QObject(parent)
{
	m_widget = NULL;
	m_dock = NULL;
	depthSensor = NULL;

	m_hasWidget = false;

	m_liveScanTimer = NULL;
	m_playTimer = NULL;

//	m_surfaceMatcher = NULL;
}

RgbdViewer::~RgbdViewer()
{

}

void RgbdViewer::startScan()
{
	if (m_playTimer)
	{
		m_playTimer->stop();
		delete m_playTimer;

		m_playTimer = NULL;

		depthSensor->clearStream();
		depthSensor->setPause(false);
	}

	if (m_liveScanTimer)
	{
		m_liveScanTimer->stop();
		delete m_liveScanTimer;
		m_liveScanTimer = NULL;

		depthSensor->clearStream();
		depthSensor->setPause(false);
	}

	depthSensor->createStream();
	depthSensor->createFirstConnected();

	m_widget->ui->startRecordButton->setEnabled(true);

	// update FPS should be larger than real FPS since need process time for getting and showing stream
	m_updateFPS = m_widget->ui->fpsSlider->value();
	
	m_liveScanTimer = new QTimer(this);
	m_liveScanTimer->setSingleShot(false);
	m_liveScanTimer->setInterval((1.0 / m_updateFPS)*1000.0);

	/// Timed events, first work, then display
	connect(m_liveScanTimer, SIGNAL(timeout()), depthSensor, SLOT(updateData()));
	m_liveScanTimer->start();
}

void RgbdViewer::setFPS(int f)
{
	m_updateFPS = f;

	if (m_liveScanTimer)
	{
		m_liveScanTimer->setInterval((1.0 / m_updateFPS)*1000.0);
	}
}

void RgbdViewer::loadScan()
{
	if (m_liveScanTimer)
	{
		m_liveScanTimer->stop();
		delete m_liveScanTimer;

		m_liveScanTimer = NULL;

		depthSensor->clearStream();
	}

	depthSensor->createStream();
	depthSensor->loadScan(m_widget->loadScanName());

	enableFrameBrowser();
}

void RgbdViewer::loadScan(QString filename)
{
	if (m_liveScanTimer)
	{
		m_liveScanTimer->stop();
		delete m_liveScanTimer;

		m_liveScanTimer = NULL;

		depthSensor->clearStream();
	}

	depthSensor->createStream();
	depthSensor->loadScan(filename);

	enableFrameBrowser();
}

void RgbdViewer::createWidget()
{
	depthSensor = new DepthSensor(this);
	m_widget = new RgbdViewerWidget(this);
	m_dock = new QDockWidget("RGBD Viewer");


	m_dock->setWidget(m_widget);
	parentMainWin->addDockWidget(Qt::BottomDockWidgetArea, m_dock);

	depthSensor->setColorLabel(m_widget->colorLabel());
	depthSensor->setDepthLabel(m_widget->depthLabel());
	m_hasWidget = true;

	m_widget->ui->startRecordButton->setDisabled(true);
}

void RgbdViewer::updateWidget()
{
	m_widget->repaint();
}

void RgbdViewer::startRecord()
{
	depthSensor->startRecord(m_widget->saveRecordName());
	m_widget->ui->startRecordButton->setChecked(true);
	m_widget->ui->finishRecordButton->setEnabled(true);
}

void RgbdViewer::finishRecord()
{ 
	depthSensor->finishRecord();
	m_widget->ui->finishRecordButton->setDisabled(true);
	m_widget->ui->startRecordButton->setChecked(false);
}

void RgbdViewer::playLoadScan(bool state)
{
	// set FPS for timer, real FPS will adapt to the time stamp
	int playFPS = 60;

	if (!m_playTimer)
	{
		m_playTimer = new QTimer(this);
		m_playTimer->setSingleShot(false);
		m_playTimer->setInterval((1.0 / playFPS)*1000.0);
	}

	else
	{
		m_playTimer->stop();
		m_playTimer->setSingleShot(false);
		m_playTimer->setInterval((1.0 / playFPS)*1000.0);
	}

	if (state)
	{
		depthSensor->setPause(false);

		/// Timed events, first work, then display
		connect(m_playTimer, SIGNAL(timeout()), depthSensor, SLOT(playNextFrame()));
		connect(m_playTimer, SIGNAL(timeout()), this, SLOT(updateFrameBrowser()));
		connect(depthSensor, SIGNAL(stopPlayTimer()), this, SLOT(stopPlayTimer()));

		m_playTimer->start();
		m_widget->ui->playLoadScanButton->setText(QString("Stop"));
	}

	else
	{
		depthSensor->setPause(true);
		m_playTimer->stop();

		m_widget->ui->playLoadScanButton->setText(QString("Play"));
	}
}

void RgbdViewer::saveCurrPointCloud()
{
	depthSensor->computePointCloud();
	depthSensor->savePointCloud(m_widget->savePointCloudName());
}

void RgbdViewer::changeDepthNearMode(int state)
{
	bool mode;

	switch (state)
	{
	case Qt::Checked:
		mode = true;
		break;
	case  Qt::Unchecked:
		mode = false;
		break;

	default:
		break;
	}

	depthSensor->setDepthNearMode(mode);
}

void RgbdViewer::changeSkeletonSeatedMode(int state)
{
	bool mode;

	switch (state)
	{
	case Qt::Checked:
		mode = true;
		break;
	case  Qt::Unchecked:
		mode = false;
		break;

	default:
		break;
	}

	depthSensor->setSkeletonSeatedMode(mode);
}

void RgbdViewer::changeFlipXYMode(int state)
{
	bool mode;

	switch (state)
	{
	case Qt::Checked:
		mode = true;
		break;
	case  Qt::Unchecked:
		mode = false;
		break;

	default:
		break;
	}

	depthSensor->setFilpXYMode(mode);
}

void RgbdViewer::pauseScan()
{
	if (m_liveScanTimer)
	{
		m_liveScanTimer->stop();
	}

	depthSensor->setPause(true);
}

void RgbdViewer::onWidgetClosed()
{
	if (depthSensor)
	{
		delete depthSensor;
	}
}

void RgbdViewer::enableFrameBrowser()
{
	m_widget->ui->frameBrowseSlider->setEnabled(true);

	m_widget->ui->frameBrowseSlider->setMinimum(1);
	m_widget->ui->frameBrowseSlider->setMaximum(depthSensor->getLoadFrameNum());
}

void RgbdViewer::changeToFrame(int i)
{
	depthSensor->setCurrFrameAs(i-1);

}

void RgbdViewer::updateFrameBrowser()
{
	m_widget->ui->frameBrowseSlider->setValue(depthSensor->getCurrFrameID());
}
