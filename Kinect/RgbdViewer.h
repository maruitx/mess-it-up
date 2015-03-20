#ifndef KINECTVIEWER_H
#define KINECTVIEWER_H

#include <QObject>
#include <QDockWidget>
#include <QTimer>

#include <opencv2/core/core.hpp>
#include "StarlabMainWindow.h"

#include "DepthSensor.h"

class RgbdViewerWidget;
class CScene;

class RgbdViewer : public QObject
{
	Q_OBJECT

public:
	RgbdViewer(QObject *parent);
	~RgbdViewer();

	bool hasWidget() { return m_hasWidget; };

	StarlabMainWindow *parentMainWin;

	DepthSensor *depthSensor;

public slots:
	
	void createWidget();

	void startScan();
	void pauseScan();
	void setFPS(int f);

	void loadScan();
	void loadScan(QString filename);
	void playLoadScan(bool state);

	void startRecord();
	void finishRecord();

	void stopPlayTimer() { m_playTimer->stop(); };

	// control option
	void changeFlipXYMode(int state);
	void changeDepthNearMode(int state);
	void changeSkeletonSeatedMode(int state);

	void enableFrameBrowser();
	void updateFrameBrowser();
	void changeToFrame(int n);

	void saveCurrPointCloud();

	void updateWidget();
	void onWidgetClosed();

private:
	RgbdViewerWidget *m_widget;
	bool m_hasWidget;
	QDockWidget * m_dock;

	QTimer* m_liveScanTimer;
	QTimer* m_playTimer;

	int m_updateFPS; // fps set to the timer 
};

#endif // KINECTVIEWER_H
