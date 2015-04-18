#ifndef DEPTHSENSOR_H
#define DEPTHSENSOR_H

#include <QObject>
#include <QLabel>
#include "MS_OpenCVFrameHelper.h"
#include "OpenCVHelper.h"
#include "FrameRateTracker.h"
#include "../Utilities/utility.h"
#include "../Geometry/SimplePointCloud.h"

class DepthCloud;

typedef QPair<int, int> FrameLabel;  // <action_id, model_id>

class DepthSensor : public QObject
{
	Q_OBJECT

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	explicit DepthSensor(QObject *parent);
	~DepthSensor();

	HRESULT createFirstConnected();

	// data stream
	void createColorImage();
	void createDepthImage();
	void getFrameSize(int &width, int &height);

	int createStream();
	void clearStream();
	void saveStreamToImgFiles();

	void setColorLabel(QLabel* label) { m_colorLabel = label; };
	void setDepthLabel(QLabel* label) { m_depthLabel = label; };
	void setCurrentMatToLabel();

	void setCurrFrameAs(int n);
	int getLoadFrameNum() { return m_depthMatStream.size(); };

	Mat getColorMat() { return m_colorMat; };
	Mat getDepthMat() { return m_depthMat; };
	int getCurrFrameID() { return m_currentFrameID; };

	void updateColorLabelFromFrame();
	void updateDepthLabelFromFrame();
	void updateColoarLabelFromLoad();
	void updateDepthLabelFromLoad();

	void setDepthNearMode(bool mode);
	void setSkeletonSeatedMode(bool mode);
	void setFilpXYMode(bool mode);
	bool isFlipXY() { return m_bFilpXY; };

	// play option
	int run();
	void setPause(bool state) { m_bIsPaused = state; };

	void loadScan(QString filename);

	void startRecord(QString filename);
	void finishRecord();

	QMutex* mutex() { return &m_mutex; };

	void getCoordinateMapper(INuiCoordinateMapper *m) { m_frameHelper->GetCoordinateMapper(&m); };
	NUI_LOCKED_RECT* getColorLockRect() { return m_frameHelper->GetColorLockRect(); };
	NUI_LOCKED_RECT* getDepthLockRect() { return m_frameHelper->GetDepthLockRect(); };


	// 3d point cloud
	void computePointCloud();
	void savePointCloud(QString filename);
	SimplePointCloud& getPointCloud() {return m_pointCloud; };

	void saveCoordinateMapper();
	void loadCoordinateMapper();

	// skeleton 
	void filpSkeleton(NUI_SKELETON_FRAME &skeletonFrame);
	void saveCurrSkeleton();
	void saveSkeletonFile();
	void loadSkeletonFile();
	QVector<QVector<Vector4>>& getSkeletonStream() { return m_skeletonStream; };

	// timer
	void saveTimeStampFile();
	void loadTimeStampFile();

	// actions
	void loadActionLabelFile();
	void saveActionLabelFile();

	QSet<FrameLabel> getCurrFrameLabel() { return m_frameLabels[m_currentFrameID]; };
	QVector<QSet<FrameLabel>> getFrameActionLabels() { return m_frameLabels; };

	void addLabelToCurrFrame(int actionID, int modelID);
	void clearLabelOfCurrFrame();

	void addLabelToFrames(int fromId, int toId, int actionID, int modelID);
	void clearLabelOfFrames(int fromId, int toId);

	void clearAllLabels();

	//tracking
	void setTrackRefCloud(QVector<SimplePointCloud> &clouds, Eigen::Matrix4d &transMat);
	QVector<QVector<Eigen::Matrix4d>>& getTrackedTransMat() { return m_trackTransMat; };
	void saveTrackedTransMat();
	bool loadTrackedTransMat();
	bool hasTrackedTransMat() { return m_hasTrackedTransMat; };

	void computeTrackResultCloud();

public slots:
	int updateData();
	void playNextFrame();

	void showDepthCloud();
	void resetDepthCloudView();
	void startTracking(bool trackingState);
	void doOfflineTracking();

	void setShowOfflineTrackingResult(int state) { m_showOfflineTrackingResult = (bool)state; };
	bool isShowOfflineTrackingResult() { return m_showOfflineTrackingResult; };

	void deleteDepthCloud();

signals:
	void updateFrame(bool);
	void stopPlayTimer();
	void stopRecordTimer();

	void currFrameChanged();

private:
	QLabel *m_colorLabel;
	QLabel *m_depthLabel;

	Microsoft::KinectBridge::OpenCVFrameHelper *m_frameHelper;
	OpenCVHelper *m_openCVHelper;

	cv::Mat m_colorMat;
	cv::Mat m_depthMat;

	QVector<cv::Mat> m_colorMatStream;
	QVector<cv::Mat> m_depthMatStream;
	int m_currentFrameID;

	NUI_IMAGE_RESOLUTION m_colorResolution;
	NUI_IMAGE_RESOLUTION m_depthResolution;
	int m_width;
	int m_height;

	NUI_SKELETON_FRAME m_skeletonFrame;
	QVector<QVector<Vector4>> m_skeletonStream;

	QVector<QSet<FrameLabel>> m_frameLabels;

	// App settings
	bool m_bIsPaused;

	bool m_bIsDepthNearMode;
	bool m_bFilpXY;
	bool m_bIsSkeletonSeatedMode;
	bool m_bIsTrackSkeleton;

	bool m_bIsRecording;
	int m_recordFrameCount;

	bool m_bIsShowPointCloud;
	bool m_showOfflineTrackingResult;

	// Timing
	FrameRateTracker m_colorFrameRateTracker;
	FrameRateTracker m_depthFrameRateTracker;

	SimpleTimer m_timer;
	QVector<int> m_timeStamp;
	long m_totalElapsedMs;

	// File info
	QString m_scanFileName;
	QString m_scanFilePath;
	int m_loadFrameNum;

	SimplePointCloud m_pointCloud;

	DepthCloud *m_depthCloud;
	QVector<SimplePointCloud> m_trackRefCloud;

	// 1 dim: for each tracked model
	// 2 dim: for each frame
	QVector<QVector<Eigen::Matrix4d>> m_trackTransMat;
	bool m_hasTrackedTransMat;

	INuiCoordinateMapper *m_mapper;
	
	QMutex m_mutex;
};

#endif // DEPTHSENSOR_H
