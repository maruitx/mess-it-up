#ifndef DEPTHCLOUD_H
#define DEPTHCLOUD_H

#include <QObject>

#include <pcl/io/boost.h>
//#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "DepthCloudTracker.h"
#include "../Utilities/utility.h"

class SimplePointCloud;

class DepthCloud : public QObject
{
	Q_OBJECT

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DepthCloud(QObject *parent);
	~DepthCloud();

	DepthCloud();
	DepthCloud(int w, int h);
	
	void convertDepthToPointXYZ(NUI_LOCKED_RECT* depthLockedRect, INuiCoordinateMapper* mapper);
	void convertRGBDepthToPointXYZRGB(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect, INuiCoordinateMapper* mapper);	
	void convertToRGBDCloud(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect);

	void convertRGBDepthToPointXYZRGB(cv::Mat &colorMat, cv::Mat &depthMat, INuiCoordinateMapper* mapper, bool flipMode);

	void draw();
	void resetCamView() { m_resetCamView = true; };
	void resetViewer(pcl::visualization::PCLVisualizer& viewer);

	void setFrameSize(const int &w, const int &h) { m_width = w; m_height = h; };
	void setTrackingState(bool state) { m_isTracking = state; };
	bool isOnlineTracking() { return m_isTracking; };

	void setTrackRefCloud(QVector<SimplePointCloud> clouds);
	void setTrackRefCloud(SimplePointCloud cloud);
	void setTrackResultCloud(QVector<SimplePointCloud> clouds);

	void trackOnce();
	Eigen::Matrix4d getTrackingResultMat() { return m_cloudTracker.getTrackingResultMat(); };

signals:
	void viewerClosed();

private:
	int m_width;
	int m_height;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_refCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_resultCloud;
	pcl::visualization::CloudViewer m_cloudViewer;

	

	long *m_depthToRgbMap;

	//DepthCloudTracker<pcl::PointXYZRGB> *m_cloudTracker;
	DepthCloudTracker<pcl::PointXYZRGB> m_cloudTracker;

	bool m_resetCamView;
	bool m_isTracking;
	bool m_hasTrackingResult;
};


#endif // DEPTHCLOUD_H
