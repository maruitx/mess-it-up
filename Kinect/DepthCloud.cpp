#include "DepthCloud.h"
#include "../Geometry/SimplePointCloud.h"

DepthCloud::DepthCloud(QObject *parent)
	: QObject(parent),
	m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
	m_cloudViewer(pcl::visualization::CloudViewer("Point Cloud Viewer"))
{

}

DepthCloud::DepthCloud():
m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
m_cloudViewer(pcl::visualization::CloudViewer("Point Cloud Viewer"))
{

}

DepthCloud::DepthCloud(int w, int h):
m_width(w), m_height(h),
m_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
m_refCloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
m_resultCloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
m_cloudViewer(pcl::visualization::CloudViewer("Point Cloud Viewer"))
//m_cloudTracker(0, 8, 0.01, true, false, true, false)
//m_cloudTracker(DepthCloudTracker<pcl::PointXYZRGB>)
, m_resetCamView(true)
, m_isTracking(false)
, m_hasTrackingResult(false)
{
	m_depthToRgbMap = new long[m_width*m_height * 2];

	m_cloud->width = static_cast<uint32_t>(m_width);
	m_cloud->height = static_cast<uint32_t>(m_height);
	m_cloud->is_dense = false;
	m_cloud->points.resize(m_cloud->width * m_cloud->height);

	m_cloudTracker.init(8, 0.01, true, false, true, false);
}

DepthCloud::~DepthCloud()
{
	if (m_depthToRgbMap)
	{
		delete [] m_depthToRgbMap;
	}
}

void DepthCloud::convertDepthToPointXYZ(NUI_LOCKED_RECT* depthLockedRect, INuiCoordinateMapper* mapper)
{
	NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(depthLockedRect->pBits);

	for (int y = 0; y < m_height; y++){
		for (int x = 0; x < m_width; x++){
			pcl::PointXYZ point;

			NUI_DEPTH_IMAGE_POINT depthPoint;
			depthPoint.x = x;
			depthPoint.y = y;
			depthPoint.depth = depthPixel[y * m_width + x].depth;

			// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
			Vector4 skeletonPoint;
			mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint);

			point.x = skeletonPoint.x/8;
			point.y = skeletonPoint.y/8;
			point.z = skeletonPoint.z/8;

			//m_cloud->push_back(point);
			//m_cloud->points[y*m_width + x] = point;
		}
	}
}

void DepthCloud::convertRGBDepthToPointXYZRGB(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect, INuiCoordinateMapper* mapper)
{
	//NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(depthLockedRect->pBits);
	USHORT* pDepthBuffer = reinterpret_cast<USHORT*>(depthLockedRect->pBits);

	for (int y = 0; y < m_height; y++){
		for (int x = 0; x < m_width; x++){
			pcl::PointXYZRGB point;

			NUI_DEPTH_IMAGE_POINT depthPoint;
			depthPoint.x = x;
			depthPoint.y = y;
			//depthPoint.depth = depthPixel[y * m_width + x].depth;
			depthPoint.depth = pDepthBuffer[y * m_width + x];

			// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
			Vector4 skeletonPoint;
			mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint);

			// value scaled up 8x after kinect sdk 1.6
			point.x = skeletonPoint.x/8;
			point.y = skeletonPoint.y/8;
			point.z = skeletonPoint.z/8;

			// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
			NUI_COLOR_IMAGE_POINT colorPoint;
			mapper->MapDepthPointToColorPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &colorPoint);

			if (0 <= colorPoint.x && colorPoint.x < m_width && 0 <= colorPoint.y && colorPoint.y < m_height){
				unsigned int index = colorPoint.y * colorLockedRect->Pitch + colorPoint.x * 4;
				point.b = colorLockedRect->pBits[index + 0];
				point.g = colorLockedRect->pBits[index + 1];
				point.r = colorLockedRect->pBits[index + 2];
			}

			//m_cloud->push_back(point);
			m_cloud->points[y*m_width + x] = point;
		}
	}
}

void DepthCloud::convertRGBDepthToPointXYZRGB(cv::Mat &colorMat, cv::Mat &depthMat, INuiCoordinateMapper* mapper, bool flipMode)
{
	cv::Mat colorM, depthM;

	if (flipMode == true)
	{
		cv::flip(colorMat, colorM, 1);
		cv::flip(depthMat, depthM, 1);
	}

	else
	{
		colorM = colorMat;
		depthM = depthMat;
	}

	for (int y = 0; y < m_height; y++){
		for (int x = 0; x < m_width; x++){
			USHORT* pDepthRow = depthM.ptr<USHORT>(y);

			pcl::PointXYZRGB point;
			NUI_DEPTH_IMAGE_POINT depthPoint;
			depthPoint.x = x;
			depthPoint.y = y;
			depthPoint.depth = pDepthRow[x];

			// Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
			Vector4 skeletonPoint;
			mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint);

			point.x = skeletonPoint.x/8;
			point.y = skeletonPoint.y/8;
			point.z = skeletonPoint.z/8;

			// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
			NUI_COLOR_IMAGE_POINT colorPoint;
			mapper->MapDepthPointToColorPoint(NUI_IMAGE_RESOLUTION_640x480, &depthPoint, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &colorPoint);

			if (0 <= colorPoint.x && colorPoint.x < m_width && 0 <= colorPoint.y && colorPoint.y < m_height){
				
				cv::Vec3b* pColorRow = colorM.ptr<cv::Vec3b>(colorPoint.y);

				point.b = (BYTE)pColorRow[colorPoint.x][0];
				point.g = (BYTE)pColorRow[colorPoint.x][1];
				point.r = (BYTE)pColorRow[colorPoint.x][2];
			}

			m_cloud->points[y*m_width+x] = point;
		}
	}
}

void DepthCloud::draw()
{
	if (m_resetCamView)
	{
		m_cloudViewer.runOnVisualizationThreadOnce(boost::bind(&DepthCloud::resetViewer, this, _1));
		m_resetCamView = false;
	}

	m_cloudViewer.showCloud(m_cloud);

	if (m_isTracking)
	{
		m_cloudTracker.cloud_cb_ref(m_cloud);
		//m_cloudTracker.cloud_cb(m_cloud);
		m_cloudViewer.runOnVisualizationThread(boost::bind(&DepthCloudTracker<pcl::PointXYZRGB>::viz_cb, &m_cloudTracker, _1), "viz_cb");	
		
		//m_cloudViewer.runOnVisualizationThread(boost::bind(&DepthCloudTracker<pcl::PointXYZRGB>::drawRefCloud, &m_cloudTracker, _1), "drawRefCloud");
	}

	if (m_hasTrackingResult)
	{
		m_cloudViewer.runOnVisualizationThread(boost::bind(&DepthCloudTracker<pcl::PointXYZRGB>::drawOfflineResult, &m_cloudTracker, _1), "drawOfflineResult");
		m_hasTrackingResult = false;
	}
	
	if (!m_cloudViewer.wasStopped())
	{		

	}
}

// need to fix bug
void DepthCloud::convertToRGBDCloud(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect)
{
	m_cloud->clear();

	m_cloud->width = static_cast<uint32_t>(m_width);
	m_cloud->height = static_cast<uint32_t>(m_height);
	m_cloud->is_dense = false;

	const USHORT* currDepth = (const USHORT*)depthLockedRect->pBits;
	const BYTE* startColor = (const BYTE*)colorLockedRect->pBits;

	long* depth2rgb = (long*)m_depthToRgbMap;

	for (int j = 0; j < m_height; ++j) {
		for (int i = 0; i < m_width; ++i) {
			pcl::PointXYZRGB point;

			// Get depth of pixel in millimeters
			USHORT depth = NuiDepthPixelToDepth(*currDepth);
			// Store coordinates of the point corresponding to this pixel
			Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, *currDepth);
			point.x = pos.x / pos.w;
			point.y = pos.y / pos.w;
			point.z = pos.z / pos.w;
			// Store the index into the color array corresponding to this pixel
			NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
				NUI_IMAGE_RESOLUTION_640x480, // color frame resolution
				NUI_IMAGE_RESOLUTION_640x480, // depth frame resolution
				NULL,                         // pan/zoom of color image (IGNORE THIS)
				i, j, *currDepth,                  // Column, row, and depth in depth image
				depth2rgb, depth2rgb + 1        // Output: column and row (x,y) in the color image
				);

			long shift_x = *depth2rgb;
			long shift_y = *(depth2rgb + 1);
			// If out of bounds, then do not color this pixel
			if (shift_x < 0 || shift_y < 0 || shift_x > m_width || shift_y > m_height) {
				point.b = 0;
				point.g = 0;
				point.r = 0;
			}
			else {
				// Determine rgb color for depth pixel (i,j) from color pixel (x,y)
				const BYTE* color = startColor + (shift_x + m_width*shift_y) * 4;
				point.b = color[0];
				point.g = color[1];
				point.r = color[2];
			}

			depth2rgb += 2;
			*currDepth++;

			m_cloud->push_back(point);
		}
	}
}

void DepthCloud::resetViewer(pcl::visualization::PCLVisualizer& viewer)
{
	//viewer.resetCameraViewpoint();
	viewer.setCameraPosition(0,0,-5,0,0,0,0,1,0);
	viewer.updateCamera();
}

void DepthCloud::setTrackRefCloud(QVector<SimplePointCloud> clouds)
{
	m_refCloud->is_dense = false;

	int ptNum = 0;

	for (int i = 0; i < clouds.size();i++)	
	{
		ptNum += clouds[i].size();
	}

	m_refCloud->points.resize(ptNum);
	
	int id = 0;
	for (int i = 0; i < clouds.size(); i++)
	{
		for (int j = 0; j < clouds[i].size(); j++)
		{
			pcl::PointXYZRGB point;
			point.x = clouds[i][j][0];
			point.y = clouds[i][j][1];
			point.z = clouds[i][j][2];

			m_refCloud->points[id] = point;
			id++;
		}
	}

	m_cloudTracker.setRefCloud(m_refCloud);

	m_cloudTracker.matchRefCloudToDepthCloud(m_cloud);
}

void DepthCloud::setTrackRefCloud(SimplePointCloud cloud)
{
	m_refCloud->is_dense = false;

	int ptNum = cloud.size();

	m_refCloud->points.resize(ptNum);

	int id = 0;
	for (int i = 0; i < cloud.size(); i++)
	{
		pcl::PointXYZRGB point;
		point.x = cloud[i][0];
		point.y = cloud[i][1];
		point.z = cloud[i][2];

		m_refCloud->points[id] = point;
		id++;
	}

	m_cloudTracker.setRefCloud(m_refCloud);
	m_cloudTracker.matchRefCloudToDepthCloud(m_cloud);
}

void DepthCloud::trackOnce()
{
	m_cloudTracker.cloud_cb_ref(m_cloud);	
}

void DepthCloud::setTrackResultCloud(QVector<SimplePointCloud> clouds)
{
	m_resultCloud->is_dense = false;

	int ptNum = 0;

	for (int i = 0; i < clouds.size(); i++)
	{
		ptNum += clouds[i].size();
	}

	m_resultCloud->points.resize(ptNum);


	int id = 0;
	for (int i = 0; i < clouds.size(); i++)
	{
		for (int j = 0; j < clouds[i].size(); j++)
		{
			pcl::PointXYZRGB point;
			point.x = clouds[i][j][0];
			point.y = clouds[i][j][1];
			point.z = clouds[i][j][2];

			
			point.r = ColorSet[clouds[i].getID()%16][0];
			point.g = ColorSet[clouds[i].getID()%16][1];
			point.b = ColorSet[clouds[i].getID()%16][2];

			m_resultCloud->points[id] = point;
			id++;
		}
	}

	m_cloudTracker.setResultCloud(m_resultCloud);

	m_hasTrackingResult = true;
}
