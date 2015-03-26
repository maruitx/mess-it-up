#include "DepthSensor.h"
#include "RgbdViewer.h"

#include "DepthCloud.h"

#include <QSet>

DepthSensor::DepthSensor(QObject *parent): QObject(parent),
m_currentFrameID(0),
m_bIsPaused(false),
m_bIsDepthNearMode(false),
m_bFilpXY(true),
m_bIsSkeletonSeatedMode(false),
m_bIsTrackSkeleton(true),
m_bIsRecording(false),
m_recordFrameCount(0),
m_bIsShowPointCloud(false),
m_showOfflineTrackingResult(false),
m_depthCloud(NULL),
m_hasTrackedTransMat(false),
m_mapper(NULL)
{
	m_frameHelper = new Microsoft::KinectBridge::OpenCVFrameHelper();
	m_openCVHelper = new OpenCVHelper();

	m_colorFrameRateTracker = FrameRateTracker();
	m_depthFrameRateTracker = FrameRateTracker();

	connect(m_depthCloud, SIGNAL(viewerClosed()), this, SLOT(deleteDepthCloud()));
}

DepthSensor::~DepthSensor()
{
	delete m_frameHelper;
	delete m_openCVHelper;

	if (m_depthCloud)
	{
		delete m_depthCloud;
	}
}

HRESULT DepthSensor::createFirstConnected()
{
	// If Kinect is already initialized, return
	if (m_frameHelper->IsInitialized())
	{
		return S_OK;
	}

	HRESULT hr;

	// Get number of Kinect sensors
	int sensorCount = 0;
	hr = NuiGetSensorCount(&sensorCount);
	if (FAILED(hr))
	{
		return hr;
	}

	// If no sensors, update status bar to report failure and return
	if (sensorCount == 0)
	{
		Simple_Message_Box(QString("Not ready Kinect is found"));
		return E_FAIL;
	}

	// Iterate through Kinect sensors until one is successfully initialized
	for (int i = 0; i < sensorCount; ++i)
	{
		INuiSensor* sensor = NULL;
		hr = NuiCreateSensorByIndex(i, &sensor);
		if (SUCCEEDED(hr))
		{
			hr = m_frameHelper->Initialize(sensor);
			if (SUCCEEDED(hr))
			{
				m_frameHelper->GetCoordinateMapper(&m_mapper);

				// Report success
				return S_OK;
			}
			else
			{
				// Uninitialize KinectHelper to show that Kinect is not ready
				m_frameHelper->UnInitialize();
			}
		}
	}

	// Report failure
	Simple_Message_Box(QString("Not ready Kinect is found"));
	return E_FAIL;
}

void DepthSensor::createColorImage()
{
	m_frameHelper->SetColorFrameResolution(NUI_IMAGE_RESOLUTION_640x480);
	m_colorResolution = NUI_IMAGE_RESOLUTION_640x480; 

	DWORD width, height;
	m_frameHelper->GetColorFrameSize(&width, &height);

	Size size(width, height);
	m_colorMat.create(size, m_frameHelper->COLOR_TYPE);
}

void DepthSensor::createDepthImage()
{
	m_frameHelper->SetDepthFrameResolution(NUI_IMAGE_RESOLUTION_640x480);
	m_depthResolution = NUI_IMAGE_RESOLUTION_640x480;

	DWORD width, height;
	m_frameHelper->GetDepthFrameSize(&width, &height);

	Size size(width, height);
	//m_depthMat.create(size, m_frameHelper->DEPTH_RGB_TYPE);
	m_depthMat.create(size, m_frameHelper->DEPTH_TYPE);

	m_width = (int)width;
	m_height = (int)height;
}

int DepthSensor::createStream()
{
	createColorImage();
	createDepthImage();

	m_skeletonFrame = NUI_SKELETON_FRAME();	

	return 0;
}

void DepthSensor::getFrameSize(int &width, int &height)
{
	DWORD w, h;
	m_frameHelper->GetDepthFrameSize(&w, &h);
	width = (int)w;
	height = (int)h;
}

void DepthSensor::clearStream()
{
	m_colorMat.release();
	m_depthMat.release();

	m_colorLabel->clear();
	m_depthLabel->clear();
}

int DepthSensor::updateData()
{
	// Update image outputs
	if (m_frameHelper->IsInitialized())
	{
		m_mutex.lock();

		// Update skeleton frame
		if (m_bIsTrackSkeleton && !m_bIsPaused
			&& SUCCEEDED(m_frameHelper->UpdateSkeletonFrame()))
		{
			m_frameHelper->GetSkeletonFrame(&m_skeletonFrame);
		
			if (m_bFilpXY)
			{
				filpSkeleton(m_skeletonFrame);
			}
		}

		// Update color frame
		if (!m_bIsPaused && SUCCEEDED(m_frameHelper->UpdateColorFrame()))
		{

			HRESULT hr = m_frameHelper->GetColorImage(&m_colorMat);

			// flip the frame so that kinect will act as an eye instead of a mirror
			if (m_bFilpXY)
			{
				cv::flip(m_colorMat, m_colorMat, 1);
			}		

			// Notify frame rate tracker that new frame has been rendered
			m_colorFrameRateTracker.Tick();

			if (m_bIsRecording)
			{
				m_timeStamp.push_back(m_timer.elipse_ms());
				m_colorMatStream.push_back(m_colorMat.clone());
			}

			// Update bitmap for drawing
			updateColorLabelFromFrame();
		}

		// Update depth frame
		if (!m_bIsPaused && SUCCEEDED(m_frameHelper->UpdateDepthFrame()))
		{
			//HRESULT hr = m_frameHelper->GetDepthImageAsArgb(&m_depthMat);
			HRESULT hr = m_frameHelper->GetDepthImage(&m_depthMat);
			if (m_bFilpXY)
			{
				cv::flip(m_depthMat, m_depthMat, 1);
			}

			// Notify frame rate tracker that new frame has been rendered
			m_depthFrameRateTracker.Tick();

			if (m_bIsRecording)
			{
				m_depthMatStream.push_back(m_depthMat.clone());
				
				if (m_bIsTrackSkeleton)
				{
					saveCurrSkeleton();
				}
				
				m_recordFrameCount++;
			}

			// Update bitmap for drawing
			updateDepthLabelFromFrame();

		}

		if (m_bIsShowPointCloud&&!m_bIsPaused && SUCCEEDED(m_frameHelper->UpdateColorFrame() && SUCCEEDED(m_frameHelper->UpdateDepthFrame())))
		{
			m_depthCloud->convertRGBDepthToPointXYZRGB(m_frameHelper->GetColorLockRect(), m_frameHelper->GetDepthLockRect(), m_mapper);	
			m_depthCloud->draw();
		}

		m_mutex.unlock();
	}

	return 0;
}

void DepthSensor::loadScan(QString filename)
{
	QFile inFile(filename);
	QFileInfo inFileInfo(inFile.fileName());
	m_scanFileName = inFileInfo.baseName();
	m_scanFilePath = inFileInfo.absolutePath() + "/";
	
	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	QString optionNameStr;
	int optionValue;
	ifs >> optionNameStr;

	while (!optionNameStr.isEmpty())
	{
		if (optionNameStr == "NumFrame")
		{
			ifs >> m_loadFrameNum;
		}

		else if (optionNameStr == "IsFlipXY")
		{
			ifs >> optionValue;
			m_bFilpXY = optionValue;
		}

		else if (optionNameStr == "IsSkeletonSeatedMode")
		{
			ifs >> optionValue;
			m_bIsSkeletonSeatedMode = optionValue;
		}

		else if (optionNameStr == "IsDepthNearMode")
		{
			ifs >> optionValue;
			m_bIsDepthNearMode = optionValue;
		}

		else if (optionNameStr == "IsWithSkeleton")
		{
			ifs >> optionValue;
			m_bIsTrackSkeleton = optionValue;
		}
		
		ifs >> optionNameStr;
	}

	inFile.close();

	// load skeleton
	if (m_bIsTrackSkeleton)
	{
		loadSkeletonFile();
	}
	
	// load images
	QString extName = "png";
	cv::VideoCapture colorStream, depthStream;

	colorStream.open(QString(m_scanFilePath + m_scanFileName + "/" + "color_%4d." + extName).toStdString());
	depthStream.open(QString(m_scanFilePath + m_scanFileName + "/" + "depth_%4d." + extName).toStdString());

	if (!colorStream.isOpened() || !depthStream.isOpened())
	{
		Simple_Message_Box(QString("Cannot open record scan file"));
		return;
	}

	m_colorMatStream.clear();
	m_depthMatStream.clear();

	Mat newColor, newDepth;
	while (colorStream.read(newColor) && depthStream.read(newDepth))
	{
		m_colorMatStream.push_back(newColor.clone());
		m_depthMatStream.push_back(newDepth.clone());
	}

	m_currentFrameID = 0;
	setCurrentMatToLabel();

	loadTimeStampFile();
	loadCoordinateMapper();
	loadActionLabelFile();
}

void DepthSensor::playNextFrame()
{
	const double updateInv = 1000.0 / 60;
	double residue = 0;

	if (m_currentFrameID < m_colorMatStream.size() && !m_bIsPaused)
	{
		if (m_currentFrameID > 0)
		{
			m_totalElapsedMs += m_timeStamp[m_currentFrameID];
			residue = (m_totalElapsedMs / updateInv + 1)*updateInv-m_totalElapsedMs;
			Sleep(m_timeStamp[m_currentFrameID]-residue);
		}
		
		setCurrentMatToLabel();
		m_currentFrameID++;	
	}

	else
	{
		m_currentFrameID = 0;
		m_bIsPaused = true;

		m_colorFrameRateTracker.Stop();
		m_depthFrameRateTracker.Stop();

		setCurrentMatToLabel();

		//saveTrackedTransMat();
		emit stopPlayTimer();
	}
}

void DepthSensor::startRecord(QString filename)
{
	QFile scanFile(filename);
	QFileInfo scanFileInfo(scanFile.fileName());
	m_scanFileName = scanFileInfo.baseName();
	m_scanFilePath = scanFileInfo.absolutePath() + "/";
	
	if (QDir().mkdir(m_scanFilePath + m_scanFileName + "/"))
	{
		m_bIsRecording = true;
		m_recordFrameCount = 0;

		m_colorMatStream.clear();
		m_depthMatStream.clear();
		m_skeletonStream.clear();
		m_timeStamp.clear();
		m_timer.start();

		saveCoordinateMapper();
	}

	else
	{
		Simple_Message_Box(QString("Folder exists\n Use another filename"));
	}
}

void DepthSensor::finishRecord()
{
	m_bIsRecording = false;

	QString filename = m_scanFilePath + m_scanFileName + ".txt";
	QFile scan_file(filename);
	QTextStream out(&scan_file);

	if (!scan_file.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	out << "NumFrame " << m_recordFrameCount<<"\n";
	out << "IsFlipXY " << m_bFilpXY << "\n";
	out << "IsSkeletonSeatedMode " << m_bIsSkeletonSeatedMode << "\n";
	out << "IsDepthNearMode " << m_bIsDepthNearMode <<"\n";
	out << "IsWithSkeleton " << m_bIsTrackSkeleton;

	scan_file.close();

	Simple_Message_Box(QString("Record %1 s, %2 frames\nStart saving files...").arg(m_timer.elipse_s()).arg(m_recordFrameCount));

	saveTimeStampFile();

	if (m_bIsTrackSkeleton)
	{
		saveSkeletonFile();
	}

	saveStreamToImgFiles();

	Simple_Message_Box(QString("Streamed files saved"));
}

void DepthSensor::setCurrentMatToLabel()
{
	m_colorMat = m_colorMatStream[m_currentFrameID];
	m_depthMat = m_depthMatStream[m_currentFrameID];
	
	m_colorFrameRateTracker.Tick();
	m_depthFrameRateTracker.Tick();

	// Update bitmap for drawing
	updateColoarLabelFromLoad();

	// Update bitmap for drawing
	updateDepthLabelFromLoad();

	//
	if (m_bIsShowPointCloud)
	{
		m_depthCloud->convertRGBDepthToPointXYZRGB(m_colorMat, m_depthMat, m_mapper, m_bFilpXY);
		m_depthCloud->draw();
	}

	if (m_bIsShowPointCloud && m_depthCloud->isOnlineTracking())
	{
		m_trackTransMat[0][m_currentFrameID] = m_depthCloud->getTrackingResultMat();
	}

	emit updateFrame(true);
}

void DepthSensor::setCurrFrameAs(int n)
{
	m_currentFrameID = n;

	setCurrentMatToLabel();

	emit currFrameChanged();
}

void DepthSensor::computePointCloud()
{
	DWORD width, height;
	m_frameHelper->GetColorFrameSize(&width, &height);

	m_pointCloud.resize(width*height);
	int id = 0;

	m_bIsPaused = true;

	Mat tempMat;
	cv::flip(m_depthMat, tempMat, 1);

	for (int row = 0; row < height; row++)
	{
		USHORT* pDepthRow = tempMat.ptr<USHORT>(row);

		for (int col = 0; col < width; col++)
		{
			USHORT depth = pDepthRow[col];
			
			// Get depth of pixel in millimeters
			//USHORT depth = NuiDepthPixelToDepth(pDepthRow[col]);

			// Store coordinates of the point corresponding to this pixel
			Vector4 pos = NuiTransformDepthImageToSkeleton(col, row, depth, NUI_IMAGE_RESOLUTION_640x480);
			Surface_mesh::Point pt(pos.x/pos.w, pos.y/pos.w, pos.z/pos.w);
			m_pointCloud[id] = pt;
			id++;
		}
	}

	m_bIsPaused = false;
}

void DepthSensor::savePointCloud(QString filename)
{
	m_pointCloud.saveAsPly(filename);
}

void DepthSensor::setDepthNearMode(bool mode)
{
	m_bIsDepthNearMode = mode;

	HRESULT hr = m_frameHelper->SetDepthStreamFlag(NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE, m_bIsDepthNearMode);

	if (hr == E_NUI_HARDWARE_FEATURE_UNAVAILABLE)
	{
		Simple_Message_Box(QString("Near mode is not supported"));
	}
}

void DepthSensor::setSkeletonSeatedMode(bool mode)
{
	m_bIsSkeletonSeatedMode = mode;

	// Update skeleton tracking flag, checking for failures
	HRESULT hr = m_frameHelper->SetSkeletonTrackingFlag(NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT, m_bIsSkeletonSeatedMode);
	if (FAILED(hr))
	{
		Simple_Message_Box(QString("Skeleton seated mode is not supported"));
	}
}

void DepthSensor::setFilpXYMode(bool mode)
{
	m_bFilpXY = mode;
}

void DepthSensor::filpSkeleton(NUI_SKELETON_FRAME &skeletonFrame)
{
	for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
	{
		// flip x to -x
		NUI_SKELETON_DATA *pSkel = &(skeletonFrame.SkeletonData[i]);

		for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
		{
			Vector4 pos = pSkel->SkeletonPositions[j];
			pos.x *= -1;
			pSkel->SkeletonPositions[j] = pos;
		}
	}
}

void DepthSensor::saveCurrSkeleton()
{
	bool bSkeletonTracked = false;

	for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
	{
		
		NUI_SKELETON_DATA pSkel = m_skeletonFrame.SkeletonData[i];

		if (pSkel.eTrackingState == NUI_SKELETON_TRACKED)
		{
			bSkeletonTracked = true;

			QVector<Vector4> currFrameSkel(NUI_SKELETON_POSITION_COUNT);

			for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
			{
				Vector4 pos = pSkel.SkeletonPositions[j];

				// save the tracking state in the last component
				pos.w = pSkel.eSkeletonPositionTrackingState[j];

				currFrameSkel[j] = pos;
			}

			m_skeletonStream.push_back(currFrameSkel);
		}
	}

	if (!bSkeletonTracked)
	{
		QVector<Vector4> currFrameSkel(NUI_SKELETON_POSITION_COUNT);

		for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
		{
			Vector4 pos;
			pos.x = 0; pos.y = 0; pos.z = 0; pos.w = 0;
			currFrameSkel[j] = pos;
		}

		m_skeletonStream.push_back(currFrameSkel);
	}
}

void DepthSensor::saveSkeletonFile()
{
	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".skel";
	QFile outFile(filename);
	QTextStream out(&outFile);

	if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	for (int i = 0; i < m_skeletonStream.size(); i++)
	{
		for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
		{
			out << m_skeletonStream[i][j].x << " " << m_skeletonStream[i][j].y << " " << m_skeletonStream[i][j].z << " " << m_skeletonStream[i][j].w << "\n";
		}
	}

	outFile.close();
}

void DepthSensor::loadSkeletonFile()
{
	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".skel";
	QFile inFile(filename);

	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return;

	m_skeletonStream.resize(m_loadFrameNum);

	for (int i = 0; i < m_loadFrameNum; i++)
	{
		QVector<Vector4> currFrameSkel(NUI_SKELETON_POSITION_COUNT);
		for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
		{
			ifs >> currFrameSkel[j].x >> currFrameSkel[j].y >> currFrameSkel[j].z >> currFrameSkel[j].w;
		}

		m_skeletonStream[i] = currFrameSkel;
	}

	inFile.close();
}

void DepthSensor::updateColorLabelFromFrame()
{
	QPixmap colorImg;

	// Draw skeleton onto color stream
	if (m_bIsTrackSkeleton)
	{
		m_openCVHelper->DrawSkeletonsInColorImage(&m_colorMat, &m_skeletonFrame, m_colorResolution, m_depthResolution);
	}

	colorImg = QPixmap::fromImage(cvMatToQImage(m_colorMat));

	DrawText_QPixmap(&colorImg, QString("%1 %2").arg("# ").arg(m_currentFrameID), QPoint(50, 50));

	if (!m_bIsPaused)
	{
		DrawText_QPixmap(&colorImg, QString("%1 %2").arg(m_colorFrameRateTracker.CurrentFPS()).arg("FPS"), QPoint(50, 90));
	}
	m_colorLabel->setPixmap(colorImg.scaled(m_colorLabel->width(), m_colorLabel->height(), Qt::KeepAspectRatio));
}

void DepthSensor::updateDepthLabelFromFrame()
{
	QPixmap depthImg;

	//convert to 3 channels so that color skeleton and fps text could be drawn on the depth image
	Mat m_depthMat8UC3 = Mat16UToMat8UC3(m_depthMat);

	//cv::imshow("depth16u", m_depthMat);
	//cv::imshow("depth8uc3", m_depthMat8UC3);

	// Draw skeleton onto depth stream
	if (m_bIsTrackSkeleton)
	{
		m_openCVHelper->DrawSkeletonsInDepthImage(&m_depthMat8UC3, &m_skeletonFrame, m_depthResolution);
	}

	depthImg = QPixmap::fromImage(cvMatToQImage(m_depthMat8UC3));
	
	DrawText_QPixmap(&depthImg, QString("%1 %2").arg("# ").arg(m_currentFrameID), QPoint(50, 50));

	if (!m_bIsPaused)
	{
		DrawText_QPixmap(&depthImg, QString("%1 %2").arg(m_depthFrameRateTracker.CurrentFPS()).arg("FPS"), QPoint(50, 90));
	}
	m_depthLabel->setPixmap(depthImg.scaled(m_depthLabel->width(), m_depthLabel->height(), Qt::KeepAspectRatio));
}

void DepthSensor::updateColoarLabelFromLoad()
{
	QPixmap colorImg;

	// Draw skeleton onto color stream
	if (m_bIsTrackSkeleton)
	{
		m_openCVHelper->DrawSkeletonsInColorImage(&m_colorMat, m_skeletonStream[m_currentFrameID], m_colorResolution, m_depthResolution);
	}

	colorImg = QPixmap::fromImage(cvMatToQImage(m_colorMat));

	DrawText_QPixmap(&colorImg, QString("%1 %2").arg("# ").arg(m_currentFrameID), QPoint(50, 50));

	if (!m_bIsPaused)
	{
		DrawText_QPixmap(&colorImg, QString("%1 %2").arg(m_colorFrameRateTracker.CurrentFPS()).arg("FPS"), QPoint(50, 90));
	}
	m_colorLabel->setPixmap(colorImg.scaled(m_colorLabel->width(), m_colorLabel->height(), Qt::KeepAspectRatio));
}

void DepthSensor::updateDepthLabelFromLoad()
{
	QPixmap depthImg;

	Mat m_depthMat8UC3 = Mat16UToMat8UC3(m_depthMat);

	// Draw skeleton onto depth stream
	if (m_bIsTrackSkeleton)
	{
		m_openCVHelper->DrawSkeletonsInDepthImage(&m_depthMat8UC3, m_skeletonStream[m_currentFrameID], m_depthResolution);
	}

	depthImg = QPixmap::fromImage(cvMatToQImage(m_depthMat8UC3));

	DrawText_QPixmap(&depthImg, QString("%1 %2").arg("# ").arg(m_currentFrameID), QPoint(50,50));

	if (!m_bIsPaused)
	{
		DrawText_QPixmap(&depthImg, QString("%1 %2").arg(m_depthFrameRateTracker.CurrentFPS()).arg("FPS"), QPoint(50,90));
	}
	m_depthLabel->setPixmap(depthImg.scaled(m_depthLabel->width(), m_depthLabel->height(), Qt::KeepAspectRatio));
}

void DepthSensor::saveTimeStampFile()
{
	// change the time stamp to intervals
	QVector<int> tempStamp(m_timeStamp);

	for (int i = 1; i < m_timeStamp.size(); i++)
	{
		m_timeStamp[i] = tempStamp[i] - tempStamp[i - 1];
	}

	m_timeStamp[0] = 0;

	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".timeStamp";
	QFile outFile(filename);
	QTextStream out(&outFile);

	if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	for (int i = 0; i < m_timeStamp.size(); i++)
	{
		out << m_timeStamp[i] << "\n";
	}

	outFile.close();
}

void DepthSensor::loadTimeStampFile()
{
	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".timeStamp";
	QFile inFile(filename);

	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return;

	m_timeStamp.resize(m_loadFrameNum);

	for (int i = 0; i < m_loadFrameNum; i++)
	{
		ifs >> m_timeStamp[i];
	}

	inFile.close();

	m_totalElapsedMs = 0;
}

void DepthSensor::saveStreamToImgFiles()
{
	// color stream from kinect is BGRA format
	Mat tempMat(m_colorMat.size(), CV_8UC3);

	for (int i = 0; i < m_colorMatStream.size();i++)
	{
		cv::cvtColor(m_colorMatStream[i], tempMat, COLOR_BGRA2BGR);
		cv::imwrite(QString(m_scanFilePath + m_scanFileName + "/" + "color_%1.png").arg(i, 4).toStdString(), tempMat);
	}

	for (int i = 0; i < m_depthMatStream.size();i++)
	{
		cv::imwrite(QString(m_scanFilePath + m_scanFileName + "/" + "depth_%1.png").arg(i, 4).toStdString(), m_depthMatStream[i]);
	}
}

void DepthSensor::showDepthCloud()
{
	m_depthCloud = new DepthCloud(m_width, m_height);

	if (m_colorMatStream.size() > 0)
	{
		m_colorMat = m_colorMatStream[m_currentFrameID];
		m_depthMat = m_depthMatStream[m_currentFrameID];
	}

	m_depthCloud->convertRGBDepthToPointXYZRGB(m_colorMat, m_depthMat, m_mapper, m_bFilpXY);

	m_depthCloud->draw();

	m_bIsShowPointCloud = true;

}

void DepthSensor::saveCoordinateMapper()
{
	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".mapper";
	QFile outFile(filename);
	QDataStream  out(&outFile);

	if (!outFile.open(QIODevice::ReadWrite)) return;

	ULONG DataByteCount;
	void *ppData;

	if (S_OK == m_mapper->GetColorToDepthRelationalParameters(&DataByteCount, &ppData))
	{
		BYTE* ppDataByte = reinterpret_cast<BYTE*>(ppData);

		out << (quint32)DataByteCount;

		for (int i = 0; i < DataByteCount;i++)
		{
			out << *ppDataByte;
			ppDataByte++;
		}
	}

	outFile.close();
}

void DepthSensor::loadCoordinateMapper()
{
	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".mapper";
	QFile inFile(filename);

	QDataStream  ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly)) return;

	quint32 DataByteCount;
	BYTE* ppDataByte = new BYTE[DataByteCount];
	BYTE* pCurrByte = ppDataByte;

	ifs >> DataByteCount;

	for (int i = 0; i < DataByteCount; i++)
	{
		ifs >> *pCurrByte;
		pCurrByte++;
	}

	inFile.close();

	HRESULT hr = NuiCreateCoordinateMapperFromParameters((ULONG)DataByteCount, (void*)ppDataByte, &m_mapper);

	delete[] ppDataByte;
}

void DepthSensor::deleteDepthCloud()
{
	delete m_depthCloud;
	m_depthCloud = NULL;

	m_bIsShowPointCloud = false;
}

void DepthSensor::loadActionLabelFile()
{
	m_frameLabels.resize(m_depthMatStream.size());

	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".labels";
	QFile inFile(filename);

	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		return;
	}

	else
	{
		for (int i = 0; i < m_frameLabels.size(); i++)
		{
			int pairNum;
			ifs >> pairNum;

			for (int j = 0; j < pairNum; j++)
			{
				int actionID, modelID;
				ifs >> actionID >> modelID;

				FrameLabel newLabel(actionID, modelID);
				m_frameLabels[i].insert(newLabel);
			}
		}
	}

	inFile.close();
}

void DepthSensor::saveActionLabelFile()
{
	QString filename = m_scanFilePath + m_scanFileName + "/" + m_scanFileName + ".labels";
	QFile outFile(filename);
	QTextStream out(&outFile);

	if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	for (int i = 0; i < m_frameLabels.size(); i++)
	{
		out << m_frameLabels[i].size() << " ";

		for (QSet<FrameLabel>::iterator it = m_frameLabels[i].begin(); it != m_frameLabels[i].end(); it++)
		{
			out << it->first << " " << it->second << " ";
		}

		out << "\n";
	}

	outFile.close();	
}

void DepthSensor::addLabelToCurrFrame(int actionID, int modelID)
{

	FrameLabel newLabel;
	newLabel.first = actionID;
	newLabel.second = modelID;

	m_frameLabels[m_currentFrameID].insert(newLabel);
}

void DepthSensor::clearLabelOfCurrFrame()
{
	m_frameLabels[m_currentFrameID].clear();
}

void DepthSensor::addLabelToFrames(int fromId, int toId, int actionID, int modelID)
{
	for (int id = fromId; id <= toId; id++)
	{
		FrameLabel newLabel;
		newLabel.first = actionID;
		newLabel.second = modelID;

		m_frameLabels[id].insert(newLabel);
	}
}

void DepthSensor::clearLabelOfFrames(int fromId, int toId)
{
	for (int id = fromId; id <= toId; id++)
	{
		m_frameLabels[id].clear();
	}
}

void DepthSensor::clearAllLabels()
{
	for (int i = 0; i < m_frameLabels.size(); i++)
	{
		m_frameLabels[i].clear();
	}
}

void DepthSensor::startTracking(bool trackingState)
{
	m_depthCloud->setTrackingState(trackingState);
	m_depthCloud->setTrackRefCloud(m_trackRefCloud);
}

void DepthSensor::resetDepthCloudView()
{
	m_depthCloud->resetCamView();
}

void DepthSensor::setTrackRefCloud(QVector<SimplePointCloud> &clouds, Eigen::Matrix4d &transMat)
{
	m_trackRefCloud = clouds;

	for (int i = 0; i < m_trackRefCloud.size();i++)
	{
		for (int j = 0; j < m_trackRefCloud[i].size(); j++)
		{
			Eigen::Vector4d newPt = transMat*Eigen::Vector4d(m_trackRefCloud[i][j][0], m_trackRefCloud[i][j][1], m_trackRefCloud[i][j][2], 1.0);
			m_trackRefCloud[i][j] = Eigen::Vector3d(newPt[0] / newPt[3], newPt[1] / newPt[3], newPt[2] / newPt[3]);
		}
	}

	// init tracking result transformation matrix
	m_trackTransMat.resize(m_trackRefCloud.size());

	for (int i = 0; i < m_trackTransMat.size(); i++)
	{
		m_trackTransMat[i].fill(Eigen::Matrix4d::Identity(), m_depthMatStream.size());
	}
}

void DepthSensor::computeTrackResultCloud()
{
	QVector<SimplePointCloud> resultCloud = m_trackRefCloud;

	for (int i = 0; i < m_trackRefCloud.size(); i++)
	{
		Eigen::Matrix4d transMat = m_trackTransMat[i][m_currentFrameID];

		for (int j = 0; j < m_trackRefCloud[i].size(); j++)
		{
			Eigen::Vector4d newPt = transMat*Eigen::Vector4d(m_trackRefCloud[i][j][0], m_trackRefCloud[i][j][1], m_trackRefCloud[i][j][2], 1.0);
			resultCloud[i][j] = Eigen::Vector3d(newPt[0] / newPt[3], newPt[1] / newPt[3], newPt[2] / newPt[3]);
		}
	}

	m_depthCloud->setTrackResultCloud(resultCloud);
}

void DepthSensor::doOfflineTracking()
{
	for (int r = 0; r < m_trackRefCloud.size(); r++)
	{
		m_depthCloud->setTrackRefCloud(m_trackRefCloud[r]);

		for (int i = 0; i < m_depthMatStream.size(); i++)
		{
			m_depthCloud->convertRGBDepthToPointXYZRGB(m_colorMatStream[i], m_depthMatStream[i], m_mapper, m_bFilpXY);

			m_depthCloud->trackOnce();
			m_trackTransMat[r][i] = m_depthCloud->getTrackingResultMat();
		}
	}

	saveTrackedTransMat();

	m_hasTrackedTransMat = true;

	Simple_Message_Box("Offline tracking finished");
}

void DepthSensor::saveTrackedTransMat()
{
	for (int trackID = 0; trackID < m_trackRefCloud.size(); trackID++)
	{
		QString filename = m_scanFilePath + m_scanFileName + "/" + QString(m_trackRefCloud[trackID].getLabel().c_str()) + ".track";
		QFile outFile(filename);
		QTextStream  out(&outFile);

		if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

		out << m_trackRefCloud[trackID].getID() << "\n";

		for (int i = 0; i < m_trackTransMat[trackID].size(); i++)
		{
			for (int x = 0; x < 4; x++)
			{
				for (int y = 0; y < 4; y++)
				{
					out << m_trackTransMat[trackID][i](x, y) << " ";
				}
			}
			out << "\n";
		}

		outFile.close();
	}
}

bool DepthSensor::loadTrackedTransMat()
{
	for (int trackID = 0; trackID < m_trackRefCloud.size(); trackID++)
	{
		QString filename = m_scanFilePath + m_scanFileName + "/" + QString(m_trackRefCloud[trackID].getLabel().c_str()) + ".track";
		QFile inFile(filename);

		QTextStream ifs(&inFile);

		if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

		int trackModelID;

			ifs >> trackModelID;

			if (trackModelID != m_trackRefCloud[trackID].getID())
			{
				return false;
			}

			for (int i = 0; i < m_trackTransMat[trackID].size(); i++)
			{
				for (int x = 0; x < 4; x++)
				{
					for (int y = 0; y < 4; y++)
					{
						ifs >> m_trackTransMat[trackID][i](x, y);
					}
				}
			}

		inFile.close();
	}

	m_hasTrackedTransMat = true;

	return true;
}
