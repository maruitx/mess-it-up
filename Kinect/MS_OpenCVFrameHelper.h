//-----------------------------------------------------------------------------
// <copyright file="OpenCVFrameHelper.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation. All rights reserved.
// </copyright>
//-----------------------------------------------------------------------------

#pragma once
#include "MS_KinectHelper.h"

#include <QImage>
#include <QDebug>

// Suppress warnings that come from compiling OpenCV code since we have no control over it
#pragma warning(push)
#pragma warning(disable : 6294 6031)
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#pragma warning(pop)

using namespace cv;
namespace Microsoft {
    namespace KinectBridge {
        class OpenCVFrameHelper : public KinectHelper<Mat> {
        public:
            // Functions:
            /// <summary>
            /// Constructor
            /// </summary>
            OpenCVFrameHelper() : KinectHelper<Mat>() {}

            /// <summary>
            /// Destructor
            /// </summary>
            ~OpenCVFrameHelper() {}

            // Constants
            // Mat type for each usage
            static const int COLOR_TYPE = CV_8UC4;
            static const int DEPTH_TYPE = CV_16U;
            static const int DEPTH_RGB_TYPE = CV_8UC4;

        protected:
            // Functions:
            /// <summary>
            /// Converts from Kinect color frame data into a RGB OpenCV image matrix
            /// </summary>
            /// <param name="pImage">pointer in which to return the OpenCV image matrix</param>
            /// <returns>S_OK if successful, an error code otherwise</returns>
            HRESULT GetColorData(Mat* pImage) const override;

            /// <summary>
            /// Converts from Kinect depth frame data into a OpenCV matrix
            /// </summary>
            /// <param name="pImage">pointer in which to return the OpenCV matrix</param>
            /// <returns>S_OK if successful, an error code otherwise</returns>
            HRESULT GetDepthData(Mat* pImage) const override;

            /// <summary>
            /// Converts from Kinect depth frame data into a ARGB OpenCV image matrix
            /// </summary>
            /// <param name="pImage">pointer in which to return the OpenCV image matrix</param>
            /// <returns>S_OK if successful, an error code otherwise</returns>
            HRESULT GetDepthDataAsArgb(Mat* pImage) const override;

            /// <summary>
            /// Verify image is of the given resolution
            /// </summary>
            /// <param name="pImage">pointer to image to verify</param>
            /// <param name="resolution">resolution of image</param>
            /// <returns>S_OK if image matches given width and height, an error code otherwise</returns>
            HRESULT VerifySize(const Mat* pImage, NUI_IMAGE_RESOLUTION resolution) const override;
        };
    }
}

/**
* Return true if the number is NAN.
*/
template<class T>
inline bool uIsNan(const T & value)
{
#ifdef __APPLE__
	return std::isnan(value);
#elif _MSC_VER
	return _isnan(value) != 0;
#else
	return isnan(value);
#endif
}


// Conversion from/to cv::Mat to QImage
inline QImage cvMatToQImage(const cv::Mat &inMat){
	switch (inMat.type())
	{
		// 8-bit, 4 channel
	case CV_8UC4:
	{
		QImage image(inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32);
		return image;
	}
		// 8-bit, 3 channel
	case CV_8UC3:
	{
		QImage image(inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888);
		return image.rgbSwapped();
	}
		// 8-bit, 1 channel
	case CV_8UC1:
	{
		static QVector<QRgb>  sColorTable;
		// only create our color table once
		if (sColorTable.isEmpty())
		{
			for (int i = 0; i < 256; ++i)
				sColorTable.push_back(qRgb(i, i, i));
		}
		QImage image(inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8);
		image.setColorTable(sColorTable);
		return image;
	}

	case  CV_16U:
	{
		// Assume depth image (unsigned short in mm)
		const unsigned short * data = (const unsigned short *)inMat.data;
		double min, max;
		cv::minMaxIdx(inMat, &min, &max);

		QImage image = QImage(inMat.cols, inMat.rows, QImage::Format_Indexed8);
		for (int y = 0; y < inMat.rows; ++y, data += inMat.cols)
		{
			for (int x = 0; x < inMat.cols; ++x)
			{
				uchar * p = image.scanLine(y) + x;
				if (data[x] < min || data[x] > max || uIsNan(data[x]) || max == min)
				{
					*p = 0;
				}
				else
				{
					// reverse the intensity for visualization
					// p = 0, black
					// p = 255, white
					*p = uchar(255.0f - (float(data[x] - min) / float(max - min))*255.0f);
					// if depth == min
					if (*p == 255)  
					{
						*p = 0;
					}
				}
			}
		}



		QVector<QRgb> my_table;
		for (int i = 0; i < 256; i++) my_table.push_back(qRgb(i, i, i));
		image.setColorTable(my_table);
		return image;
	}

	default:
		qDebug() << "cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
		break;
	}
	return QImage();
}

// Will force a conversion to grayscale
inline cv::Mat QImageToCvMat(const QImage &inputImage, bool inCloneImageData = true){
	QImage swapped = inputImage.convertToFormat(QImage::Format_RGB888).rgbSwapped();
	cv::Mat colored = cv::Mat(swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine()).clone();
	cv::Mat gray;
	cvtColor(colored, gray, cv::COLOR_BGR2GRAY);
	return gray;
}

inline cv::Mat Mat16UToMat8UC3(const cv::Mat &inMat)
{
	cv::Mat tempMat = inMat.clone();
	double min, max;
	cv::minMaxIdx(tempMat, &min, &max);

	// scale the data and reverse the color for visualization
	// grey = 255 - alpha*(x-min)
	double alpha = 255.0 / (max - min);
	tempMat.convertTo(tempMat, CV_8UC1, -alpha, 255 + alpha*min);

	// if original value == 0, set to black
	cv::Mat mask = tempMat == 255;
	tempMat.setTo(0, mask);

	cv::Mat outMat(tempMat.size(), CV_8UC3);
	
	// convert to 3 channels
	cv::cvtColor(tempMat, outMat, COLOR_GRAY2BGR);

	return outMat;
}
