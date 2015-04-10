#pragma once
#include "opencv2/core/core_c.h"
#include "opencv2/ml/ml.hpp"
#include "../Utilities/utility.h"

class OpenCVClassifier
{
public:
	enum ClassifierType
	{
		RandomTrees
	};

	OpenCVClassifier();
	~OpenCVClassifier();

	void buildClassifier(ClassifierType classifierType, int featureDim, QString dataFileName, QString classifierFileName);
	int read_num_class_data(const char* filename, int var_count,
		CvMat** data, CvMat** responses);
	void load_classifier(const char* filename);

	int build_rtrees_classifier(const char* data_filenameconst, const char* filename_to_save);
	float predict(CvMat* testSample);

private:
	CvRTrees m_forest;
	int m_featureDim;
};

void convertStdVecToCvMat(const std::vector<double> &featureVec, CvMat* featureMat);
void convertCvMatToStdVec(CvMat *featureMat, int rowID, std::vector<double> &featureVec);