#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/ml/ml.hpp"
#include "../Utilities/utility.h"

#include <type_traits>

template<typename T>
class OpenCVClassifier
{
public:
	enum ClassifierType
	{
		RandomTress
	};

	OpenCVClassifier();
	~OpenCVClassifier();

	bool read_train_data(const std::string& filename, int var_count);
	void prepare_train_data(const cv::Mat& data, const cv::Mat& responses, int ntrain_samples);

	void load_classifier(const std::string& filename);
	void save_classifier(const std::string& filename);

	void buildClassifier(std::string filename);
	void build_rtrees_classifier();

	float predict(const cv::Mat& testSample);

	cv::TermCriteria TC(int iters, double eps)
	{
		return cv::TermCriteria(cv::TermCriteria::MAX_ITER + (eps > 0 ? cv::TermCriteria::EPS : 0), iters, eps);
	}

private:
	std::string m_classifierName;
	std::string m_dataFileName;

	cv::Ptr<T> m_model;
	cv::Ptr<cv::ml::TrainData> m_trainData;

	cv::Mat m_data;
	cv::Mat m_response;
};

template<typename T>
OpenCVClassifier<T>::~OpenCVClassifier()
{

}

template<typename T>
OpenCVClassifier<T>::OpenCVClassifier()
{

}

template<typename T>
bool OpenCVClassifier<T>::read_train_data(const std::string& filename, int var_count)
{
	const int M = 1024;
	char buf[M + 2];

	Mat el_ptr(1, var_count, CV_32F);
	int i;
	vector<int> responses;

	m_data.release();
	m_response.release();

	FILE* f = fopen(filename.c_str(), "rt");
	if (!f)
	{
		cout << "Could not read the database " << filename << endl;
		return false;
	}

	for (;;)
	{
		char* ptr;
		if (!fgets(buf, M, f) || !strchr(buf, ','))
			break;
		responses.push_back((int)buf[0]);
		ptr = buf + 2;
		for (i = 0; i < var_count; i++)
		{
			int n = 0;
			sscanf(ptr, "%f%n", &el_ptr.at<float>(i), &n);
			ptr += n + 1;
		}
		if (i < var_count)
			break;
		m_data.push_back(el_ptr);
	}
	fclose(f);
	Mat(responses).copyTo(m_response);

	return true;
}


template<typename T>
void OpenCVClassifier<T>::prepare_train_data(const cv::Mat& data, const cv::Mat& responses, int ntrain_samples)
{
	cv::Mat sample_idx = cv::Mat::zeros(1, data.rows, CV_8U);
	cv::Mat train_samples = sample_idx.colRange(0, ntrain_samples);
	train_samples.setTo(cv::Scalar::all(1));

	int nvars = data.cols;
	cv::Mat var_type(nvars + 1, 1, CV_8U);
	var_type.setTo(cv::Scalar::all(cv::ml::VAR_ORDERED));
	var_type.at<uchar>(nvars) = cv::ml::VAR_CATEGORICAL;

	m_trainData = cv::ml::TrainData::create(data, cv::ml::ROW_SAMPLE, responses,
		noArray(), sample_idx, noArray(), var_type);
}



template<typename T>
void OpenCVClassifier<T>::load_classifier(const std::string& filename)
{
	// load classifier from the specified file
	m_model = cv::ml::StatModel::load<T>(filename);
}


template<typename T>
void OpenCVClassifier<T>::save_classifier(const std::string& filename)
{
	if (!filename.empty())
	{
		m_model->save(filename);
	}
}

template<typename T>
void OpenCVClassifier<T>::buildClassifier(std::string filename)
{
	if (std::is_same<T, cv::ml::RTrees>::value)
	{
		m_model = cv::ml::StatModel::load<T>(filename);
		
		if (m_model.empty())
		{
			build_rtrees_classifier();
		}	
	}
}


template<typename T>
void OpenCVClassifier<T>::build_rtrees_classifier()
{
	int nsamples_all = m_data.rows;
	int ntrain_samples = (int)(nsamples_all*0.8);

	// create classifier by using <data> and <responses>
	//cout << "Training the classifier ...\n";
	prepare_train_data(m_data, m_response, ntrain_samples);
	m_model = cv::ml::StatModel::train<cv::ml::RTrees>(m_trainData, cv::ml::RTrees::Params(10, 10, 0, false, 15, Mat(), true, 4, TC(100, 0.01f)));
}


template<typename T>
float OpenCVClassifier<T>::predict(const cv::Mat& testSample)
{
	return m_model->predict(testSample);
}

static void convertToMat(std::vector<double> &featureVec, cv::Mat &featureMat)
{
	featureMat = cv::Mat(1, featureVec.size(), CV_32F);

	for (int i = 0; i < featureVec.size(); i++)
	{
		featureMat.at<float>(i) = featureVec[i];
	}
}