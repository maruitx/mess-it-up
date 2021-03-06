#include "ActionPredictor.h"
#include "../Geometry/Scene.h"
#include "../Geometry/Skeleton.h"
#include "../Geometry/SkeletonSampler.h"
#include "ActionFeature.h"


ActionPredictor::ActionPredictor(QObject *parent)
	: QObject(parent)
{
	m_classProbThreshold = 0.8;

	m_showSampledSkeleton = false;
	m_showStartPose = false;
	m_showEndPose = false;

	m_showSampeRegion = false;
	m_finishPredict = false;
}

ActionPredictor::~ActionPredictor()
{

}

void ActionPredictor::init(mess_mode *m)
{
	m_scene = m->getScene();
	m_drawArea = m->drawArea();
}

bool ActionPredictor::loadTestJob(const QString &filename)
{
	//load job file
	QFile inFile(filename);
	QFileInfo inFileInfo(inFile.fileName());

	if (!inFile.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		Simple_Message_Box("Cannot open Test file");
		return false;
	}
	
	QTextStream ifs(&inFile);

	ifs >> m_sceneFileName;
	ifs >> m_useFeatureType;

	inFile.close();

	m_jobFilePath = inFileInfo.absolutePath() + "/";
	m_scene->loadScene(m_jobFilePath + "Scene/" + m_sceneFileName + "/" + m_sceneFileName + "/" + m_sceneFileName + ".txt");
	m_scene->setSceneDrawArea(m_drawArea);

	return true;
}

void ActionPredictor::loadTrainingResult()
{
	loadActionRepSkels();
	loadClassifiers();
}

	// load learned classifiers
void ActionPredictor::loadClassifiers()
{
	if (!m_classifiers.empty())
	{
		m_classifiers.clear();
	}

	m_classifiers.resize(ACTION_PHASE_NUM);
	QString featureFilePath = m_jobFilePath + "Feature/Train/";

	for (int phase_id = 0; phase_id < ACTION_PHASE_NUM; phase_id++)
	{
		if (isPhaseConsidered(phase_id))
		{
			m_classifiers[phase_id].resize(ACTION_NUM);

			for (int action_id = 0; action_id < ACTION_NUM; action_id++)
			{
				QString filename = featureFilePath + QString(Action_Phase_Names[phase_id]) + "_" + QString(Action_Labels[action_id]) +"_classifier.xml";
				QFile classifierFile(filename);

				if (classifierFile.exists())
				{
					m_classifiers[phase_id][action_id] = new OpenCVClassifier();
					m_classifiers[phase_id][action_id]->load_classifier(filename.toStdString().c_str());
				}
			}
		}
	}

	Simple_Message_Box("Training result loaded");
}

void ActionPredictor::startPredicting()
{
	if (m_loadedSkeletonsForTest.size() == 0)
	{
		Simple_Message_Box("Please load training data first");
		return;
	}

	m_skeletonSampler = new SkeletonSampler(m_scene);
	
	sampleSkeletons();
	genRandomSkeletonListForDisplay(5);

	m_showSampledSkeleton = true;
	m_showStartPose = true;
	m_showEndPose = false;

	//m_showSampeRegion = true;
	m_finishPredict = true;

	Simple_Message_Box("Action prediction done");
}

void ActionPredictor::loadActionRepSkels()
{
	m_loadedSkeletonsForTest.resize(ACTION_PHASE_NUM);

	loadActionRepSkels(ActionFeature::ActionPhase::StartAction);
	loadActionRepSkels(ActionFeature::ActionPhase::EndAction);
}

// for action predictor, skeleton is not constraint to scenes now
void ActionPredictor::loadActionRepSkels(int phaseID)
{
	m_loadedSkeletonsForTest[phaseID].clear();
	m_loadedSkeletonsForTest[phaseID].resize(ACTION_NUM);

	QString featureFilePath = m_jobFilePath + "Feature/Train/";

	if (m_sceneFileName.isEmpty())
	{
		m_sceneFileName = m_scene->getSceneName();
	}	

	for (int actionID = 0; actionID < ACTION_NUM; actionID++)
	{
		QString filename = featureFilePath + m_useFeatureType + "_" + QString(Action_Phase_Names[phaseID]) + "_" + QString(Action_Labels[actionID]) + ".skel";
		QFile inFile(filename);
		QTextStream ifs(&inFile);

		if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text))
			continue;

		while (!ifs.atEnd())
		{
			QString skeletonString = ifs.readLine();

			QStringList skeletonJointsList = skeletonString.split(" ");

			QVector<Eigen::Vector4d> joints(skeletonJointsList.size() / 3, Eigen::Vector4d(0, 0, 0, 1));

			for (int id = 0; id < skeletonJointsList.size() / 3; id++)
			{
				joints[id][0] = skeletonJointsList[3 * id].toDouble();
				joints[id][1] = skeletonJointsList[3 * id + 1].toDouble();
				joints[id][2] = skeletonJointsList[3 * id + 2].toDouble();
			}

			Skeleton *newSkel = new Skeleton(joints);
			newSkel->setScene(m_scene);

			//if (m_useFeatureType == "synth")
			{
				newSkel->AlignToOrigin();
				newSkel->computeOrientation();
				newSkel->computeOBB();
			}
			
			m_loadedSkeletonsForTest[phaseID][actionID].push_back(newSkel);
		}

		inFile.close();
	}
}

void ActionPredictor::genRandomSkeletonListForDisplay(int num)
{
	ModelRelatedSkeletonList::iterator it;

	// sample ids for sampled skeletons
	for (it = m_sampledSkeletonsForActions.begin(); it != m_sampledSkeletonsForActions.end(); it++)
	{
		int modelID = it->first;

		m_randomSampledSkeletonIdList[modelID].resize(ACTION_PHASE_NUM);

		for (int phaseID = 0; phaseID < ACTION_PHASE_NUM; phaseID++)
		{
			m_randomSampledSkeletonIdList[modelID][phaseID].resize(ACTION_NUM);

			for (int actionID = 0; actionID < ACTION_NUM; actionID++)
			{
				int actionNum = it->second[phaseID][actionID].size();
				if (actionNum > 0)
				{
					if (actionNum < num)
					{
						num = actionNum;
					}

					std::vector<int> randIdList(num);

					for (int i = 0; i < num; i++)
					{
						int skelID = std::rand() % m_sampledSkeletonsForActions[modelID][phaseID][actionID].size();
						randIdList[i] = skelID;
					}

					m_randomSampledSkeletonIdList[modelID][phaseID][actionID] = randIdList;
				}
			}
		}
	}

	// sample ids for predicted skeletons
	for (it = m_predictedSkeletonsForActions.begin(); it != m_predictedSkeletonsForActions.end(); it++)
	{
		int modelID = it->first;

		m_randomPredictedSkeletonIdList[modelID].resize(ACTION_PHASE_NUM);

		for (int phaseID = 0; phaseID < ACTION_PHASE_NUM; phaseID++)
		{
			m_randomPredictedSkeletonIdList[modelID][phaseID].resize(ACTION_NUM);

			for (int actionID = 0; actionID < ACTION_NUM; actionID++)
			{
				int actionNum = it->second[phaseID][actionID].size();
				if (actionNum > 0)
				{
					if (actionNum < num)
					{
						num = actionNum;
					}

					std::vector<int> randIdList(num);

					for (int i = 0; i < num; i++)
					{
						int skelID = std::rand() % m_predictedSkeletonsForActions[modelID][phaseID][actionID].size();
						randIdList[i] = skelID;
					}

					m_randomPredictedSkeletonIdList[modelID][phaseID][actionID] = randIdList;
				}
			}
		}
	}

	updateDrawArea();
}

void ActionPredictor::drawSampledSkeletons(int modelID, int phaseID, int actionID)
{
	if (m_sampledSkeletonsForActions[modelID][phaseID][actionID].size() > 0)
	{
		for (int i = 0; i < m_randomSampledSkeletonIdList[modelID][phaseID][actionID].size(); i++)
		{
			int skeID = m_randomSampledSkeletonIdList[modelID][phaseID][actionID][i];
			Skeleton *currSkel = m_sampledSkeletonsForActions[modelID][phaseID][actionID][skeID];
			currSkel->draw(phaseID);
		}

		// show sample region
		if (m_showSampeRegion)
		{
			drawSampleRange(modelID);
		}
	}
}

void ActionPredictor::drawPredictedSkeletons(int modelID, int phaseID, int actionID)
{
	if (m_predictedSkeletonsForActions[modelID][phaseID][actionID].size() > 0)
	{
		for (int i = 0; i < m_randomPredictedSkeletonIdList[modelID][phaseID][actionID].size(); i++)
		{
			int skeID = m_randomPredictedSkeletonIdList[modelID][phaseID][actionID][i];
			Skeleton *currSkel = m_predictedSkeletonsForActions[modelID][phaseID][actionID][skeID];
			currSkel->draw(phaseID);
			currSkel->drawPotentialPlacement();
		}

		// show sample region
		if (m_showSampeRegion)
		{
			drawSampleRange(modelID);
		}
	}
}

void ActionPredictor::updateDrawArea()
{
	m_drawArea->updateGL();
}

void ActionPredictor::drawSampleRange(int modelID)
{
	std::vector<double> sampleRange = m_skeletonSampler->getSampleRange(modelID);

	GLfloat red[] = { 1.0, 0.3, 0.3, 1.0 };
	GLfloat green[] = { 0.3, 1.0, 0.3 };
	GLfloat blue[] = { 0.3, 0.3, 1.0, 1.0};


	glPushAttrib(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glDepthMask(GL_FALSE);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glColor4f(green[0], green[1], green[2], 0.8f);

	MathLib::Vector3 upright = m_scene->getUprightVec();

	/*
	xmin, xmax, ymin, ymax
	  0,	1,	  2,	3
	
	0,3 -------------- 1,3
	 |                  |
	 |					|
	 |					|	
	0,2 -------------- 1,2	
	*/

	glBegin(GL_TRIANGLES);

	glNormal3f(upright.x, upright.y, upright.z);
	glVertex3d(sampleRange[0], sampleRange[2], 0.01);
	glVertex3d(sampleRange[1], sampleRange[2], 0.01);
	glVertex3d(sampleRange[0], sampleRange[3], 0.01);

	glNormal3f(upright.x, upright.y, upright.z);
	glVertex3d(sampleRange[0], sampleRange[3], 0.01);
	glVertex3d(sampleRange[1], sampleRange[2], 0.01);
	glVertex3d(sampleRange[1], sampleRange[3], 0.01);

	glEnd();
	glPopAttrib();

	glColor4f(blue[0], blue[1], blue[2], 0.9f);

	std::vector<MathLib::Vector3>& samplePostions = m_skeletonSampler->getSamplePositions(modelID);
	for (int i = 0; i < samplePostions.size(); i++)
	{
		float r = PointRadius3D;
		renderSphere(samplePostions[i][0], samplePostions[i][1], samplePostions[i][2], r);
	}
}

int ActionPredictor::getSampledSkelNum(int modelID, int actionID)
{
	// need to fix: num of end phase skeleton is different from start phase
	// each pair is a possible action
	return m_sampledSkeletonsForActions[modelID][0][actionID].size();
}

int ActionPredictor::getPredictedSkelNum(int modelID, int actionID)
{
	return m_predictedSkeletonsForActions[modelID][0][actionID].size();
}

void ActionPredictor::resampleSkeletonForDisplay(int num)
{
	m_randomSampledSkeletonIdList.clear();
	genRandomSkeletonListForDisplay(num);

	emit finishPrediction();
}

void ActionPredictor::sampleSkeletons()
{
	m_sampledSkeletonsForActions.clear();
	m_predictedSkeletonsForActions.clear();

	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		m_sampledSkeletonsForActions[modelID].resize(ACTION_PHASE_NUM);
		m_predictedSkeletonsForActions[modelID].resize(ACTION_PHASE_NUM);
	}

	// sample within the floor range
	m_skeletonSampler->setFloorRange(m_scene->getFloorXYRange());

	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		for (int phaseID = 0; phaseID < ACTION_PHASE_NUM; phaseID++)
		{
			m_sampledSkeletonsForActions[modelID][phaseID].resize(ACTION_NUM);
			m_predictedSkeletonsForActions[modelID][phaseID].resize(ACTION_NUM);
		}
	}

	for (int phase_id = 0; phase_id < ACTION_PHASE_NUM; phase_id++)
	{
		sampleSkeletonsForActionPhrase(phase_id);
	}

	emit finishPrediction();
}

bool ActionPredictor::isPhaseConsidered(int phaseID)
{
	for (int action_id = 0; action_id < ACTION_NUM; action_id++)
	{
		if (m_loadedSkeletonsForTest[phaseID].size() > 0 && m_loadedSkeletonsForTest[phaseID][action_id].size() > 0)
		{
			return true;
		}
	}

	return false;
}

void ActionPredictor::sampleSkeletonsForActionPhrase(int phaseID)
{
	// predict possible action for each model
	for (int model_id = 0; model_id < m_scene->getModelNum(); model_id++)
	{
		// test for each action
		// random sample an/several actions from the action library
		for (int action_id = 0; action_id < m_loadedSkeletonsForTest[phaseID].size(); action_id++)
		{
			if (!m_scene->isModelFixed(model_id))
			{
				if (m_loadedSkeletonsForTest[phaseID][action_id].size() > 0)
				{
					SkeletonPtrList sampledSkeletons;
					m_skeletonSampler->setSkeletonList(m_loadedSkeletonsForTest[phaseID][action_id]);
					
					if (phaseID == ActionFeature::ActionPhase::StartAction)
					{
						m_skeletonSampler->sampleSkeletonAroundModelFromSkelList(model_id, "random");
						sampledSkeletons = m_skeletonSampler->getSampledStartSkeletons();
					}
					 
					if (phaseID == ActionFeature::ActionPhase::EndAction)
					{
						m_skeletonSampler->sampleArrangeSkeletonsInScene("random");
						sampledSkeletons = m_skeletonSampler->getSampledEndSkeletons();
					}

					m_sampledSkeletonsForActions[model_id][phaseID][action_id] = sampledSkeletons;

					SkeletonPtrList classifiedSkeletons;

					// test with classifier
					for (int skel_id = 0; skel_id < sampledSkeletons.size(); skel_id++)
					{
						Skeleton *currSkel = sampledSkeletons[skel_id];
						currSkel->sampleObjPlacementPosNearby();

						//if (m_classifiers[phaseID][action_id] != nullptr && classifyTestForSkeletons(model_id, phaseID, action_id, currSkel)) // hard classification
						if (m_classifiers[phaseID][action_id] != nullptr && classifyTestForSkeletonsFuzzy(model_id, phaseID, action_id, currSkel)) // fuzzy classificaiton
						{
							classifiedSkeletons.push_back(currSkel);
						}
					}

					m_predictedSkeletonsForActions[model_id][phaseID][action_id] = classifiedSkeletons;
				}
			}
		}
	}
}

void ActionPredictor::setShowStartPose(int state)
{
	m_showStartPose = state;
	updateDrawArea();
}

void ActionPredictor::setShowEndPose(int state)
{
	m_showEndPose = state;
	updateDrawArea();
}

bool ActionPredictor::classifyTestForSkeletons(int modelID, int phaseID, int actionID, Skeleton *skel)
{
	ActionFeature newFeature(m_scene);

	std::vector<double> actionFeature;

	newFeature.computeActionFeatureForSkel(skel, modelID, actionFeature);

	//// test feature with classifier
	//cv::Mat testFeatureMat;
	//convertToMat(actionFeature, testFeatureMat);

	CvMat* testFeatureMat = 0;
	testFeatureMat = cvCreateMat(1, actionFeature.size(), CV_32F);
	convertStdVecToCvMat(actionFeature, testFeatureMat);

	float predicted_id = m_classifiers[phaseID][actionID]->predict(testFeatureMat);

	if (std::abs(predicted_id - actionID) < LABEL_DIFF_TH)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void ActionPredictor::setDrawSampleRegionStatus(int s)
{
	m_showSampeRegion = s;
	updateDrawArea();
}

// works for binary classification problems only
// return probability or confidence of the sample belonging to the second class
// It is calculated as the proportion of decision trees that classified the sample to the second class
bool ActionPredictor::classifyTestForSkeletonsFuzzy(int modelID, int phaseID, int actionID, Skeleton *skel)
{
	bool state;
	if (phaseID == ActionFeature::ActionPhase::StartAction)
	{
		state = classifyStartSkeletonFuzzy(modelID, actionID, skel);
	}

	if (phaseID == ActionFeature::ActionPhase::EndAction)
	{
		state = classifyEndSkeletonFuzzy(modelID, actionID, skel);
	}

	return state;
}

void ActionPredictor::repredicting(double prob, int showSkelNum)
{
	m_classProbThreshold = prob;

	sampleSkeletons();
	genRandomSkeletonListForDisplay(showSkelNum);

	emit finishPrediction();
}

bool ActionPredictor::classifyStartSkeletonFuzzy(int modelID, int actionID, Skeleton *skel)
{
	ActionFeature newFeature(m_scene);

	std::vector<double> actionFeature;

	newFeature.computeActionFeatureForSkel(skel, modelID, actionFeature);

	CvMat* testFeatureMat = 0;
	testFeatureMat = cvCreateMat(1, actionFeature.size(), CV_32F);
	convertStdVecToCvMat(actionFeature, testFeatureMat);

	// important need to fix: feature dim doesn't match
	//float probVal = m_classifiers[ActionFeature::ActionPhase::StartAction][actionID]->predict_prob(testFeatureMat);

	//if (probVal > m_classProbThreshold)
	{
		return true;
	}

		return false;
}

bool ActionPredictor::classifyEndSkeletonFuzzy(int modelID, int actionID, Skeleton *skel)
{
	ActionFeature newFeature(m_scene);

	std::vector<std::vector<double>> actionFeatureList;
	
	newFeature.computeActionFeatureForSkelAndModel(skel, modelID, actionFeatureList);
	
	CvMat* testFeatureMat = 0;
	testFeatureMat = cvCreateMat(1, actionFeatureList[0].size(), CV_32F);

	std::vector<ObjLocation> predictPos;

	for (int i = 0; i < actionFeatureList.size(); i++)
	{
		// some action feature is empty, since some potential location is not valid, causing collision when move model there
		if (actionFeatureList[i].size() > 0)
		{
			convertStdVecToCvMat(actionFeatureList[i], testFeatureMat);

			//float probVal = m_classifiers[ActionFeature::ActionPhase::StartAction][actionID]->predict_prob(testFeatureMat);

			//if (probVal > m_classProbThreshold)
			{
				predictPos.push_back(skel->getSampledLocation(i));
			}
		}
	}

	if (!predictPos.empty())
	{
		skel->setObjPredictedLocation(predictPos);
		return true;
	}

	return false;
}

int ActionPredictor::getRandomPredictedSkelListSize(int modelID, int actionID)
{
	return m_randomPredictedSkeletonIdList[modelID][ActionFeature::ActionPhase::EndAction][actionID].size();
}

int ActionPredictor::getSkelIDFromRandomPredictedSkelList(int modelID, int actionID, int idInList)
{
	return m_randomPredictedSkeletonIdList[modelID][ActionFeature::ActionPhase::EndAction][actionID][idInList];
}


int ActionPredictor::getNewLocationNumOfSkel(int modelID, int actionID, int skelID)
{
	return m_predictedSkeletonsForActions[modelID][ActionFeature::ActionPhase::EndAction][actionID][skelID]->getPredictedLocationNum();
}

std::vector<int> ActionPredictor::getRandomPredictedSkelList(int modelID, int actionID)
{
	return m_randomPredictedSkeletonIdList[modelID][ActionFeature::ActionPhase::EndAction][actionID];
}

void ActionPredictor::setSkeletonPicked(int modelID, int actionID, int skelID, bool state)
{
	m_predictedSkeletonsForActions[modelID][ActionFeature::ActionPhase::EndAction][actionID][skelID]->setPicked(state);
}

MathLib::Vector3 ActionPredictor::getNewLocation(int modelID, int actionID, int skelID, int locationID)
{
	return m_predictedSkeletonsForActions[modelID][ActionFeature::ActionPhase::EndAction][actionID][skelID]->getPredictedLocation(locationID).pos;
}


