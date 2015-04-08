#include "ActionPredictor.h"
#include "../Geometry/Scene.h"
#include "../Geometry/Skeleton.h"
#include "../Geometry/SkeletonSampler.h"
#include "ActionFeature.h"

double LABEL_DIFF_TH = 0.1;

ActionPredictor::ActionPredictor(QObject *parent)
	: QObject(parent)
{
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

	Simple_Message_Box("Training result loaded");
}

void ActionPredictor::loadClassifiers()
{
	m_classifiers.resize(ACTION_PHASE_NUM);
	QString featureFilePath = m_jobFilePath + "Feature/Train/";

	for (int phase_id = 0; phase_id < ACTION_PHASE_NUM; phase_id++)
	{
		if (isPhaseConsidered(phase_id))
		{
			QString filename = featureFilePath + QString(Action_Phase_Names[phase_id]) + "_actions.RFClassifier";
			QFile classifierFile(filename);
			
			if (classifierFile.exists())
			{
				m_classifiers[phase_id] = new OpenCVClassifier<cv::ml::RTrees>();
				m_classifiers[phase_id]->load_classifier(filename.toStdString());
			}
		}
	}
}

void ActionPredictor::startPredicting()
{
	if (m_loadedSkeletonsForTest.size() == 0)
	{
		Simple_Message_Box("Please load training data first");
		return;
	}

	m_skeletonSampler = new SkeletonSampler(m_scene);
	m_sampledSkeletonsForActions.clear();

	m_scene->voxelizeModels();  // voxelize model and build octree

	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		m_sampledSkeletonsForActions[modelID].resize(ACTION_PHASE_NUM);
	}
	
	sampleSkeletons();

	genRandomSkeletonListForDisplay(5);

	m_showSampledSkeleton = true;
	m_showStartPose = true;
	m_showEndPose = false;

	m_showSampeRegion = true;
	m_finishPredict = true;

	Simple_Message_Box("Action prediction done");
}

void ActionPredictor::loadActionRepSkels()
{
	m_loadedSkeletonsForTest.resize(ACTION_PHASE_NUM);

	for (int phase_id = 0; phase_id < ACTION_PHASE_NUM; phase_id++)
	{
		loadActionRepSkels(phase_id);
	}
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
			m_loadedSkeletonsForTest[phaseID][actionID].push_back(newSkel);
		}

		inFile.close();
	}
}

void ActionPredictor::genRandomSkeletonListForDisplay(int num)
{
	ModelRelatedSkeletonList::iterator it;

	for (it = m_sampledSkeletonsForActions.begin(); it != m_sampledSkeletonsForActions.end(); it++)
	{
		int modelID = it->first;

		m_randomSkeletonIdList[modelID].resize(ACTION_PHASE_NUM);

		for (int phaseID = 0; phaseID < ACTION_PHASE_NUM; phaseID++)
		{
			m_randomSkeletonIdList[modelID][phaseID].resize(ACTION_NUM);

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

					m_randomSkeletonIdList[modelID][phaseID][actionID] = randIdList;
				}
			}
		}
	}
}

void ActionPredictor::drawSampledSkeletons(int modelID, int phaseID, int actionID)
{
	if (m_sampledSkeletonsForActions[modelID][phaseID][actionID].size() > 0)
	{
		for (int i = 0; i < m_randomSkeletonIdList[modelID][phaseID][actionID].size(); i++)
		{
			int skeID = m_randomSkeletonIdList[modelID][phaseID][actionID][i];
			m_sampledSkeletonsForActions[modelID][phaseID][actionID][skeID]->draw(phaseID);
		}

		// show sample region
		//drawSampleRange(modelID);
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

	glPushAttrib(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glDepthMask(GL_FALSE);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glColor4f(red[0], red[1], red[2], 0.1f);

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
	for (int i = 0; i < 4; i++) {
		glNormal3f(upright.x, upright.y, upright.z);
		glVertex3d(sampleRange[0], sampleRange[2], 0);
		glVertex3d(sampleRange[1], sampleRange[2], 0);
		glVertex3d(sampleRange[0], sampleRange[3], 0);

		glNormal3f(upright.x, upright.y, upright.z);
		glVertex3d(sampleRange[0], sampleRange[3], 0);
		glVertex3d(sampleRange[1], sampleRange[2], 0);
		glVertex3d(sampleRange[1], sampleRange[3], 0);
	}
	glEnd();
	glPopAttrib();
}

int ActionPredictor::getSampledSkelNum(int modelID, int actionID)
{
	return m_sampledSkeletonsForActions[modelID][0][actionID].size();
}

void ActionPredictor::resampleSkeletonForDisplay(int num)
{
	m_randomSkeletonIdList.clear();
	genRandomSkeletonListForDisplay(num);
}

void ActionPredictor::sampleSkeletons()
{
	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		for (int phaseID = 0; phaseID < ACTION_PHASE_NUM; phaseID++)
		{
			m_sampledSkeletonsForActions[modelID][phaseID].resize(ACTION_NUM);
		}
	}

	for (int phase_id = 0; phase_id < ACTION_PHASE_NUM; phase_id++)
	{
		sampleSkeletonsForActionPhrase(phase_id);
	}
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
			if (m_loadedSkeletonsForTest[phaseID][action_id].size() > 0)
			{
				int k = std::rand() % m_loadedSkeletonsForTest[phaseID][action_id].size(); // random select
				m_skeletonSampler->setSkeleton(m_loadedSkeletonsForTest[phaseID][action_id][k]);
				m_skeletonSampler->sampleSkeletonAroundModel(model_id);

				SkeletonPtrList sampledSkeletons = m_skeletonSampler->getSampledSkeletons();
				SkeletonPtrList classifiedSkeletons;

				// test with classifier
				for (int skel_id = 0; skel_id < sampledSkeletons.size(); skel_id++)
				{
					Skeleton *currSkel = sampledSkeletons[skel_id];
					if (m_classifiers[phaseID]!=nullptr && testForSkeletons(model_id, phaseID, action_id, currSkel))
					{
						classifiedSkeletons.push_back(currSkel);
					}									
				}

				m_sampledSkeletonsForActions[model_id][phaseID][action_id] = classifiedSkeletons;
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

bool ActionPredictor::testForSkeletons(int modelID, int phaseID, int actionID, Skeleton *skel)
{
		ActionFeature newFeature(m_scene);

		std::vector<double> actionFeature;

		newFeature.computeActionFeatureForSkel(skel, modelID, actionFeature);

		// test feature with classifier
		cv::Mat testFeatureMat;
		convertToMat(actionFeature, testFeatureMat);

		float predicted_id = m_classifiers[phaseID]->predict(testFeatureMat);

		if (std::abs(predicted_id - actionID) < LABEL_DIFF_TH)
		{
			return true;
		}
		else
		{
			return false;
		}
		
}

bool ActionPredictor::isPhaseConsidered(int phaseID)
{
	for (int action_id = 0; action_id < ACTION_NUM; action_id++)
	{
		if (m_loadedSkeletonsForTest[phaseID][action_id].size() > 0)
		{
			return true;
		}
	}

	return false;
}

