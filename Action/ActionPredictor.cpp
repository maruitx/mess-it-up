#include "ActionPredictor.h"
#include "../Geometry/Scene.h"
#include "../Geometry/Skeleton.h"
#include "../Geometry/SkeletonSampler.h"

ActionPredictor::ActionPredictor(QObject *parent)
	: QObject(parent)
{
	m_showSampledSkeleton = false;
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

bool ActionPredictor::loadTestScene(const QString &filename)
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

	inFile.close();

	m_jobFilePath = inFileInfo.absolutePath() + "/";
	m_scene->loadScene(m_jobFilePath + "Scene/" + m_sceneFileName + "/" + m_sceneFileName + "/" + m_sceneFileName + ".txt");
	m_scene->setSceneDrawArea(m_drawArea);

	return true;
}

void ActionPredictor::loadTrainingResult()
{
	loadActionRepSkels();

	// load learned classifiers

	Simple_Message_Box("Training result loaded");
}

void ActionPredictor::startPredicting()
{
	if (m_actionRepSkeletons.size() == 0)
	{
		Simple_Message_Box("Please load training data first");
		return;
	}

	m_skeletonSampler = new SkeletonSampler(m_scene);
	m_sampledSkeletonsForActions.clear();

	m_scene->voxelizeModels();  // voxelize model and build octree

	// predict possible action for each model
	for (int modelID = 0; modelID < m_scene->getModelNum(); modelID++)
	{
		m_sampledSkeletonsForActions[modelID].resize(m_actionRepSkeletons.size());
		// test for each action
		// random sample an/several actions from the action library
		for (int actionID = 0; actionID < m_actionRepSkeletons.size(); actionID++)
		{
			if (m_actionRepSkeletons[actionID].size() > 0)
			{
				int k = std::rand() % m_actionRepSkeletons[actionID].size(); // random select
				m_skeletonSampler->setSkeleton(m_actionRepSkeletons[actionID][k]);
				m_skeletonSampler->sampleSkeletonAroundModel(modelID);

				SkeletonPtrList sampledSkeletons = m_skeletonSampler->getSampledSkeletons();
				m_sampledSkeletonsForActions[modelID][actionID] = sampledSkeletons;
			}
		}
	}

	genRandomSkeletonList(10);

	m_showSampledSkeleton = true;
	m_showSampeRegion = true;
	m_finishPredict = true;

	Simple_Message_Box("Action prediction done");
}

void ActionPredictor::loadActionRepSkels()
{
	m_actionRepSkeletons.clear();
	m_actionRepSkeletons.resize(ACTION_NUM);

	QString featureFilePath = m_jobFilePath + "Feature/Train/";

	for (int actionID = 0; actionID < m_actionRepSkeletons.size(); actionID++)
	{
		QString filename = featureFilePath + QString(Action_Labels[actionID]) + ".skel";
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
			m_actionRepSkeletons[actionID].push_back(newSkel);
		}

		inFile.close();
	}
}

void ActionPredictor::genRandomSkeletonList(int num)
{
	std::map<int, std::vector<SkeletonPtrList>>::iterator it;

	for (it = m_sampledSkeletonsForActions.begin(); it != m_sampledSkeletonsForActions.end(); it++)
	{
		int modelID = it->first;

		m_randomSkeletonIdList[modelID].resize(m_actionRepSkeletons.size());

		for (int actionID = 0; actionID < m_actionRepSkeletons.size(); actionID++)
		{
			int actionNum = it->second[actionID].size();
			if (actionNum > 0)
			{
				if (actionNum < num)
				{
					num = actionNum;
				}

				std::vector<int> randIdList(num);

				for (int i = 0; i < num; i++)
				{
					int skelID = std::rand() % m_sampledSkeletonsForActions[modelID][actionID].size();
					randIdList[i] = skelID;
				}

				m_randomSkeletonIdList[modelID][actionID] = randIdList;
			}
		}
	}
}

void ActionPredictor::drawSampledSkeletons(int modelID, int actionID)
{
	if (m_sampledSkeletonsForActions[modelID][actionID].size() > 0)
	{
		for (int i = 0; i < m_randomSkeletonIdList[modelID][actionID].size(); i++)
		{
			int skeID = m_randomSkeletonIdList[modelID][actionID][i];
			m_sampledSkeletonsForActions[modelID][actionID][skeID]->draw();
		}

		// show sample region
		drawSampleRange(modelID);
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
	return m_sampledSkeletonsForActions[modelID][actionID].size();
}

void ActionPredictor::resampleSkeleton(int num)
{
	m_randomSkeletonIdList.clear();
	genRandomSkeletonList(num);
}

