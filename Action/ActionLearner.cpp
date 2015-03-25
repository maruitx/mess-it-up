
#include "ActionLearner.h"
#include "../Geometry/Scene.h"
#include "../Geometry/Skeleton.h"
#include "../Geometry/SkeletonSampler.h"
#include "../Kinect/RgbdViewer.h"
#include "ActionFeature.h"

ActionLearner::ActionLearner(QObject *parent)
	: QObject(parent)
{
	m_hasJob = false;
//	m_showSampledSkeleton = false;

	scanToSceneAxisTransMat << 1.0, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 0,
		0, 0, 0, 1;
}

ActionLearner::~ActionLearner()
{

}

//void ActionLearner::init(RgbdViewer *viewer, CScene *s, Starlab::DrawArea *area)
//{
//	m_rgbdViewer = viewer;
//	m_scene = s;
//	m_drawArea = area;
//}

void ActionLearner::init(mess_mode *m)
{
	m_rgbdViewer = m->rgbdViewer;
	m_scene = m->getScene();
	m_drawArea = m->drawArea();

	m_currFrameId = 0;

	m_scanToSceneMat = Eigen::Matrix4d::Identity();
	m_sceneToScanMat = Eigen::Matrix4d::Identity();
}

// align scene to scan by changing the camera dir
void ActionLearner::alignSceneToScan()
{
	float scanRotMat[3][3];
	for (int i = 0; i < 3;i++)
	{
		for (int j = 0; j < 3;j++)
		{
			scanRotMat[i][j] = m_scanToSceneMat(i, j);
		}
	}

	qglviewer::Quaternion scanRotQ, newRotQ;
	scanRotQ.setFromRotationMatrix(scanRotMat);
	qglviewer::Vec rotAxis = scanRotQ.axis();
	qglviewer::Vec up(0, 0, 1);

	if (rotAxis*up < 0)
	{
		newRotQ.setAxisAngle(rotAxis, 1-scanRotQ.angle());
	}
	else
	{
		newRotQ.setAxisAngle(rotAxis, scanRotQ.angle());
	}

	newRotQ.getRotationMatrix(scanRotMat);
	Eigen::Matrix4d viewTransMat = m_scanToSceneMat;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			viewTransMat(i, j) = scanRotMat[i][j];
		}
	}

	qglviewer::Vec cameraViewVec = m_drawArea->camera()->viewDirection();
	Eigen::Vector4d newViewVec(cameraViewVec[0], cameraViewVec[1], cameraViewVec[2], 0);
	m_alignViewDir = viewTransMat*newViewVec;

	m_drawArea->camera()->setViewDirection(qglviewer::Vec(m_alignViewDir[0], m_alignViewDir[1], m_alignViewDir[2]));
	m_drawArea->showEntireScene();
	m_drawArea->updateGL();

	// store aligned frame matrix
	m_drawArea->camera()->frame()->getMatrix(m_alignedFrameMat);

	//
	m_sceneToScanMat = m_scanToSceneMat.inverse();
	
	// the scan is scaled up incorrectly

	m_scene->setSceneTransMat(m_sceneToScanMat);
}

void ActionLearner::loadScanToSceneTransMat()
{
	QString filename = m_jobFilePath + "Record/" + m_scanFileName + "/" + m_scanFileName + ".transMat";

	QFile inFile(filename);
	if (!inFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	QTextStream ifs(&inFile);

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4;j++)
		{
			ifs >> m_scanToSceneMat(i, j);
		}
	}

	inFile.close();
}

bool ActionLearner::loadJob(const QString &filename)
{
	//load job file
	QFile inFile(filename);
	QFileInfo inFileInfo(inFile.fileName());

	if (!inFile.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		Simple_Message_Box("Cannot open Job file");
		return false;
	}

	QTextStream ifs(&inFile);

	QString optionNameStr;
	int optionValue;
	QVector<QString> trackingObjName;

	ifs >> optionNameStr;

	while (!optionNameStr.isEmpty())
	{
		if (optionNameStr == "Scene")
		{
			ifs >> m_sceneFileName;
		}

		if (optionNameStr == "Scan")
		{
			ifs >> m_scanFileName;
		}

		if (optionNameStr == "Tracking")
		{
			int trackNum;
			ifs >> trackNum;

			m_trackingObjID.resize(trackNum);
			trackingObjName.resize(trackNum);

			for (unsigned int i = 0; i < trackNum;i++)
			{
				ifs >> trackingObjName[i];
			}
		}

		ifs >> optionNameStr;
	}
	
	inFile.close();

	m_jobFilePath = inFileInfo.absolutePath() + "/";	
	m_scene->loadScene(m_jobFilePath + "Scene/" + m_sceneFileName + "/" + m_sceneFileName + "/" + m_sceneFileName + ".txt");
	m_scene->setSceneDrawArea(m_drawArea);
	
	m_rgbdViewer->createWidget();
	m_rgbdViewer->loadScan(m_jobFilePath + "Record/" + m_scanFileName + ".txt");
		
	// load transformation matrix from scan to scene
	loadScanToSceneTransMat();
	
	// skeleton need to be transformed to same coord frame of scene
	setSkeletonStream();

	// set tracking object
	for (unsigned int i = 0; i < trackingObjName.size(); i++)
	{
		m_trackingObjID[i] = m_scene->getModelIdByName(trackingObjName[i]);
	}

	m_actionFeatures.resize(ACTION_NUM);
//	m_actionRepSkeletons.resize(ACTION_NUM);

	m_hasJob = true;

	return true;
}

void ActionLearner::resetAlignView()
{
	m_drawArea->camera()->frame()->setFromMatrix(m_alignedFrameMat);
	m_drawArea->showEntireScene();
	m_drawArea->updateGL();
}

void ActionLearner::setSkeletonStream()
{
	QVector<QVector<Vector4>>& skeletonStream = m_rgbdViewer->depthSensor->getSkeletonStream();

	for (int i = 0; i < skeletonStream.size(); i++)
	{
		QVector<Eigen::Vector4d> skeletonJoints(skeletonStream[i].size());

		int f = 1;
		if (m_rgbdViewer->depthSensor->isFlipXY())
		{
			f = -1;
		}

		for (int id = 0; id < skeletonJoints.size();id++)
		{
			// may need to filp loaded skeleton
			Eigen::Vector4d joint(f*skeletonStream[i][id].x, skeletonStream[i][id].y, skeletonStream[i][id].z, 1);	

			joint = m_scanToSceneMat*joint;
			skeletonJoints[id] = joint;
		}

		Skeleton* s = new Skeleton(skeletonJoints);
		m_skeletonStream.push_back(s);
	}	
}

void ActionLearner::drawSkeleton()
{
	m_skeletonStream[m_currFrameId]->draw();


}

void ActionLearner::updateSkeletonInteract()
{
	m_scene->testInteractSkeleton(m_skeletonStream[m_currFrameId]);
}

void ActionLearner::syncWithScan()
{
	updateFrameId();
	updateSkeletonInteract();

	if (m_rgbdViewer->depthSensor->hasTrackedTransMat())
	{
		updateModelTransMat();
	}

	if (m_rgbdViewer->depthSensor->isShowOfflineTrackingResult())
	{
		m_rgbdViewer->depthSensor->computeTrackResultCloud();
	}
}

void ActionLearner::updateFrameId()
{
	m_currFrameId = m_rgbdViewer->depthSensor->getCurrFrameID();
}

void ActionLearner::setTrackingObj()
{
	m_rgbdViewer->depthSensor->setTrackRefCloud(m_scene->getSelectedPointCloud(m_trackingObjID), m_sceneToScanMat);

	m_rgbdViewer->depthSensor->loadTrackedTransMat();
}

void ActionLearner::updateModelTransMat()
{
	QVector<QVector<Eigen::Matrix4d>>&  trackTransMat = m_rgbdViewer->depthSensor->getTrackedTransMat();

	for (int i = 0; i < m_trackingObjID.size();i++)
	{
		int modelID = m_trackingObjID[i];

		// tracking info is captured in scan coordinate frame
		// first transform scene data to scan frame, then apply tracking transform, and transform back
		Eigen::Matrix4d transMat = m_scanToSceneMat*trackTransMat[i][m_currFrameId]*m_sceneToScanMat;
		
		m_scene->setModelTransMat(modelID, transMat);
	}
}

// analyze labeled data to extract instances of action
// action instances is unsorted, it depends on the labeling data
void ActionLearner::extractActionInstances()
{
	m_actionInstances.clear();

	QVector<QSet<FrameLabel>>& frameLabels = m_rgbdViewer->depthSensor->getFrameActionLabels();

	QSet<FrameLabel> previousLabelSet, currentLabelSet;

	for (int i = 0; i < frameLabels.size(); i++)
	{
		currentLabelSet = frameLabels[i];
		
		QSet<FrameLabel>::iterator curr_it, pre_it;

		for (curr_it = currentLabelSet.begin(); curr_it != currentLabelSet.end(); curr_it++)
		{
			if (previousLabelSet.find(*curr_it)==previousLabelSet.end())
			{
				ActionInstance newInstances;
				newInstances.actionID = curr_it->first;
				newInstances.modelID = curr_it->second;
				newInstances.actionLabel = QString(Action_Labels[curr_it->first]);
				newInstances.startFrameID = i;
				newInstances.endFrameID = i;  // will keep update
				newInstances.isCompleted = false;
				m_actionInstances.push_back(newInstances);
			}
			else
			{
				for (int inst_id = 0; inst_id < m_actionInstances.size(); inst_id++)
				{
					// recording the growth of new instance
					if (curr_it->first == m_actionInstances[inst_id].actionID && !m_actionInstances[inst_id].isCompleted)
					{
						m_actionInstances[inst_id].endFrameID++;
						break;
					}
				}
			}
		}

		for (pre_it = previousLabelSet.begin(); pre_it != previousLabelSet.end(); pre_it++)
		{
			if (currentLabelSet.find(*pre_it) == currentLabelSet.end())
			{
				for (int inst_id = 0; inst_id < m_actionInstances.size(); inst_id++)
				{
					if (pre_it->first == m_actionInstances[inst_id].actionID && !m_actionInstances[inst_id].isCompleted)
					{
						m_actionInstances[inst_id].isCompleted = true;
						break;
					}
				}
			}
			else if (i == frameLabels.size() - 1)  // if one action lasts to last frame, just complete this action
			{
				for (int inst_id = 0; inst_id < m_actionInstances.size(); inst_id++)
				{
					if (pre_it->first == m_actionInstances[inst_id].actionID && !m_actionInstances[inst_id].isCompleted)
					{
						m_actionInstances[inst_id].isCompleted = true;
						break;
					}
				}
			}	
		}

		previousLabelSet = currentLabelSet;
	}
}

void ActionLearner::startLearning()
{
	extractActionInstances();

	m_actionFeatures.clear();
	//m_actionRepSkeletons.clear();  

	m_actionFeatures.resize(ACTION_NUM);
	//m_actionRepSkeletons.resize(ACTION_NUM);

	for (int i = 0; i < m_actionInstances.size(); i++)
	{
		ActionFeature newFeature(this);
		newFeature.setActionInstance(i);
		newFeature.extractFeature();

		// record action feature for each type of action
		m_actionFeatures[newFeature.actionID()].push_back(newFeature);

		std::vector<Skeleton*> repSkeletons = newFeature.getActionRepSkeletons();

		//collect rep skeletons from all instances, need to re-think
		//m_actionRepSkeletons[newFeature.actionID()].insert(m_actionRepSkeletons[newFeature.actionID()].end(), repSkeletons.begin(), repSkeletons.end());
	}

	saveExtractedFeatures();
	saveActionRepSkels();

	Simple_Message_Box("Action learning done");
}

QVector<QPair<int, Eigen::Matrix4d>> ActionLearner::getModelTrackMat(int frame_id)
{
	QVector<QPair<int, Eigen::Matrix4d>> trackMats;

	QVector<QVector<Eigen::Matrix4d>>&  trackTransMat = m_rgbdViewer->depthSensor->getTrackedTransMat();

	for (int i = 0; i < m_trackingObjID.size(); i++)
	{
		int modelID = m_trackingObjID[i];

		// tracking info is captured in scan coordinate frame
		// first transform scene data to scan frame, then apply tracking transform, and transform back
		Eigen::Matrix4d transMat = m_scanToSceneMat*trackTransMat[i][frame_id] * m_sceneToScanMat;

		QPair<int, Eigen::Matrix4d> track_mat(modelID, transMat);
		trackMats.push_back(track_mat);
	}

	return trackMats;
}

void ActionLearner::saveExtractedFeatures()
{
	QString featureFilePath = m_jobFilePath + "Feature/Train/";
	QVector<QString> labels;

	// for each action
	for (int i = 0; i < m_actionFeatures.size(); i++)
	{
		if (m_actionFeatures[i].size() > 0)
		{
			QString filename = featureFilePath + QString(Action_Labels[i]) + ".txt";
			
			labels.push_back(Action_Labels[i]); // collect the available labels for weka output

			QFile outFile(filename);
			QTextStream out(&outFile);

			if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

			// save action features for each instance
			for (int j = 0; j < m_actionFeatures[i].size(); j++)
			{
				std::map<int, std::vector<double>>& features = m_actionFeatures[i][j].getFeatureVector();

				std::map<int, std::vector<double>>::iterator it;
				for (it = features.begin(); it != features.end(); it++)
				{
					foreach(double d, it->second)
					{
						out << d << " ";
					}

					out << "\n";
				}
			}

			outFile.close();
		}
	}

	// save to arff format for weka
	QString filename = featureFilePath + "actions.arff";

	QFile outFile(filename);
	QTextStream out(&outFile);

	if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;

	out << "@Relation " << m_scanFileName << "\n";

	// fill in the attribute
	for (int i = 0; i < m_actionFeatures.size(); i++)
	{
		if (m_actionFeatures[i].size() > 0)
		{
			for (int j = 0; j < m_actionFeatures[i][0].featureDim(); j++)
			{
				out << "@ATTRIBUTE " << j << " REAL" << "\n";
			}

			out << "@ATTRIBUTE class " << "{";

			for (int k = 0; k < labels.size() - 1; k++)
			{
				out << labels[k] << ",";
			}

			out << labels[labels.size() - 1] << "}\n";
		}

		break;
	}

	// fill in the data
	out << "@DATA\n";
	for (int i = 0; i < m_actionFeatures.size(); i++)
	{
		if (m_actionFeatures[i].size() > 0)
		{
			// for each instance
			for (int j = 0; j < m_actionFeatures[i].size(); j++)
			{
				std::map<int, std::vector<double>>& features = m_actionFeatures[i][j].getFeatureVector();

				std::map<int, std::vector<double>>::iterator it;
				for (it = features.begin(); it != features.end(); it++)
				{
					foreach(double d, it->second)
					{
						out << d << ",";
					}

					out << QString(Action_Labels[i]) << "\n";
				}
			}
		}
	}

	outFile.close();

}

void ActionLearner::saveActionRepSkels()
{
	QString featureFilePath = m_jobFilePath + "Feature/Train/";

	for (int i = 0; i < m_actionFeatures.size(); i++)
	{
		if (m_actionFeatures[i].size() > 0)
		{
			QString filename = featureFilePath + QString(Action_Labels[i]) + ".skel";
			QFile outFile(filename);
			QTextStream out(&outFile);

			if (!outFile.open(QIODevice::ReadWrite | QIODevice::Text)) return;
			for (int j = 0; j < m_actionFeatures[i].size(); j++)
			{
				std::vector<Skeleton*> repSkeletons = m_actionFeatures[i][j].getActionRepSkeletons();

				for (int si = 0; si < repSkeletons.size(); si++)
				{
					std::vector<MathLib::Vector3> joints = repSkeletons[si]->getNormalizedJoints();

					for (int jt = 0; jt < joints.size(); jt++)
					{
						out << joints[jt].x << " " << joints[jt].y << " " << joints[jt].z << " ";
					}

					out << "\n";
				}
			}

			outFile.close();
		}
	}
}

void ActionLearner::updateDrawArea()
{
	m_drawArea->updateGL();
}