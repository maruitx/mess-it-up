#include "Skeleton.h"
#include "Scene.h"
#include "SuppPlane.h"
#include "qgl.h"
#include "qglviewer/quaternion.h"
#include "../Utilities/CustomDrawObjects.h"

QColor colorset[2][2] = { { QColor(255, 0, 0), QColor(0, 255, 0) }, { QColor(220, 0, 220), QColor(0, 220, 220) } };

Skeleton::Skeleton()
{
}

Skeleton::Skeleton(QVector<Eigen::Vector4d> pts)
	:m_joints(pts)
{

}

Skeleton::Skeleton(std::vector<MathLib::Vector3> pts)
{
	m_joints.resize(pts.size());

	for (int i = 0; i < pts.size(); i++)
	{
		Eigen::Vector4d joint;
		joint[0] = pts[i].x;
		joint[1] = pts[i].y;
		joint[2] = pts[i].z;
		joint[3] = 1.0;

		m_joints[i] = joint;
	}
}


Skeleton::~Skeleton()
{
}

void Skeleton::draw(int drawMode/*= 0*/)
{
	glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT | GL_HINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);
	glDisable(GL_TEXTURE_2D);

	glDisable(GL_LIGHTING);
	glEnable(GL_LINE_STIPPLE);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	for (int i = 0; i < 19; i++)
	{
		drawBone(i, colorset[drawMode][1]);
	}

	for (int i = 0; i < m_joints.size(); i++)
	{
		drawJoint(i, colorset[drawMode][0]);
	}

	glPopAttrib();
}

void Skeleton::drawJoint(int id, QColor c /*= QColor(255,0,0)*/)
{
	float r = 3*PointSize3D;
	if (id == HEAD)
	{
		r = 3 * r;
	}

	glColorQt(c);
	renderSphere(m_joints[id][0], m_joints[id][1], m_joints[id][2], r);
}

void Skeleton::drawBone(int id, QColor c /*= QColor(0,0,255)*/)
{
	int firstJoint = BoneMap[id][0];
	int secondJoint = BoneMap[id][1];

	glColorQt(c);
	glLineWidth(5.0);

	glBegin(GL_LINES);
	glVertex3d(m_joints[firstJoint][0], m_joints[firstJoint][1], m_joints[firstJoint][2]);
	glVertex3d(m_joints[secondJoint][0], m_joints[secondJoint][1], m_joints[secondJoint][2]);
	glEnd();	
}

std::vector<MathLib::Vector3> Skeleton::getJoints()
{
	std::vector<MathLib::Vector3> joints(m_joints.size());

	for (int i = 0; i < joints.size();i++)
	{
		joints[i] = MathLib::Vector3(m_joints[i][0], m_joints[i][1], m_joints[i][2]);
	}

	return joints;
}

std::vector<SurfaceMesh::Vector3> Skeleton::getSurfaceMeshJoints()
{
	std::vector<SurfaceMesh::Vector3> joints(m_joints.size());

	for (int i = 0; i < joints.size(); i++)
	{
		joints[i] = SurfaceMesh::Vector3(m_joints[i][0], m_joints[i][1], m_joints[i][2]);
	}

	return joints;
}

std::vector<MathLib::Vector3> Skeleton::getNormalizedJoints()
{
	std::vector<MathLib::Vector3> joints(m_joints.size());

	for (int i = 0; i < joints.size(); i++)
	{
		joints[i] = MathLib::Vector3(m_joints[i][0] - m_joints[0][0], m_joints[i][1] - m_joints[0][1], m_joints[i][2]);
	}

	return joints;
}

std::vector<MathLib::Vector3> Skeleton::getTransformedJoints(double transX, double transY, double transZ, double thetaZ, MathLib::Vector3 upright)
{
	qglviewer::Quaternion rotQ(qglviewer::Vec(upright.x, upright.y, upright.z), thetaZ);

	float rotMatData[3][3];
	rotQ.getRotationMatrix(rotMatData);

	Eigen::Matrix4d rotMat = Eigen::Matrix4d::Identity();

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			rotMat(i, j) = rotMatData[i][j];
		}
	}

	std::vector<MathLib::Vector3> trans_joints(m_joints.size());

	for (int i = 0; i < trans_joints.size(); i++)
	{
		Eigen::Vector4d joint = rotMat*m_joints[i];

		trans_joints[i] = MathLib::Vector3(joint[0] + transX, joint[1] + transY, joint[2] + transZ);
	}

	return trans_joints;
}

MathLib::Vector3 Skeleton::getJoint(int id)
{
	return MathLib::Vector3(m_joints[id][0], m_joints[id][1], m_joints[id][2]);
}

// align hip center to z axis
void Skeleton::AlignToOrigin()
{
	double transX = m_joints[0][0];
	double transY = m_joints[0][1];

	for (int i = 0; i < m_joints.size(); i++)
	{
		m_joints[i][0] -= transX;
		m_joints[i][1] -= transY;
	}
}

void Skeleton::computePlacementRegion()
{
	std::vector<double> skelReachRegion(4);
	skelReachRegion[0] = m_sampledPos.x - ReachDistThreshold; // xmin
	skelReachRegion[1] = m_sampledPos.x + ReachDistThreshold; // xmax
	skelReachRegion[2] = m_sampledPos.y - ReachDistThreshold; // ymin
	skelReachRegion[3] = m_sampledPos.y + ReachDistThreshold; // ymax

	// test intersection for supp planes of each model
	// important, need to fix: exclude the interacted center object when computing
	for (int m_id = 0; m_id < m_scene->getModelNum(); m_id++)
	{
		CModel *m = m_scene->getModel(m_id);

		// need to fix: consider transform of support planes, m_transMat * planeAABBCorners;
		std::vector<SuppPlane*> allSuppPlanes = m->getAllSuppPlanes();

		for (int i = 0; i < allSuppPlanes.size(); i++)
		{
			std::vector<double> planeAABBCorners = allSuppPlanes[i]->convertToAABBPlane();

			double x_max_min = std::max(skelReachRegion[0], planeAABBCorners[0]); // max of xmin
			double x_min_max = std::min(skelReachRegion[1], planeAABBCorners[1]); // min of xmax

			double y_max_min = std::max(skelReachRegion[2], planeAABBCorners[2]); // max of ymin
			double y_min_max = std::min(skelReachRegion[3], planeAABBCorners[3]); // min of ymax

			double x_overlap = std::max(x_min_max - x_max_min, 0.0);
			double y_overlap = std::max(y_min_max - y_max_min, 0.0);

			double overlapArea = x_overlap*y_overlap;

			if (overlapArea > Area_Threshold)
			{
				m_potentialObjPlacement.potentialSuppPlanesToPlace.push_back(allSuppPlanes[i]);

				std::vector<double> intersectionPlane(6);
				intersectionPlane[0] = x_max_min;
				intersectionPlane[1] = x_min_max;
				intersectionPlane[2] = y_max_min;
				intersectionPlane[3] = y_min_max;
				intersectionPlane[4] = allSuppPlanes[i]->GetZ() + 0.01;
				intersectionPlane[5] = m_id;

				m_potentialObjPlacement.accessPlanesToPlace.push_back(intersectionPlane);
			}
		}
	}

	if (m_potentialObjPlacement.hasAccessiblePlanes())
	{
		m_potentialObjPlacement.setScene(m_scene);
	}
}

void Skeleton::drawPotentialPlacement()
{
	if (m_potentialObjPlacement.hasAccessiblePlanes()>0)
	{
		m_potentialObjPlacement.drawAccessPlanes(QColor(255, 255, 80));
		m_potentialObjPlacement.drawSampledLocations(QColor(20,255,40));

	}
}

void Skeleton::sampleObjPlacementPosNearby()
{
	m_potentialObjPlacement.samplePotentialLocation();
}

bool ObjPotentialPlacement::randomSampleOnAccessPlanes(MathLib::Vector3 &samplePos)
{
	bool isPosValid = false;

	if (accessPlanesToPlace.size() > 0)  // start phase skeleton doesn't compute the access planes 
	{
		int planeID = std::rand() % accessPlanesToPlace.size();

		double rand_x = (double)std::rand() / (double)RAND_MAX;
		double rand_y = (double)std::rand() / (double)RAND_MAX;

		double length = accessPlanesToPlace[planeID][1] - accessPlanesToPlace[planeID][0];
		double width = accessPlanesToPlace[planeID][3] - accessPlanesToPlace[planeID][2];

		samplePos.x = accessPlanesToPlace[planeID][0] + rand_x*length;
		samplePos.y = accessPlanesToPlace[planeID][2] + rand_y*width;
		samplePos.z = accessPlanesToPlace[planeID][4];

		int modelID = accessPlanesToPlace[planeID][5];

		CModel *m = m_scene->getModel(modelID);
		Eigen::Matrix4d modelTransMat = m->getTransMat();
		Eigen::Vector4d transSamplePos = modelTransMat.inverse()*Eigen::Vector4d(samplePos[0], samplePos[1], samplePos[2], 0);

		MathLib::Vector3 startPos = MathLib::Vector3(transSamplePos[0], transSamplePos[1], transSamplePos[2]);
		MathLib::Vector3 rayDir = MathLib::Vector3(0, 0, -1);

		double depthVal = 1e6;
		MathLib::Vector3 faceNormal(0, 0, 1);
		int faceID = m->PickByRay(startPos, rayDir, depthVal, faceNormal);

		if (faceID != -1)
		{
			if (MathLib::Acos(faceNormal.dot(m_scene->getUprightVec())) < AngleThreshold)
			{
				double hitPosZ = startPos.z + rayDir.z*depthVal;

				if ( abs(hitPosZ - startPos.z) < 0.02)
				{
					isPosValid = true;
				}				
			}
		}
	}

	return isPosValid;
}

void ObjPotentialPlacement::samplePotentialLocation()
{
	int iterCount = 0;

	while (iterCount < 100)
	{
		if (sampledLocation.size() == 5) return;

		MathLib::Vector3 samplePos;	

		if (randomSampleOnAccessPlanes(samplePos))
		{
			sampledLocation.push_back(samplePos);
		}
		
		iterCount++;
	}
}

void ObjPotentialPlacement::drawAccessPlanes(QColor c)
{
	glDisable(GL_LIGHTING);
	glColorQt(c);

	for (int planeID = 0; planeID < accessPlanesToPlace.size(); planeID++)
	{
		glBegin(GL_QUADS);
		glVertex3f(accessPlanesToPlace[planeID][0], accessPlanesToPlace[planeID][2], accessPlanesToPlace[planeID][4]);
		glVertex3f(accessPlanesToPlace[planeID][1], accessPlanesToPlace[planeID][2], accessPlanesToPlace[planeID][4]);
		glVertex3f(accessPlanesToPlace[planeID][1], accessPlanesToPlace[planeID][3], accessPlanesToPlace[planeID][4]);
		glVertex3f(accessPlanesToPlace[planeID][0], accessPlanesToPlace[planeID][3], accessPlanesToPlace[planeID][4]);
		glEnd();
	}

	glEnable(GL_LIGHTING);
}

void ObjPotentialPlacement::drawSampledLocations(QColor c)
{
	glDisable(GL_LIGHTING);
	glColorQt(c);

	for (int i = 0; i < sampledLocation.size(); i++)
	{
		float r = PointSize3D;
		renderSphere(sampledLocation[i][0], sampledLocation[i][1], sampledLocation[i][2], r);
	}

	glEnable(GL_LIGHTING);
}

