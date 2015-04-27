#include "Skeleton.h"
#include "Scene.h"
#include "SuppPlane.h"
#include "qgl.h"
#include "qglviewer/quaternion.h"
#include "../Utilities/CustomDrawObjects.h"

QColor colorset[2][2] = { { QColor(255, 0, 0), QColor(0, 255, 0) }, { QColor(220, 0, 220), QColor(0, 220, 220) } };
double JointRadius = 3 * PointRadius3D;
double HeadRadius = 3 * JointRadius;

Skeleton::Skeleton()
{
}

Skeleton::Skeleton(QVector<Eigen::Vector4d> pts):
m_joints(pts), m_isPicked(false)
{
}

Skeleton::Skeleton(std::vector<MathLib::Vector3> pts):
m_isPicked(false)
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

	if (m_isPicked)
	{
		drawOBB();
	}	
}

void Skeleton::drawJoint(int id, QColor c /*= QColor(255,0,0)*/)
{
	float r = JointRadius;
	if (id == HEAD)
	{
		r = HeadRadius;
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

				AcessPlane intersectionPlane;
				intersectionPlane.corners.resize(4);

				intersectionPlane.corners[0] = x_max_min;
				intersectionPlane.corners[1] = x_min_max;
				intersectionPlane.corners[2] = y_max_min;
				intersectionPlane.corners[3] = y_min_max;
				intersectionPlane.zVal = allSuppPlanes[i]->GetZ() + 0.01;
				intersectionPlane.modelID = m_id;
				intersectionPlane.suppPlaneID = allSuppPlanes[i]->getSuppPlaneID();

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
		m_potentialObjPlacement.drawAccessPlanes(QColor(255, 255, 80, 220));
		m_potentialObjPlacement.drawSampledLocations(QColor(20,255,40));

	}
}

void Skeleton::sampleObjPlacementPosNearby()
{
	m_potentialObjPlacement.samplePotentialLocation();
}

void Skeleton::computeOrientation()
{
	Eigen::Vector4d shoulderDir = m_joints[SHOULDER_LEFT] - m_joints[SHOULDER_RIGHT];
	m_shoulderDir = MathLib::Vector3(shoulderDir[0], shoulderDir[1], shoulderDir[2]);
	m_shoulderDir.normalize();

	Eigen::Vector4d torsoDir = m_joints[SHOULDER_CENTER] - m_joints[SPINE];
	m_torsoDir = MathLib::Vector3(torsoDir[0], torsoDir[1], torsoDir[2]);
	m_torsoDir.normalize();

	m_torsoNormal = m_shoulderDir.cross(m_torsoDir);
}

void Skeleton::computeOBB()
{
	std::vector<MathLib::Vector3> axis;
	MathLib::Vector3 center;
	MathLib::Vector3 obbSize;
	std::vector<MathLib::Vector3> vertices(8);

	// compute axis

	//         up
	//     n   ^
	//      \  |
	//       \ |
	//        \|	
	//	s<-----o

	axis.resize(3);
	axis[1] = m_scene->getUprightVec();

	double shoulderDirProjToUpVal = axis[1].dot(m_shoulderDir);
	MathLib::Vector3 shoulderDirProjToUpVec = axis[1] * shoulderDirProjToUpVal;

	axis[0] = m_shoulderDir - shoulderDirProjToUpVec;
	axis[0].normalize();

	axis[2] = axis[0].cross(axis[1]);

	// compute 8 corners
	double a_max[3], a_min[3];
	for (int i = 0; i < 3; i++)
	{
		a_max[i] = std::numeric_limits<int>::min();
		a_min[i] = std::numeric_limits<int>::max();
	}

	std::vector<MathLib::Vector3> jointsToShoulderCenterVec(20);
	MathLib::Vector3 shoulderCenter = this->getJoint(SHOULDER_CENTER);

	for (int i = 0; i < 20; i++)
	{
		jointsToShoulderCenterVec[i] = this->getJoint(i) - shoulderCenter;
	}

	// find max and min value along the obb axis
	for (int id = 0; id < 20; id++)
	{
		for (int i = 0; i < 3; i++)
		{
			double jointVecProjToAxisVal;

			jointVecProjToAxisVal = jointsToShoulderCenterVec[id].dot(axis[i]);
			if (jointVecProjToAxisVal > a_max[i])
			{
				a_max[i] = jointVecProjToAxisVal;
			}

			if (jointVecProjToAxisVal < a_min[i])
			{
				a_min[i] = jointVecProjToAxisVal;
			}
		}
	}

	// expand the box a little bit
	a_min[0] -= JointRadius;
	a_min[1] -= JointRadius;
	a_min[2] -= JointRadius;

	a_max[0] += JointRadius;
	a_max[1] += HeadRadius;
	a_max[2] += JointRadius;

	vertices[0] = shoulderCenter + axis[0] * a_min[0] + axis[1] * a_min[1] + axis[2] * a_min[2];
	vertices[1] = shoulderCenter + axis[0] * a_max[0] + axis[1] * a_min[1] + axis[2] * a_min[2];
	vertices[2] = shoulderCenter + axis[0] * a_max[0] + axis[1] * a_min[1] + axis[2] * a_max[2];
	vertices[3] = shoulderCenter + axis[0] * a_min[0] + axis[1] * a_min[1] + axis[2] * a_max[2];

	vertices[4] = shoulderCenter + axis[0] * a_min[0] + axis[1] * a_max[1] + axis[2] * a_min[2];
	vertices[5] = shoulderCenter + axis[0] * a_max[0] + axis[1] * a_max[1] + axis[2] * a_min[2];
	vertices[6] = shoulderCenter + axis[0] * a_max[0] + axis[1] * a_max[1] + axis[2] * a_max[2];
	vertices[7] = shoulderCenter + axis[0] * a_min[0] + axis[1] * a_max[1] + axis[2] * a_max[2];

	for (int i = 0; i < 8; i++)
	{
		center += vertices[i];
	}
	center = center*(1.0 / 8);

	for (int i = 0; i < 3; i++)
	{
		obbSize[i] = std::abs(a_min[i]) + std::abs(a_max[i]);
	}


	m_OBB = COBB(center, axis, obbSize);
}

void Skeleton::drawOBB()
{
	m_OBB.DrawBox(false, m_isPicked, false, false, false);
}

bool ObjPotentialPlacement::randomSampleOnAccessPlanes(ObjLocation &sampleLoc)
{
	bool isPosValid = false;

	if (accessPlanesToPlace.size() > 0)  // start phase skeleton doesn't compute the access planes 
	{
		int planeID = std::rand() % accessPlanesToPlace.size();

		double rand_x = (double)std::rand() / (double)RAND_MAX;
		double rand_y = (double)std::rand() / (double)RAND_MAX;

		double length = accessPlanesToPlace[planeID].corners[1] - accessPlanesToPlace[planeID].corners[0];
		double width = accessPlanesToPlace[planeID].corners[3] - accessPlanesToPlace[planeID].corners[2];
		int modelID = accessPlanesToPlace[planeID].modelID;

		sampleLoc.pos[0] = accessPlanesToPlace[planeID].corners[0] + rand_x*length;
		sampleLoc.pos[1] = accessPlanesToPlace[planeID].corners[2] + rand_y*width;
		sampleLoc.pos[2] = accessPlanesToPlace[planeID].zVal;
		sampleLoc.modelID = modelID;
		sampleLoc.suppPlaneID = accessPlanesToPlace[planeID].suppPlaneID;

		CModel *m = m_scene->getModel(modelID);
		Eigen::Matrix4d modelTransMat = m->getInitTransMat();
		Eigen::Vector4d transSamplePos = modelTransMat.inverse()*Eigen::Vector4d(sampleLoc.pos[0], sampleLoc.pos[1], sampleLoc.pos[2], 0);

		MathLib::Vector3 startPos = MathLib::Vector3(transSamplePos[0], transSamplePos[1], transSamplePos[2]);
		MathLib::Vector3 rayDir = MathLib::Vector3(0, 0, -1);

		double depthVal = 1e6;
		MathLib::Vector3 faceNormal(0, 0, 1);
		int faceID = m->PickByRay(startPos, rayDir, depthVal, faceNormal);

		if (faceID != -1)
		{
			//if (MathLib::Acos(faceNormal.dot(m_scene->getUprightVec())) < AngleThreshold)
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
		if (sampledLocations.size() == SampleLocationNum) return;

		ObjLocation sampleLoc;

		if (randomSampleOnAccessPlanes(sampleLoc))
		{
			sampledLocations.push_back(sampleLoc);
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
		glVertex3f(accessPlanesToPlace[planeID].corners[0], accessPlanesToPlace[planeID].corners[2], accessPlanesToPlace[planeID].zVal);
		glVertex3f(accessPlanesToPlace[planeID].corners[1], accessPlanesToPlace[planeID].corners[2], accessPlanesToPlace[planeID].zVal);
		glVertex3f(accessPlanesToPlace[planeID].corners[1], accessPlanesToPlace[planeID].corners[3], accessPlanesToPlace[planeID].zVal);
		glVertex3f(accessPlanesToPlace[planeID].corners[0], accessPlanesToPlace[planeID].corners[3], accessPlanesToPlace[planeID].zVal);
		glEnd();
	}

	glEnable(GL_LIGHTING);
}

void ObjPotentialPlacement::drawSampledLocations(QColor c)
{
	glDisable(GL_LIGHTING);
	glColorQt(c);

	for (int i = 0; i < sampledLocations.size(); i++)
	{
		float r = 2*PointRadius3D;
		renderSphere(sampledLocations[i].pos[0], sampledLocations[i].pos[1], sampledLocations[i].pos[2], r);
	}

	glEnable(GL_LIGHTING);
}


