#include "Skeleton.h"
#include "qgl.h"
#include "qglviewer/quaternion.h"
#include "../Utilities/CustomDrawObjects.h"


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

void Skeleton::draw()
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
		drawBone(i);
	}

	for (int i = 0; i < m_joints.size(); i++)
	{
		drawJoint(i);
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

std::vector<MathLib::Vector3> Skeleton::getTransformedJoints(double newHipX, double newHipY, double thetaZ, MathLib::Vector3 upright)
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

		trans_joints[i] = MathLib::Vector3(joint[0] + newHipX, joint[1] + newHipY, joint[2]);
	}

	return trans_joints;
}

MathLib::Vector3 Skeleton::getJoint(int id)
{
	return MathLib::Vector3(m_joints[id][0], m_joints[id][1], m_joints[id][2]);
}



