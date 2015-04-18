#include "SuppPlane.h"
#include "CModel.h"
#include "QuickMeshDraw.h"


SuppPlane::SuppPlane(void)
{

}


SuppPlane::~SuppPlane(void)
{

}

SuppPlane::SuppPlane(std::vector<MathLib::Vector3> &PointSet)
{
	Build(PointSet);
}

SuppPlane::SuppPlane(std::vector<MathLib::Vector3> &PointSet, std::vector<CModel*> SuppChildren, std::vector<MathLib::Vector3> &SuppChildrenPos)
{
	Build(PointSet);

	for (int i=0; i< SuppChildren.size(); i++)
	{
		std::string currModelName;
		MathLib::Vector3 currPos = SuppChildrenPos[i];

		if (abs(currPos.z - center.z) < 2)
		{
			double para_x, para_y;
			para_x = (currPos.x - m_corners[0].x)/length;
			para_y = (currPos.y - m_corners[0].y)/width;

			std::vector<Vector2> currPosVec; 
			if (m_SuppModels.find(currModelName) != m_SuppModels.end())
			{
				currPosVec = m_SuppModels[currModelName];
			}

			currPosVec.push_back(Vector2(para_x, para_y));
			m_SuppModels[currModelName] = currPosVec;
		}
	}
}

SuppPlane::SuppPlane(MathLib::Vector3 newCorners[4], ModelInfo &suppModels)
{
	m_corners[0] = newCorners[0];
	m_corners[1] = newCorners[1];
	m_corners[2] = newCorners[2];
	m_corners[3] = newCorners[3];

	m_SuppModels = suppModels;
	center = (m_corners[0]+m_corners[1]+m_corners[2]+m_corners[3])*0.25;
	length = (m_corners[1] - m_corners[0]).magnitude();
	width = (m_corners[2] - m_corners[1]).magnitude();
}

SuppPlane::SuppPlane(MathLib::Vector3 newCorners[4])
{
	m_corners[0] = newCorners[0];
	m_corners[1] = newCorners[1];
	m_corners[2] = newCorners[2];
	m_corners[3] = newCorners[3];

	computeParas();
}

SuppPlane::SuppPlane(std::vector<MathLib::Vector3> &PointSet, const std::vector<MathLib::Vector3> obbAxis)
{
	axis.resize(3);
	axis[0] = obbAxis[0];
	axis[1] = obbAxis[1];

	axis[0].normalize();
	axis[1].normalize();

	MathLib::Vector3 setCenter(0, 0, 0);

	for (int i = 0; i < PointSet.size(); i++)
	{
		setCenter += PointSet[i];
	}

	setCenter /= PointSet.size();

	std::vector<MathLib::Vector3> centerToPointVec(PointSet.size());

	for (int i = 0; i < PointSet.size(); i++)
	{
		centerToPointVec[i] = PointSet[i] - setCenter;
	}

	double max_a0_x = -1e6;
	double min_a0_x = 1e6;
	double max_a1_y = -1e6;
	double min_a1_y = 1e6;  

	for (int i = 0; i < PointSet.size(); i++)
	{
		double pt_a0_comp = centerToPointVec[i].dot(axis[0]);

		if (pt_a0_comp > max_a0_x)
		{
			max_a0_x = pt_a0_comp;
		}

		if (pt_a0_comp < min_a0_x)
		{
			min_a0_x = pt_a0_comp;
		}

		double pt_a1_comp = centerToPointVec[i].dot(axis[1]);
		
		if (pt_a1_comp > max_a1_y)
		{
			max_a1_y = pt_a1_comp;
		}

		if (pt_a1_comp < min_a1_y)
		{
			min_a1_y = pt_a1_comp;
		}
	}

	// min_x, max_y is negative
	m_corners[0] = setCenter + axis[0] * min_a0_x + axis[1] * min_a1_y;
	m_corners[1] = setCenter + axis[0] * max_a0_x + axis[1] * min_a1_y;
	m_corners[2] = setCenter + axis[0] * max_a0_x + axis[1] * max_a1_y;
	m_corners[3] = setCenter + axis[0] * min_a0_x + axis[1] * max_a1_y;

	computeParas();
}

void SuppPlane::Build(std::vector<MathLib::Vector3> &PointSet)
{
	m_SuppPointSet = PointSet;

	// axis aligned plane
	double min_x = 1e6;
	double min_y = 1e6;
	double max_x = -1e6;
	double max_y = -1e6;
	double center_z = 0;

	for (int i=0;i<PointSet.size();i++)
	{
		MathLib::Vector3 currPoint = PointSet[i];
		if (currPoint.x > max_x)
		{
			max_x = currPoint.x;
		}
		if (currPoint.x < min_x)
		{
			min_x = currPoint.x;
		}

		if (currPoint.y > max_y)
		{
			max_y = currPoint.y;
		}
		if (currPoint.y < min_y)
		{
			min_y = currPoint.y;
		}

		center_z += PointSet[i].z;
	}

	center_z = center_z/PointSet.size();
	m_corners[0] = MathLib::Vector3(min_x, min_y, center_z);
	m_corners[1] = MathLib::Vector3(max_x, min_y, center_z);
	m_corners[2] = MathLib::Vector3(max_x, max_y, center_z);
	m_corners[3] = MathLib::Vector3(min_x, max_y, center_z);

	computeParas();
}


void SuppPlane::Draw(QColor c)
{
	glDisable(GL_LIGHTING);
	glColorQt(c);
	glBegin(GL_QUADS);
	glVertex3f(m_corners[0].x, m_corners[0].y, m_corners[0].z);
	glVertex3f(m_corners[1].x, m_corners[1].y, m_corners[1].z);
	glVertex3f(m_corners[2].x, m_corners[2].y, m_corners[2].z);
	glVertex3f(m_corners[3].x, m_corners[3].y, m_corners[3].z);
	glEnd();
	glEnable(GL_LIGHTING);
}

int SuppPlane::GetSuppModelNum()
{
	int modelNum = 0;
	//for (ModelInfo::const_iterator it=m_SuppModels.begin(); it!=m_SuppModels.end();it++)
	//{
	//	modelNum += it->second.size();
	//}

	return modelNum;
}

void SuppPlane::BuildAAPlane(std::vector<MathLib::Vector3> &PointSet)
{
	m_SuppPointSet = PointSet;

	// axis aligned plane
	double min_x = 1e6;
	double min_y = 1e6;
	double max_x = -1e6;
	double max_y = -1e6;
	double center_z = 0;

	for (int i = 0; i < PointSet.size(); i++)
	{
		MathLib::Vector3 currPoint = PointSet[i];
		if (currPoint.x > max_x)
		{
			max_x = currPoint.x;
		}
		if (currPoint.x < min_x)
		{
			min_x = currPoint.x;
		}

		if (currPoint.y > max_y)
		{
			max_y = currPoint.y;
		}
		if (currPoint.y < min_y)
		{
			min_y = currPoint.y;
		}

		center_z += PointSet[i].z;
	}

	center_z = center_z / PointSet.size();
	m_corners[0] = MathLib::Vector3(min_x, min_y, center_z);
	m_corners[1] = MathLib::Vector3(max_x, min_y, center_z);
	m_corners[2] = MathLib::Vector3(max_x, max_y, center_z);
	m_corners[3] = MathLib::Vector3(min_x, max_y, center_z);

	center = (m_corners[0] + m_corners[1] + m_corners[2] + m_corners[3])*0.25;
	length = max_x - min_x;
	width = max_y - min_y;
}

bool SuppPlane::isTooSmall()
{
	if (width < EdgeLength_Threshold || length < EdgeLength_Threshold || width*length < Area_Threshold)
	{
		return true;
	}

	else
	{
		return false;
	}
}

void SuppPlane::computeParas()
{
	center = (m_corners[0] + m_corners[1] + m_corners[2] + m_corners[3])*0.25;
	length = (m_corners[1] - m_corners[0]).magnitude();
	width = (m_corners[2] - m_corners[1]).magnitude();
}