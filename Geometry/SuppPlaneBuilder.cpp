#include "SuppPlaneBuilder.h"
#include "CModel.h"
#include "SimplePointCloud.h"


SuppPlaneBuilder::SuppPlaneBuilder()
{
}

SuppPlaneBuilder::SuppPlaneBuilder(CModel *m)
{
	m_model = m;
}


SuppPlaneBuilder::~SuppPlaneBuilder()
{

}

void SuppPlaneBuilder::build(int method)
{
	m_method = method;


}

void SuppPlaneBuilder::computeSuppZlevel()
{
	//std::vector<double> allZvals;
	//for (int i = 0; i < m_SuppChildrenPos.size(); i++)
	//{
	//	allZvals.push_back(m_SuppChildrenPos[i].z);
	//}

	//std::sort(allZvals.begin(), allZvals.end());

	//double zRange = allZvals[allZvals.size() - 1] - allZvals[0];
	//double zGap = 0.1*zRange + 0.1;

	//m_zLevel.push_back(allZvals[0]);

	//for (int i = 1; i < allZvals.size(); i++)
	//{
	//	double currZ = m_zLevel[m_zLevel.size() - 1];
	//	if (allZvals[i] - currZ > zGap)
	//	{
	//		m_zLevel.push_back(allZvals[i]);
	//	}
	//	else
	//		continue;
	//}
}

void SuppPlaneBuilder::collectSuppPtsSet()
{
	MathLib::Vector3 up(0, 0, 1);
	SimplePointCloud& pts = m_model->getPointCloud();
	std::vector<Surface_mesh::Point> SupportPointSoup;

	if (pts.isEmpty())
	{
		m_model->extractToPointCloud();
		pts = m_model->getPointCloud();
	}

	Surface_mesh::Vertex_property<Surface_mesh::Vector3> vnormals = m_model->meshData()->vertex_property<Surface_mesh::Vector3>("v:normal");
	Surface_mesh::Vertex_iterator vit, vend = m_model->meshData()->vertices_end();

	int id = 0;
	for (vit = m_model->meshData()->vertices_begin(); vit != vend; vit++)
	{
		MathLib::Vector3 vn = MathLib::Vector3(vnormals[vit][0], vnormals[vit][1], vnormals[vit][2]);
		if (MathLib::Acos(vn.dot(up)) < 10)
		{
			SupportPointSoup.push_back(pts[id]);
		}

		id++;
	}
}

void SuppPlaneBuilder::seperateSuppZlevel()
{

}
