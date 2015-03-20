#include "SimplePointCloud.h"
#include <QFile>


SimplePointCloud::SimplePointCloud()
{
}

SimplePointCloud::SimplePointCloud(std::vector<SurfaceMesh::Point> &pts) :
m_data(pts)
{
	m_numPts = m_data.size();
}


SimplePointCloud::~SimplePointCloud()
{
}

void SimplePointCloud::saveAsPly(QString filename)
{
	QFile file(filename);

	if (file.open(QIODevice::ReadWrite))
	{
		QTextStream ofs(&file);
		
		ofs << "ply\n";
		ofs << "format ascii 1.0\n";
		ofs << "element vertex " << m_numPts<<"\n";
		ofs << "property float x\n";
		ofs << "property float y\n";
		ofs << "property float z\n";
		ofs << "end_header\n";

		for (int i = 0; i < m_numPts; i++)
		{
			ofs << m_data[i][0] << " " << m_data[i][1] << " " << m_data[i][2] << "\n";
		}
			
		file.close();
	}
}

void SimplePointCloud::addPoint(SurfaceMesh::Point pt)
{
	m_data.push_back(pt);
}

void SimplePointCloud::addPoints(std::vector<SurfaceMesh::Point> pts)
{
	m_data.insert(m_data.end(), pts.begin(), pts.end());
}

void SimplePointCloud::mergePointCloud(SimplePointCloud &pts)
{
	m_data.insert(m_data.end(), pts.begin(), pts.end());
}

void SimplePointCloud::transform(Eigen::Matrix4d &m)
{
	for (int i = 0; i < m_numPts; i++)
	{
		Eigen::Vector4d newPt = m*Eigen::Vector4d(m_data[i][0], m_data[i][1], m_data[i][2],1);
		m_data[i] = Eigen::Vector3d(newPt[0], newPt[1], newPt[2]);
	}
}
