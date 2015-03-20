#pragma once
#include "SurfaceMeshHelper.h"

class SimplePointCloud
{
public:
	SimplePointCloud();
	~SimplePointCloud();

	SimplePointCloud(std::vector<SurfaceMesh::Point> &pts);

	void saveAsPly(QString filename);

	bool isEmpty() { return m_data.empty(); };

	void addPoint(SurfaceMesh::Point pt);
	void addPoints(std::vector<SurfaceMesh::Point> pts);

	void mergePointCloud(SimplePointCloud &pts);

	void transform(Eigen::Matrix4d &m);


	void setID(const int i) { m_id = i; };
	int getID() { return m_id; };

	void setLabel(const std::string &s) { m_label = s; };
	std::string getLabel() { return m_label; };

	int size() { return m_data.size(); };
	void resize(int n) { m_data.resize(n); m_numPts = m_data.size(); };
	Surface_mesh::Point& operator[](int i) { return m_data[i]; };
	std::vector<SurfaceMesh::Point>::iterator begin() { return m_data.begin(); };
	std::vector<SurfaceMesh::Point>::iterator end() { return m_data.end(); };


private:
	std::vector<SurfaceMesh::Point> m_data;
	int m_numPts;

	std::string m_label;
	int m_id;
};

