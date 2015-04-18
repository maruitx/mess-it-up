#pragma once
#include "../Math/mathlib.h"
#include <qgl.h>

class CModel;

const double EdgeLength_Threshold = 0.1;
const double Area_Threshold = 0.02;

class SuppPlane
{
public:

	typedef std::map<std::string, std::vector<MathLib::Vector2>> ModelInfo;

	SuppPlane(void);
	~SuppPlane(void);

	SuppPlane(std::vector<MathLib::Vector3> &PointSet);
	SuppPlane(std::vector<MathLib::Vector3> &PointSet, std::vector<CModel*> SuppChildren, std::vector<MathLib::Vector3> &SuppChildrenPos);
	SuppPlane(MathLib::Vector3 newCorners[4], ModelInfo &suppModels);
	SuppPlane(MathLib::Vector3 newCorners[4]);

	SuppPlane(std::vector<MathLib::Vector3> &PointSet, const std::vector<MathLib::Vector3> obbAxis);

	void setModel(CModel *m) { m_model = m; };

	void Build(std::vector<MathLib::Vector3> &PointSet);
	void BuildAAPlane(std::vector<MathLib::Vector3> &PointSet);
	void Draw(QColor c);

	void computeParas();

	MathLib::Vector3* GetCorners() {return m_corners;};
	ModelInfo& GetSuppModels() {return m_SuppModels;};
	int GetSuppModelTypeNum() {m_SuppModels.size();};
	int GetSuppModelNum(); // same instances will appear, the number is different from m_SuppModels.size()
	MathLib::Vector3 GetCenter() {return center;};
	MathLib::Vector3 GetCorner(int i) {return m_corners[i];};
	double GetWidth() { return width;};
	double GetLength() {return length;};
	double GetZ() {return m_corners[0].z;};	
	double GetArea() { return width*length; };

	std::vector<MathLib::Vector3> GetAxis() { return axis; };

	bool isTooSmall();

private:
	double length;
	double width;
	MathLib::Vector3 center;
	MathLib::Vector3 normal;

	std::vector<MathLib::Vector3> axis;

	MathLib::Vector3 m_corners[4];
	std::vector<MathLib::Vector3> m_vertices;
	
	ModelInfo m_SuppModels;
	std::vector<MathLib::Vector3> m_SuppPointSet;

	CModel *m_model;

};

