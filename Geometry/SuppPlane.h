#pragma once
#include "../Math/mathlib.h"
#include <qgl.h>

class CModel;

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

	void Build(std::vector<MathLib::Vector3> &PointSet);
	void Draw();

	MathLib::Vector3* GetCorners() {return m_corners;};
	ModelInfo& GetSuppModels() {return m_SuppModels;};
	int GetSuppModelTypeNum() {m_SuppModels.size();};
	int GetSuppModelNum(); // same instances will appear, the number is different from m_SuppModels.size()
	MathLib::Vector3 GetCenter() {return center;};
	MathLib::Vector3 GetConrner(int i) {return m_corners[i];};
	double GetWidth() { return width;};
	double GetLength() {return length;};
	double GetZ() {return m_corners[0].z;};

private:
	double length;
	double width;
	MathLib::Vector3 center;
	MathLib::Vector3 normal;

	MathLib::Vector3 m_corners[4];
	std::vector<MathLib::Vector3> m_vertices;
	
	ModelInfo m_SuppModels;
	std::vector<MathLib::Vector3> m_SuppPointSet;

};

