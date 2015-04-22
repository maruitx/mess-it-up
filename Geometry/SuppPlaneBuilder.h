#pragma once
#include "../Math/mathlib.h"
#include "SurfaceMeshHelper.h"
#include "../Utilities/utility.h"

class CModel;
class SuppPlane;

class SuppPlaneBuilder
{
public:
	enum BuildMethod{ AABB_PLANE, OBB_PLANE, CONVEX_HULL_PLANE };
	SuppPlaneBuilder();
	~SuppPlaneBuilder();

	SuppPlaneBuilder(CModel *m);
	void build(int method);

	void collectSuppPtsSet();
	void seperateSuppSoupByZlevel(const std::vector<Surface_mesh::Point> &pts);

	void draw();

	std::vector<SuppPlane*> getAllSuppPlanes() { return m_suppPlanes; };
	SuppPlane* getLargestAreaSuppPlane();

private:
	CModel *m_model;
	int m_method;

	// each inner vector corresponds to a support plane
	std::vector<std::vector<MathLib::Vector3>> m_SuppPtsSet;
	std::vector<double> m_zLevelVals;
	std::vector<SuppPlane*> m_suppPlanes;

	MathLib::Vector3 m_upRightVec;
};

