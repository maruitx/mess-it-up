#pragma once

class CModel;

class SuppPlaneBuilder
{
public:
	enum{AABB_PLANE, OBB_PLANE, CONVEX_HULL_PLANE} BuildMethod;
	SuppPlaneBuilder();
	~SuppPlaneBuilder();

	SuppPlaneBuilder(CModel *m);
	void build(int method);
	void collectSuppPtsSet();
	void computeSuppZlevel();
	void seperateSuppZlevel();

private:
	CModel *m_model;
	int m_method;

};

