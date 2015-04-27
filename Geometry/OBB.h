/***************************************************************************
OBB_estimator.h  - Compute Oriented Bounding Box for mesh
----------------------------------------------------------------------------
Note: 
----------------------------------------------------------------------------
begin                : jun 2007
copyright            : (C) 2007 by Kevin (Kai) Xu - NUDT603
email                : kevin.kai.xu@gmail.com
***************************************************************************/
#pragma once
#include "../Utilities/utility.h"
#include "../Math/mathlib.h"
#include "../Math/Eigen3x3.h"
#include "qglviewer/qglviewer.h"
#include "ShapeLib.h"

#include "Eigen/Dense"

#include <algorithm>
#include <vector>
#include <functional>

//static int 

#define BB_G_FACE		0x01	// 
#define BB_G_3DEDGE		0x02	// 

#define BB_SIMI_DIST	0x01	// use distance in computing OBB similarity
#define BB_SIMI_SHAPE	0x02	// use shape characteristics in computing OBB similarity
#define BB_SIMI_ORIEN	0x04	// use orientation in computing OBB similarity
#define BB_SIMI_ALL		0x07	// use all in computing OBB similarity

using namespace MathLib;

class COBB
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<MathLib::Vector3> PointSet;

	COBB(void);
	COBB(const COBB& obb);
	COBB(const MathLib::Vector3 &c, const std::vector<MathLib::Vector3> &a, const MathLib::Vector3 &s);
	virtual ~COBB();

	void SetAAData(const MathLib::Vector3 &c, const MathLib::Vector3 &s);
	void SetData(const MathLib::Vector3 &c, const std::vector<MathLib::Vector3> &a, const MathLib::Vector3 &s);
	void SetData(const MathLib::Vector3 &c, const MathLib::Vector3 &e1, const MathLib::Vector3 &e2, const MathLib::Vector3 &e3);
	void SetData(double *sides, double *matrix);
	void updateDataAS(void);	// update with axes and center as known
	void updateDataP(void);		// update with positions as known
	void TransScl(double dx, double dy, double dz, double s);
	void Blend(const COBB &obb);
	void analyzeAxis(void);
	void Unitize(const MathLib::Vector3 &c, double s);
	int GetClosestAxis(const MathLib::Vector3 &v) const;
	void CalcAxisMatchMap(const COBB &bb, std::vector<int> &m) const;
	void MatchAxis(const COBB &bb);
	void CalcAxisMatchBB(const COBB &bbr, COBB &bbo, std::vector<int> &m) const;
//	int CalcDissimilarity(const COBB &bb, double &d, double dDMax=0.0, char flags=BB_SIMI_ALL) const;
	int Dissimilarity(const COBB &bb, double dDMax, double &d) const;
	double HausdorffDist(const COBB &bb) const;
	double HausdorffDist_Proj(const COBB &bb, const MathLib::Vector3 &dir) const;
	void Anisotropy(MathLib::Vector3 &c) const;
	void ClosestPoint(const MathLib::Vector3 &p, MathLib::Vector3 &cp) const;
	double SqDistance_Approx(const COBB &bb) const;
	void GetLongestAxis(MathLib::Vector3 &a) const;
	void GetShortestAxis(MathLib::Vector3 &a) const;
	void SetHullVert(const PointSet &ps);
	void WriteData(FILE *fp);
	void WriteData(std::ofstream &ofs);
	void WriteData(FILE *fp, MathLib::Matrix4d &TM);
	void ReadData(std::ifstream &ifs);
	void ReadData(std::ifstream &ifs, const MathLib::Vector3 &uc, double us);
	double Vol(void) const;
	const MathLib::Vector3& C(void) const;
	const PointSet& HV(void) const;
	const PointSet* HVPointer(void) const;
	const MathLib::Vector3& A(int i) const;
	const std::vector<MathLib::Vector3>& A() const;
	const double& S(int i) const;
	const MathLib::Vector3& S(void) const;
	const double& HS(int i) const;
	const MathLib::Vector3& HS(void) const;
	const MathLib::Vector3& V(int i) const;
	MathLib::Vector3& V(int i);
	const std::vector<MathLib::Vector3>& V(void) const;
	std::vector<MathLib::Vector3>& V(void);
	MathLib::Vector3 GetFaceCent(int f) const;
	void Face(int ai, int d, std::vector<int> &fv);
	COBB& operator=(const COBB& other);
	int operator==(const COBB& other);
	int operator!=(const COBB& other);
	void GetApproxBoxes(std::vector<COBB> &BL) const;

	void DrawBox(bool b3DEdge, bool bFace, bool bGraph, bool bShowStat, bool bHighL) const;
	void DrawSamples(void);
	int DoSampling(double r);

	void Transform(const MathLib::Matrix4d &m);

	bool IsInside(const MathLib::Vector3 &p) const;
	bool IsIntersAACube(const MathLib::Vector3 &c, double s) const;
	bool IsContain(const COBB &obb, double tp) const;
	bool IsContain(const COBB &obb) const;
	bool IsContact(const COBB &obb, double ta, double td, MathLib::Vector3 &dir) const;
	bool IsSupport(const COBB &obb, double ta, double td, const MathLib::Vector3 &upright) const;
	double ConnStrength_CD(const COBB &obb) const;
	double ConnStrength_HD(const COBB &obb) const;
	bool IsOnTop(const COBB &obb, const MathLib::Vector3 &upright, double &pd) const;
	bool IsAbove(const COBB &obb, const MathLib::Vector3 &upright) const;
	bool IsTwoSide(const COBB &obb, const MathLib::Vector3 &p, const MathLib::Vector3 &n) const;
	double ConnStrength_OBB(const COBB &obb, int iFixA=-1) const;
	double ConnStrength_Proj(const COBB &obb, const MathLib::Vector3 &upright) const;
	double BottomHeightDiff(const COBB &obb, const MathLib::Vector3 &upright) const;
	double GetBottomHeight(const MathLib::Vector3 &upright) const;
	double TopHeightDiff(const COBB &obb, const MathLib::Vector3 &upright) const;

	bool PickByRay(const MathLib::Vector3 &sp, const MathLib::Vector3 &dir, double &dPD) const;
	bool PickByRay_Ortho(const MathLib::Vector3 &sp, const MathLib::Vector3 &dir, double &dPD) const;
	bool PickByRayWithTrans(const MathLib::Vector3 &sp, const MathLib::Vector3 &dir, double &dPD) const;

	bool IsInteract(const std::vector<MathLib::Vector3> &pts, double dist, std::vector<int> &states);

	Eigen::Matrix4d recordTransMat;
	bool IsInsideWithTrans(const MathLib::Vector3 &p) const;
	void ClosestPointWithTrans(const MathLib::Vector3 &p, MathLib::Vector3 &cp) const;

	MathLib::Vector3 GetTransformedCenter();
	std::vector<MathLib::Vector3> GetTransformedVertices();
	std::vector<MathLib::Vector3> GetTransformedVertices(const Eigen::Matrix4d &transMat);

public:
	MathLib::Vector3					cent;		// center
	std::vector<MathLib::Vector3>	axis;		// 3 principal axes
	MathLib::Vector3					size;		// 3 sizes
	MathLib::Vector3					hsize;		// 3 half sizes
	std::vector<MathLib::Vector3>	vp;			// 8 vertices (same order as psBoxV; see Utility.h)
	PointSet				chv;		// convex hull vertices
	PointSet				sp;			// sample
	double						vol;		// volume
	double						dl;			// diagonal length
	double						er;			// edge radius
	double						sr;			// sampling rate
	int						ca;			// characteristic axis
	double						cas;		// characteristic axis strength
	double						as, ap, al;
	
	static GLuint			s_edl;		// 3d edge (cylinder) display list (static member shared by all instances)
};
