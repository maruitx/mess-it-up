#pragma once

#include "AABB.h"
#include "OBB.h"
#include "SurfaceMeshHelper.h"
#include "SimplePointCloud.h"
#include "SuppPlane.h"
#include "SuppPlaneBuilder.h"
#include "Voxeler.h"

#include "qgl.h"
#include "../Utilities/utility.h"
#include "../Math/mathlib.h"

class SimplePointCloud;
class Skeleton;

class CModel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CModel();
	~CModel();

	void loadModel(QString filename, double metric);
	
	void setLabel(QString &l);
	QString label() { return m_label; };

	void setFilePath(const QString &filepath) { m_filePath = filepath; };

	void setID(int i) { m_id = i; };
	int getID() { return m_id; };

	Eigen::AlignedBox3d bbox() { return m_mesh->bbox(); };
	SurfaceMesh::SurfaceMeshModel* meshData() { return m_mesh; };
	SurfaceMesh::Vector3 getTransformedOBBCenter();
	QVector<SurfaceMesh::Vector3> getTransformedOBBVertices();

	void readScanObj(SurfaceMesh::SurfaceMeshModel *mesh, std::string filename);

	void extractToPointCloud();
	SimplePointCloud& getPointCloud();

	// rendering options
	void buildDisplayList();
	void draw();
	void drawAABBox();
	void drawOBB();

	void setTransMat(const Eigen::Matrix4d &m);

	bool IsContact(CModel *pOther, bool onlyOBB, double dDistT, MathLib::Vector3 &Dir);
	bool IsSupport(CModel *pOther, bool onlyOBB, double dDistT, const MathLib::Vector3 &Upright);
	bool IsContain(CModel *pOther);
	bool IsSimilar(CModel *pOther, const MathLib::Vector3 &Upright);

	int ReadOBB(const QString &sPathName);
	int WriteOBB(const QString &sPathName);

	void setAABB();
	void computeOBB(int fixAxis = -1);
	void updateOBBTransMat() { m_GOBB.transMat = m_transMat; };
	double getOBBBottomHeight(MathLib::Vector3 &uprightVec);

	std::vector<double> getAABBXYRange();

	// Support plane
	void buildSuppPlane();

	// skeleton
	void testInteractSkeleton(Skeleton *sk);
	bool isInteractSkeleton() { return m_isInteractSkeleton; };

	// voxel
	void voxelize();
	void drawVoxel();
	void drawVoxelOctree();
	bool isVoxelized() { return m_isVoxelized; };
	bool isPointInside(SurfaceMesh::Vector3 &point);
	bool isSegmentIntersect(SurfaceMesh::Vector3 &startPt, SurfaceMesh::Vector3 &endPt);

	bool loadVoxelData(const QString &filename, std::vector<VoxelerLibrary::Voxel> &voxels);

	// interaction
	void setPicked(bool state) { m_isPicked = state; };
	void updatePickedState() { m_isPicked = !m_isPicked; };
	bool isPicked() { return m_isPicked; };

	// arrangement
	bool isFixed() { return m_isFixed; };

	int suppParentID;
	std::vector<int> suppChindrenList;
	int supportLevel;

public:
	CAABB m_AABB;
	COBB m_GOBB;

private:
	QString m_label;
	QString m_filePath;
	int m_id;
	double m_metric;

	Eigen::Matrix4d m_transMat;
	bool m_isTransformed;

	SurfaceMesh::SurfaceMeshModel *m_mesh;
	SimplePointCloud m_pts;
	QVector<SuppPlane*> m_suppPlanes;
	SuppPlaneBuilder *m_suppPlaneBuilder;

	VoxelerLibrary::Voxeler *m_voxeler;
	bool m_isVoxelized;
	int m_voxelNum;
	double m_voxelSize;

	bool m_isPicked;
	bool m_isInteractSkeleton;

	bool m_isFixed;

	GLuint m_displayListID;
};
