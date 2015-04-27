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
struct ObjLocation;

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

	void setSceneUpRightVec(const MathLib::Vector3 &upright) { m_sceneUpRightVec = upright; };

	Eigen::AlignedBox3d bbox() { return m_mesh->bbox(); };
	SurfaceMesh::SurfaceMeshModel* meshData() { return m_mesh; };
	SurfaceMesh::Vector3 getOBBCenter();
	SurfaceMesh::Vector3 getTransformedOBBCenter();
	QVector<SurfaceMesh::Vector3> getTransformedOBBVertices();
	std::vector<MathLib::Vector3> getOBBAxis() { return m_GOBB.axis; };

	void readObjFile(SurfaceMesh::SurfaceMeshModel *mesh, std::string filename);

	void extractToPointCloud();
	SimplePointCloud& getPointCloud();

	// rendering options
	void buildDisplayList();
	void draw();
	void drawAABBox();
	void drawOBB();

	Eigen::Matrix4d getInitTransMat() { return m_recordTransMat; };
	void setTempDisplayTransMat(const Eigen::Matrix4d &m);
	void setInitTransMat(const Eigen::Matrix4d &m);

	bool IsContact(CModel *pOther, bool onlyOBB, double dDistT, MathLib::Vector3 &Dir);
	bool IsSupport(CModel *pOther, bool onlyOBB, double dDistT, const MathLib::Vector3 &Upright);
	bool IsContain(CModel *pOther);
	bool IsSimilar(CModel *pOther, const MathLib::Vector3 &Upright);

	int ReadOBB(const QString &sPathName);
	int WriteOBB(const QString &sPathName);

	void setAABB();
	void computeOBB(int fixAxis = -1);
	void updateOBBTransMat() { m_GOBB.recordTransMat = m_recordTransMat; };
	double getOBBBottomHeight(MathLib::Vector3 &uprightVec);
	std::vector<double> getOBBSize();

	std::vector<double> getAABBXYRange();

	// Support plane
	void buildSuppPlane();
	void drawSuppPlane();
	std::vector<SuppPlane*> getAllSuppPlanes() { return m_suppPlaneBuilder->getAllSuppPlanes(); };
	SuppPlane* getLargestAreaSuppPlane() { return m_suppPlaneBuilder->getLargestAreaSuppPlane(); };
	SuppPlane* getSuppPlane(int id) { return m_suppPlaneBuilder->getSuppPlane(id); };

	bool isVertexUpRight(Surface_mesh::Vertex v, double angleTh);

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
	double getClosestDistToVoxel(SurfaceMesh::Vector3 &pt);

	// interaction
	void setPicked(bool state) { m_isPicked = state; };
	void updatePickedState() { m_isPicked = !m_isPicked; };
	bool isPicked() { return m_isPicked; };
	int PickByRay(MathLib::Vector3 startPoint, MathLib::Vector3 rayDir, double &depth, MathLib::Vector3 &faceNormal);

	// arrangement
	bool isFixed() { return m_isFixed; };
	bool testStabilityForNewLocation(const MathLib::Vector3 &newLocation, SuppPlane *suppPlane); // whether collision will happen if move to new location, reject if yes
	void updateCurrentLocation();
	MathLib::Vector3 getCurrentLocation() { return m_currentLocation; };

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
	MathLib::Vector3 m_sceneUpRightVec;

	MathLib::Vector3 m_currentLocation;
	MathLib::Vector3 m_newLocation;

	Eigen::Matrix4d m_tempDisplayMat;
	Eigen::Matrix4d m_recordTransMat;
	bool m_isTransformed;

	Eigen::Matrix4d m_tempTransMat;

	SurfaceMesh::SurfaceMeshModel *m_mesh;
	SimplePointCloud m_pts;
	SuppPlaneBuilder *m_suppPlaneBuilder;
	bool m_hasSuppPlane;

	VoxelerLibrary::Voxeler *m_voxeler;
	bool m_isVoxelized;
	int m_voxelNum;
	double m_voxelSize;

	bool m_isPicked;
	bool m_isInteractSkeleton;

	bool m_isFixed;

	GLuint m_displayListID;
};
