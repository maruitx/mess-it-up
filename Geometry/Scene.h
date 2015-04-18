#pragma once
#include "../Utilities/utility.h"
#include "../Utilities/CustomDrawObjects.h"
#include "../Math/MatrixType.h"
#include "../Math/mathlib.h"

#include "qglviewer/qglviewer.h"
#include "StarlabDrawArea.h"
#include "SurfaceMeshHelper.h"
#include "SimplePointCloud.h"
#include "CModel.h"
#include "SceneRG.h"
#include "OBBOBBIntersect.h"

class Skeleton;

class CScene
{
public:
	typedef enum { CT_SUPPORT = 0, 
		CT_CONTACT, 
		CT_CONTAIN, 
		CT_PROXIMITY, 
		CT_SYMMETRY, 
		CT_WEAK, 
		CT_SUP_SYM, 
		CT_CONTACT_SYM, 
		CT_PROX_SYM, 
		CT_WEAK_SYM } ConnType;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CScene();
	~CScene();

	void loadScene(const QString filename);
	void setSceneName(const QString &sceneName) { m_sceneFileName = sceneName; };
	QString getSceneName() { return m_sceneFileName; };

	void setSceneDrawArea(Starlab::DrawArea *a) { m_drawArea = a; };
	MathLib::Vector3 getUprightVec() { return m_uprightVec; };

	CModel* getModel(int id) { return m_modelList[id]; };
	QString getModelName(int modelID) { return m_modelList[modelID]->label(); };
	int getModelIdByName(const QString &s) { return m_modelNameIdMap[s]; };
	QVector<QString> getModelNameList();
	int getModelNum() { return m_modelNum; };
	std::vector<double> getFloorXYRange();

	Eigen::AlignedBox3d bbox() { return m_bbox; }

	void extractToPointCloud();
	QVector<SimplePointCloud> getSelectedPointCloud(std::vector<int> ids);

	void setSceneTransMat(const Eigen::Matrix4d &m) { m_sceneTransMat = m; };
	void setModelTransMat(int modelID, const Eigen::Matrix4d &m);
	void draw();
	void buildModelDislayList();

	void setShowModel(bool state) { m_isShowModel = state; };
	void setShowOBB(bool state) { m_isShowOBB = state; };
	void setShowRG(bool state) { m_isShowRG = state; };
	void setShowModelName(bool state) { m_isShowModelName = state; };
	void setShowModelVoxel(bool state) { m_isShowModelVoxel = state; };

	void updateDrawArea() { m_drawArea->updateGL(); };
	
	// obb
	void computeModelOBB();
	int readModelOBB(bool bSilent);
	int writeModelOBB(bool bSilent);

	void buildModelSuppPlane();

	// relation graph
	void buildRelationshipGraph();
	void drawRelationGraph();
	int readRG(bool bSilent);
	int writeRG(bool bSilent);
	CSceneRG& getSceneRG() { return m_RG; };

	void buildSupportHierarchy();
	void setChildrenSupportLevel(CModel *m);
	bool hasSupportHierarchy() { return m_hasSupportHierarchy; };

	// skeleton
	void testInteractSkeleton(Skeleton *sk);

	// voxel
	void voxelizeModels();
	bool isInsideModel(SurfaceMesh::Vector3 &point, int modelID);
	bool isIntersectModel(SurfaceMesh::Vector3 &startPt, SurfaceMesh::Vector3 &endPt, int modelID);
	void setShowModelVoxel(int state) { m_isShowModelVoxel = state; };
	void setShowVoxelOctree(int state) { m_isShowModelVoxelOctree = state; };

	// arrangement
	bool isModelFixed(int modelID) {return m_modelList[modelID]->isFixed(); };
	void setCenterModelID(int id) { m_centerModelID = id; };

	// select interaction
	void pickModelAt(QPoint p);
	void setPickModelMode(bool state) { m_isPickModelMode = state; };
	bool isPickModelModeOn() { return m_isPickModelMode; };
	void unPickAllModels();
	void setModelPicked(int modelID, bool state);
	QVector<int> getSelectedModelIDs();

	// 2d names
	void drawModelName(const QString &s, SurfaceMesh::Vector3 pos);
	void drawModelName();
	void beginDraw2DText();
	void endDraw2DText();
	QImage fontImage;

private:

	int extractContactRel(); //
	int extractSupportRel(); //
	int extractContainRel(void);
	int extractProximityRel(void);
	int extractSymGroup();

	int pruneSupportRel();

	int doSymmetryConnection(ConnType ct, bool bCompSCG = false);
	int connectCommunities(ConnType ct = CT_WEAK); //

	void computeConnStrengthMat(void);
	void computeOnTopList(void);
	void computeModelDistMat(void);

private:
	Starlab::DrawArea *m_drawArea;

	int m_modelNum;
	QVector<CModel *> m_modelList;
	QMap<QString, int> m_modelNameIdMap;

	QVector<SimplePointCloud> m_points;
	Eigen::AlignedBox3d m_bbox;

	const double m_SuppThresh;
	const double m_ContactThresh;
	double m_metricConvert;

	MathLib::Vector3 m_uprightVec;
	Eigen::Matrix4d m_sceneTransMat;

	std::vector<int>				m_onTopList;			// on top list
	CDistMat<double>					m_modelDistMat;			// model center distance matrix
	CDistMat<double>					m_ConnStrenMat;			// connection strength matrix

	std::vector<std::vector<int>>	m_SymGrp;
	std::vector<int>				m_SymMap;

	CSceneRG m_RG;

	// File info
	QString m_sceneFileName;
	QString m_sceneFilePath;

	int m_centerModelID;

	bool m_hasSupportHierarchy;

	bool m_isShowModel;
	bool m_isShowOBB;
	bool m_isShowRG;
	bool m_isShowModelName;
	bool m_isShowModelVoxel;
	bool m_isShowModelVoxelOctree;

	bool m_isPickModelMode;
};

