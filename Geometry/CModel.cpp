#include "CModel.h"
#include "QuickMeshDraw.h"
#include "OBBEstimator.h"
#include "TriTriIntersect.h"
#include "Skeleton.h"
#include "Voxeler.h"
#include <QFile>

const double VOXEL_SIZE = 0.05;

CModel::CModel()
{
	m_label = QString();
	m_mesh = NULL;
	m_voxeler = NULL;
	m_isVoxelized = false;

	m_isPicked = false;
	m_isInteractSkeleton = false;
	m_transMat = Eigen::Matrix4d::Identity();
	m_isTransformed = false;

	m_isFixed = false;

	suppParentID = -1;
}

CModel::~CModel()
{

}

void CModel::loadModel(QString filename, double metric)
{
	m_mesh = new SurfaceMesh::SurfaceMeshModel(filename, m_label);
	m_metric = metric;

	//m_mesh->read(qPrintable(filename));

	this->readScanObj(m_mesh, qPrintable(filename));

	m_mesh->update_face_normals();
	m_mesh->update_vertex_normals();
	m_mesh->updateBoundingBox();

	setAABB();
	m_GOBB.transMat = m_transMat;
}

void CModel::readScanObj(SurfaceMesh::SurfaceMeshModel *mesh, std::string filename)
{
	char   s[200];
	float  x, y, z;
	std::vector<Surface_mesh::Vertex>  vertices;


	// open file (in ASCII mode)
	FILE* in = fopen(filename.c_str(), "r");
	if (!in);


	// if mesh is not empty we need an offset for vertex indices
	// also take into accout that OBJ indices start at 1 (not 0)
	const int voffset = mesh->n_vertices() - 1;


	// clear line once
	memset(&s, 0, 200);


	// parse line by line (currently only supports vertex positions & faces
	while (in && !feof(in) && fgets(s, 200, in))
	{
		// comment
		if (s[0] == '#' || isspace(s[0])) continue;

		// vertex
		else if (strncmp(s, "v ", 2) == 0)
		{
			if (sscanf(s, "v %f %f %f", &x, &y, &z))
			{
				mesh->add_vertex(Surface_mesh::Point(x*m_metric, y*m_metric, z*m_metric));
			}
		}

		// face
		else if (strncmp(s, "f ", 2) == 0)
		{
			int component(0), nV(0);
			bool endOfVertex(false);
			char *p0, *p1(s + 1);

			vertices.clear();

			// skip white-spaces
			while (*p1 == ' ') ++p1;

			while (p1)
			{
				p0 = p1;

				// overwrite next separator

				// skip '/', '\n', ' ', '\0', '\r' <-- don't forget Windows
				while (*p1 != '/' && *p1 != '\r' && *p1 != '\n' && *p1 != ' ' && *p1 != '\0') ++p1;

				// detect end of vertex
				if (*p1 != '/')
				{
					endOfVertex = true;
				}

				// replace separator by '\0'
				if (*p1 != '\0')
				{
					*p1 = '\0';
					p1++; // point to next token
				}

				// detect end of line and break
				if (*p1 == '\0' || *p1 == '\n')
				{
					p1 = 0;
				}

				// read next vertex component
				if (*p0 != '\0')
				{
					switch (component)
					{
					case 0: // vertex
						vertices.push_back(Surface_mesh::Vertex(atoi(p0) + voffset));
						break;

					case 1: // texture coord
						break;

					case 2: // normal
						break;
					}
				}

				++component;

				if (endOfVertex)
				{
					component = 0;
					nV++;
					endOfVertex = false;
				}
			}

			int v0 = vertices[0].idx();
			int v1 = vertices[1].idx();
			int v2 = vertices[2].idx();

 			if (!(v0 == v1 | v1 == v2 | v0 == v2))
				mesh->add_face(vertices);
		}


		// clear line
		memset(&s, 0, 200);
	}


	fclose(in);
}

void CModel::extractToPointCloud()
{
	Surface_mesh::Vertex_property<Surface_mesh::Vector3> points = m_mesh->vertex_property<Surface_mesh::Vector3>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = m_mesh->vertices_end();

	m_pts.resize(m_mesh->n_vertices());
	int id = 0;

	for (vit = m_mesh->vertices_begin(); vit != vend; vit++)
	{
		m_pts[id] = points[vit];
		id++;
	}
}

SimplePointCloud& CModel::getPointCloud()
{
	if (m_pts.isEmpty())
	{
		extractToPointCloud();
	}
	
	return m_pts;
}

void CModel::draw()
{
	glCallList(m_displayListID); 
}

void CModel::buildDisplayList()
{
	if (glIsList(m_displayListID))
	{
		glDeleteLists(m_displayListID, 1);
	}

	m_displayListID = glGenLists(1);

	QColor c;
	if (m_label == "other")
	{
		c = QColor(128, 128, 128, 255);
	}
	else
	{
		c = GetColorFromSet(m_id);
	}

	glNewList(m_displayListID, GL_COMPILE);
	QuickMeshDraw::drawMeshSolid(m_mesh, c, m_transMat);
	glEndList();
}

void CModel::drawAABBox()
{
	QColor c = GetColorFromSet(m_id);

	glPushMatrix();
	glMultMatrixd(m_transMat.data());
	QuickMeshDraw::drawAABBox(m_mesh->bbox(), c);
	glPopMatrix();
}

/*
bool CModel::IsSupport(CModel *pOther, double dDistT, const MathLib::Vector3 &Upright)
{
	if (pOther == NULL) {
		return false;
	}
	double dAngleT = 2.0;
	if (!m_AABB.IsIntersect(pOther->m_AABB, dDistT*4.0)) {	// if too distant
		return false;
	}
	if (m_GOBB.IsSupport(pOther->m_GOBB, dAngleT, dDistT, Upright)) {
		return true;
	}
	for (unsigned int i = 0; i < m_Comp.size(); i++)
	{
		CNMMesh *pMI = m_Comp[i];
		for (unsigned int fi = 0; fi < pMI->m_F.size(); fi++)
		{
			const std::vector<MathLib::Vector3> &VI = pMI->m_V;
			const CFacet &FI = pMI->m_F[fi];
			const MathLib::Vector3 &FNI = pMI->m_FN[fi];
			if (Acos(Abs(FNI.dot(Upright))) > 1.0) {
				continue;
			}
			for (unsigned int j = 0; j < pOther->m_Comp.size(); j++)
			{
				CNMMesh *pMJ = pOther->m_Comp[j];
				for (unsigned int fj = 0; fj < pMJ->m_F.size(); fj++)
				{
					const std::vector<MathLib::Vector3> &VJ = pMJ->m_V;
					const CFacet &FJ = pMJ->m_F[fj];
					const MathLib::Vector3 &FNJ = pMJ->m_FN[fj];
					// 					if (m_GOBB.IsTwoSide(pOther->m_GOBB,VI[FI[0]],Upright)) {
					// 						if (ContactTriTri(VI[FI[0]],VI[FI[1]],VI[FI[2]],FNI,VJ[FJ[0]],VJ[FJ[1]],VJ[FJ[2]],FNJ,dDistT)) {
					// 							return true;
					// 						}
					// 					}
					if (ContactTriTri(VI[FI[0]], VI[FI[1]], VI[FI[2]], FNI, VJ[FJ[0]], VJ[FJ[1]], VJ[FJ[2]], FNJ, dAngleT, dDistT, true)) {
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool CModel::IsContact(CModel *pOther, double dDistT, MathLib::Vector3 &Dir)
{
	if (pOther == NULL) {
		return false;
	}
	double dAngleT = 1.0;
	if (!m_AABB.IsIntersect(pOther->m_AABB, dDistT*4.0)) {
		return false;
	}
	if (m_GOBB.IsContact(pOther->m_GOBB, dAngleT, dDistT, Dir)) {
		return true;
	}
	for (unsigned int i = 0; i < m_Comp.size(); i++)
	{
		CNMMesh *pMI = m_Comp[i];
		for (unsigned int fi = 0; fi < pMI->m_F.size(); fi++)
		{
			const std::vector<MathLib::Vector3> &VI = pMI->m_V;
			const CFacet &FI = pMI->m_F[fi];
			const MathLib::Vector3 &FNI = pMI->m_FN[fi];
			for (unsigned int j = 0; j < pOther->m_Comp.size(); j++)
			{
				CNMMesh *pMJ = pOther->m_Comp[j];
				for (unsigned int fj = 0; fj < pMJ->m_F.size(); fj++)
				{
					const std::vector<MathLib::Vector3> &VJ = pMJ->m_V;
					const CFacet &FJ = pMJ->m_F[fj];
					const MathLib::Vector3 &FNJ = pMJ->m_FN[fj];
					// 					if (m_GOBB.IsTwoSide(pOther->m_GOBB,VI[FI[0]],FNI)) {
					// 						if (ContactTriTri(VI[FI[0]],VI[FI[1]],VI[FI[2]],FNI,VJ[FJ[0]],VJ[FJ[1]],VJ[FJ[2]],FNJ,dDistT)) {
					// 							Dir = FNI;
					// 							return true;
					// 						}
					// 					}
					if (ContactTriTri(VI[FI[0]], VI[FI[1]], VI[FI[2]], FNI, VJ[FJ[0]], VJ[FJ[1]], VJ[FJ[2]], FNJ, dAngleT, dDistT, true)) {
						Dir = FNI;
						return true;
					}
				}
			}
		}
	}
	return false;
}
*/

void CModel::setAABB()
{
	Surface_mesh::Vector3 cornerMin, cornerMax;
	cornerMin = m_mesh->bbox().corner(Eigen::AlignedBox3d::BottomLeftFloor);
	cornerMax = m_mesh->bbox().corner(Eigen::AlignedBox3d::TopRightCeil);

	m_AABB.SetDataM(MathLib::Vector3(cornerMin[0], cornerMin[1], cornerMin[2]), 
		MathLib::Vector3(cornerMax[0], cornerMax[1], cornerMax[2]));
}

void CModel::computeOBB(int fixAxis /*= -1*/)
{
	if (m_pts.isEmpty())
	{
		extractToPointCloud();
	}
	
	// convert to point set type
	std::vector<MathLib::Vector3> tempPts(m_pts.size());

	for (int i = 0; i < tempPts.size(); i++)
	{
		tempPts[i] = MathLib::Vector3(m_pts[i][0], m_pts[i][1], m_pts[i][2]);
	}

	COBBEstimator OBBE(&tempPts, &m_GOBB);

	OBBE.ComputeOBB_Min(fixAxis);
}

void CModel::drawOBB()
{

	//QColor c = GetColorFromSet(m_id);
	//c.setAlphaF(0.2);

	//glColorQt(c);

	glPushMatrix();
	glMultMatrixd(m_transMat.data());

	m_GOBB.DrawBox(false, m_isPicked, false, false, m_isInteractSkeleton);
	glPopMatrix();
}

bool CModel::IsSimilar(CModel *pOther, const MathLib::Vector3 &Upright)
{
	if (pOther == NULL) {
		return false;
	}
	if (m_label != pOther->m_label) {
		return false;
	}

	if (m_label == "book" || m_label == "box" || m_label == "folder" || m_label == "disk" || m_label == "plate" || m_label == "compact_disk") {
		// for things which may be laid down
		std::vector<double> S1(3), S2(3);
		for (unsigned int i = 0; i < 3; i++) {
			S1[i] = m_GOBB.size[i];
			S2[i] = pOther->m_GOBB.size[i];
		}
		std::sort(S1.begin(), S1.end());
		std::sort(S2.begin(), S2.end());
		return (MathLib::IsEqual(S1[0], S2[0], MathLib::Max(S1[0], S2[0])*.6) && MathLib::IsEqual(S1[1], S2[1], MathLib::Max(S1[1], S2[1])*.6) && MathLib::IsEqual(S1[2], S2[2], MathLib::Max(S1[2], S2[2])*.6));
	}
	else {
		std::vector<double> S1(3), S2(3);
		double dD(0), dMaxD1(0), dMaxD2(0);
		int iMaxId1(0), iMaxId2(0);
		COBB &bb1(m_GOBB), &bb2(pOther->m_GOBB);
		for (unsigned int i = 0; i < 3; i++) {
			dD = MathLib::Abs(bb1.axis[i].dot(Upright));
			if (dD > dMaxD1) { dMaxD1 = dD; iMaxId1 = i; }
			dD = MathLib::Abs(bb2.axis[i].dot(Upright));
			if (dD > dMaxD2) { dMaxD2 = dD; iMaxId2 = i; }
		}
		S1[0] = bb1.size[iMaxId1];
		S1[1] = (bb1.size[(iMaxId1 + 1) % 3] > bb1.size[(iMaxId1 + 2) % 3]) ? bb1.size[(iMaxId1 + 1) % 3] : bb1.size[(iMaxId1 + 2) % 3];
		S1[2] = (bb1.size[(iMaxId1 + 1) % 3] > bb1.size[(iMaxId1 + 2) % 3]) ? bb1.size[(iMaxId1 + 2) % 3] : bb1.size[(iMaxId1 + 1) % 3];
		S2[0] = bb2.size[iMaxId2];
		S2[1] = (bb2.size[(iMaxId2 + 1) % 3] > bb2.size[(iMaxId2 + 2) % 3]) ? bb2.size[(iMaxId2 + 1) % 3] : bb2.size[(iMaxId2 + 2) % 3];
		S2[2] = (bb2.size[(iMaxId2 + 1) % 3] > bb2.size[(iMaxId2 + 2) % 3]) ? bb2.size[(iMaxId2 + 2) % 3] : bb2.size[(iMaxId2 + 1) % 3];
		return (MathLib::IsEqual(S1[0], S2[0], MathLib::Max(S1[0], S2[0])*.3) && MathLib::IsEqual(S1[1], S2[1], MathLib::Max(S1[1], S2[1])*.3) && MathLib::IsEqual(S1[2], S2[2], MathLib::Max(S1[2], S2[2])*.3));
	}
}

bool CModel::IsContain(CModel *pOther)
{
	if (pOther == NULL) {
		return false;
	}
	if (!m_AABB.IsIntersect(pOther->m_AABB, 0.02)) {
		return false;
	}
	return m_GOBB.IsContain(pOther->m_GOBB, 0.9);
}

bool CModel::IsContact(CModel *pOther, bool onlyOBB, double dDistT, MathLib::Vector3 &Dir)
{
	if (pOther == NULL) {
		return false;
	}
	double dAngleT = 1.0;
	if (!m_AABB.IsIntersect(pOther->m_AABB, dDistT*4.0)) {
		return false;
	}
	if (onlyOBB && m_GOBB.IsContact(pOther->m_GOBB, dAngleT, dDistT, Dir)) {
		return true;
	}

	Surface_mesh *mesh_other = pOther->meshData();

	Surface_mesh::Vertex_property<Surface_mesh::Vector3> points = m_mesh->vertex_property<Surface_mesh::Vector3>("v:point");
	Surface_mesh::Face_property<Surface_mesh::Vector3> fnormals = m_mesh->face_property<Surface_mesh::Vector3>("f:normal");

	Surface_mesh::Face_iterator fit, fend = m_mesh->faces_end();
	Surface_mesh::Vertex_around_face_circulator fvit, fvend;

	Surface_mesh::Vertex_property<Surface_mesh::Vector3> points_other = mesh_other->vertex_property<Surface_mesh::Vector3>("v:point");
	Surface_mesh::Face_property<Surface_mesh::Vector3> fnormals_other = mesh_other->face_property<Surface_mesh::Vector3>("f:normal");

	Surface_mesh::Face_iterator fit_other, fend_other = mesh_other->faces_end();
	Surface_mesh::Vertex_around_face_circulator fvit_other, fvend_other;

	for (fit = m_mesh->faces_begin(); fit != fend; ++fit)
	{
		MathLib::Vector3 faceNormal = MathLib::Vector3(fnormals[fit][0], fnormals[fit][1], fnormals[fit][2]);
		MathLib::Vector3 faceVert[3], faceVert_other[3];
		fvit = fvend = m_mesh->vertices(fit);
		int vertID = 0;
		// collect vertices of the face
		do{
			Surface_mesh::Vector3 vert = points[fvit];
			faceVert[vertID++] = MathLib::Vector3(vert[0], vert[1], vert[2]);
		} while (++fvit != fvend);


		for (fit_other = mesh_other->faces_begin(); fit_other != fend_other; ++fit_other)
		{
			MathLib::Vector3 faceNormal_other =
				MathLib::Vector3(fnormals_other[fit_other][0],
				fnormals_other[fit_other][1],
				fnormals_other[fit_other][2]);

			fvit_other = fvend_other = mesh_other->vertices(fit_other);
			int vertID = 0;
			// collect vertices of the face
			do{
				Surface_mesh::Vector3 vert = points_other[fvit_other];
				faceVert_other[vertID++] = MathLib::Vector3(vert[0], vert[1], vert[2]);
			} while (++fvit_other != fvend_other);

			if (ContactTriTri(faceVert[0], faceVert[1], faceVert[2], faceNormal,
				faceVert_other[0], faceVert_other[1], faceVert_other[2], faceNormal_other,
				dAngleT, dDistT, true)) {
				return true;
			}
		}
	}

	return false;
}

bool CModel::IsSupport(CModel *pOther, bool onlyOBB, double dDistT, const MathLib::Vector3 &Upright)
{
	if (pOther == NULL) {
		return false;
	}
	double dAngleT = 2.0;
	if (!m_AABB.IsIntersect(pOther->m_AABB, dDistT*4.0)) {	// if too distant
		return false;
	}

	if (onlyOBB && m_GOBB.IsSupport(pOther->m_GOBB, dAngleT, dDistT*2, Upright)) {
		return true;
	}

	Surface_mesh *mesh_other = pOther->meshData();

	Surface_mesh::Vertex_property<Surface_mesh::Vector3> points = m_mesh->vertex_property<Surface_mesh::Vector3>("v:point");
	Surface_mesh::Face_property<Surface_mesh::Vector3> fnormals = m_mesh->face_property<Surface_mesh::Vector3>("f:normal");

	Surface_mesh::Face_iterator fit, fend = m_mesh->faces_end();
	Surface_mesh::Vertex_around_face_circulator fvit, fvend;

	Surface_mesh::Vertex_property<Surface_mesh::Vector3> points_other = mesh_other->vertex_property<Surface_mesh::Vector3>("v:point");
	Surface_mesh::Face_property<Surface_mesh::Vector3> fnormals_other = mesh_other->face_property<Surface_mesh::Vector3>("f:normal");

	Surface_mesh::Face_iterator fit_other, fend_other = mesh_other->faces_end();
	Surface_mesh::Vertex_around_face_circulator fvit_other, fvend_other;

	for (fit = m_mesh->faces_begin(); fit != fend; ++fit)
	{
		MathLib::Vector3 faceNormal = MathLib::Vector3(fnormals[fit][0], fnormals[fit][1], fnormals[fit][2]);
		if (Acos(Abs(faceNormal.dot(Upright))) > 1.0) {
			continue;
		}

		MathLib::Vector3 faceVert[3], faceVert_other[3];
		fvit = fvend = m_mesh->vertices(fit);
		int vertID = 0;
		// collect vertices of the face
		do{
			Surface_mesh::Vector3 vert = points[fvit];
			faceVert[vertID++] = MathLib::Vector3(vert[0], vert[1], vert[2]);
		} while (++fvit != fvend);


		for (fit_other = mesh_other->faces_begin(); fit_other != fend_other; ++fit_other)
		{
			MathLib::Vector3 faceNormal_other =
				MathLib::Vector3(fnormals_other[fit_other][0],
				fnormals_other[fit_other][1],
				fnormals_other[fit_other][2]);

			fvit_other = fvend_other = mesh_other->vertices(fit_other);
			int vertID = 0;
			// collect vertices of the face
			do{
				Surface_mesh::Vector3 vert = points_other[fvit_other];
				faceVert_other[vertID++] = MathLib::Vector3(vert[0], vert[1], vert[2]);
			} while (++fvit_other != fvend_other);

			if (ContactTriTri(faceVert[0], faceVert[1], faceVert[2], faceNormal,
				faceVert_other[0], faceVert_other[1], faceVert_other[2], faceNormal_other,
				dAngleT, dDistT, true)) {
				return true;
			}
		}
	}

	return false;
}

void CModel::buildSuppPlane()
{
	m_suppPlaneBuilder = new SuppPlaneBuilder(this);

	m_suppPlaneBuilder->build(2);

}

void CModel::testInteractSkeleton(Skeleton *sk)
{
	updateOBBTransMat();

	if (m_GOBB.IsInteract(sk->getJoints(), 0.05, sk->states))
	{
		m_isInteractSkeleton = true;
	}

	else
	{
		m_isInteractSkeleton = false;
	}
}

void CModel::setTransMat(const Eigen::Matrix4d &m)
{
	m_transMat = m;

	//rebuild display list
	buildDisplayList();
}

SurfaceMesh::Vector3 CModel::getTransformedOBBCenter()
{
	MathLib::Vector3 trans_cent = m_GOBB.GetTransformedCenter();

	return SurfaceMesh::Vector3(trans_cent.x, trans_cent.y, trans_cent.z);
}

QVector<SurfaceMesh::Vector3> CModel::getTransformedOBBVertices()
{	
	std::vector<MathLib::Vector3> temp_vp = m_GOBB.GetTransformedVertices();
	QVector<SurfaceMesh::Vector3> trans_vp(temp_vp.size());

	for (int i = 0; i < temp_vp.size(); i++)
	{
		trans_vp[i] = SurfaceMesh::Vector3(temp_vp[i].x, temp_vp[i].y, temp_vp[i].z);
	}

	return trans_vp;
}

double CModel::getOBBBottomHeight(MathLib::Vector3 &uprightVec)
{
	return m_GOBB.GetBottomHeight(uprightVec);
}

std::vector<double> CModel::getAABBXYRange()
{
	std::vector<double> rangeVals(4);

	MathLib::Vector3 minV, maxV;
	minV = m_AABB.GetMinV();
	maxV = m_AABB.GetMaxV();

	rangeVals[0] = minV.x;
	rangeVals[1] = maxV.x;

	rangeVals[2] = minV.y;
	rangeVals[3] = maxV.y;

	return rangeVals;
 }

void CModel::voxelize()
{
	std::vector<VoxelerLibrary::Voxel > voxels;

	if (m_voxeler)
	{
		delete m_voxeler;
	}

	QString voxelFileName = m_filePath + "/" + m_label + ".voxel";

	if (loadVoxelData(voxelFileName, voxels))
	{
		m_voxeler = new VoxelerLibrary::Voxeler(voxels, m_voxelSize);
	}

	else
	{
		m_voxelSize = VOXEL_SIZE;
		m_voxeler = new VoxelerLibrary::Voxeler(m_mesh, m_voxelSize);
		
		m_voxeler->saveVoxelData(voxelFileName);
	}

	m_voxeler->setupDraw();

	m_isVoxelized = true;
}

void CModel::drawVoxel()
{
	GLfloat blue[] = { 0.0f, 0.0f, 1.0f, 1.0f };

	glColor3f(blue[0], blue[1], blue[2]);
	m_voxeler->draw();
}

bool CModel::isPointInside(SurfaceMesh::Vector3 &point)
{
	bool isIntersect = m_voxeler->isIntersectWithPoint(point);
	return isIntersect;
}

void CModel::setLabel(QString &l)
{
	m_label = l;

	if (m_label == "floor")
	{
		m_isFixed = true;
	}
}

void CModel::drawVoxelOctree()
{
	//debug
	//if (m_label == "box")
	{
		m_voxeler->drawOctree();
	}	
}

bool CModel::isSegmentIntersect(SurfaceMesh::Vector3 &startPt, SurfaceMesh::Vector3 &endPt)
{
	return m_voxeler->isIntersectSegment(startPt, endPt);
}

bool CModel::loadVoxelData(const QString &filename, std::vector<VoxelerLibrary::Voxel> &voxels)
{
	QFile inFile(filename);

	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

	ifs >> m_voxelNum;
	ifs >> m_voxelSize;

	VoxelerLibrary::Voxel v;

	for (int i = 0; i < m_voxelNum; i++)
	{
		ifs >> v.x >> v.y >> v.z;
		voxels.push_back(v);
	}

	inFile.close();

	return true;
}

std::vector<double> CModel::getOBBSize()
{
	std::vector<double> obbSize;

	obbSize.push_back(m_GOBB.size.x);
	obbSize.push_back(m_GOBB.size.y);
	obbSize.push_back(m_GOBB.size.z);

	return obbSize;
}

double CModel::getClosestDistToVoxel(SurfaceMesh::Vector3 &pt)
{
	if (!m_isVoxelized)
	{
		voxelize();
	}

	return m_voxeler->getClosestDistToVoxle(pt);
}
