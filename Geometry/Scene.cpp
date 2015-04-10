#include "Scene.h"

#include <QFileDialog>
#include <QTextStream>
#include <iostream>
#include <fstream>

#include "Skeleton.h"

#include "font.inl"

CScene::CScene() :
m_SuppThresh(0.1),
m_ContactThresh(0.1),
m_hasSupportHierarchy(false),
m_isShowModel(true),
m_isShowOBB(true),
m_isShowRG(true),
m_isShowModelName(false),
m_isShowModelVoxel(false),
m_isPickModelMode(false)
{
	m_modelNum = 0;
	m_uprightVec = MathLib::Vector3(0, 0, 1.0);
	m_sceneTransMat = Eigen::Matrix4d::Identity(4, 4);

	m_metricConvert = 1.0;
	m_centerModelID = 0;

	fontImage = QImage(":/icon/font.png");
}

CScene::~CScene()
{

}

void CScene::loadScene(const QString filename)
{
	QFile inFile(filename);
	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	
	QFileInfo sceneFileInfo(inFile.fileName());
	m_sceneFileName = sceneFileInfo.baseName();
	m_sceneFilePath = sceneFileInfo.absolutePath() + "/";
	
	QString modelName;
	double modelMetric;

	ifs>>m_modelNum;
	ifs >> modelMetric;

	// read the rest of line
	ifs.readLine();

	for (int i = 0; i < m_modelNum; i++)
	{
		modelName = ifs.readLine();
		CModel *newModel = new CModel();
		newModel->loadModel(m_sceneFilePath + "/" + modelName + ".obj",modelMetric);	
		newModel->setLabel(modelName);
		newModel->setFilePath(m_sceneFilePath);
		newModel->setID(i);
		m_modelList.push_back(newModel);
		m_modelNameIdMap[modelName] = i;
	}

	// set scene bbox
	for (int i = 0; i < m_modelNum; i++)
	{
		Eigen::AlignedBox3d curbox = m_modelList[i]->bbox();
		m_bbox = m_bbox.merged(curbox);
	}

	buildModelDislayList();

	bool silentMode = true;
	if (readModelOBB(silentMode) != 0)
	{
		computeModelOBB();
		writeModelOBB(silentMode);
	}

	if (readRG(silentMode) != 0)
	{
		buildRelationshipGraph();
		writeRG(silentMode);
	}
}

void CScene::extractToPointCloud()
{
	int id = 0;

	for each (CModel *m in m_modelList)
	{
		SimplePointCloud& mpts = m->getPointCloud();
		mpts.setID(id++);
		mpts.setLabel(m->label().toStdString());
		m_points.push_back(mpts);
	}
}

QVector<SimplePointCloud> CScene::getSelectedPointCloud(std::vector<int> ids)
{
	if (m_points.size() == 0)
	{
		extractToPointCloud();
	}

	QVector<SimplePointCloud> clouds;

	for (int i = 0; i < ids.size();i++)
	{
		clouds.push_back(m_points[ids[i]]);
	}

	return clouds;
}

void CScene::draw()
{
	if (m_modelList.size())
	{
		int id = 0;

		//glPushMatrix();
		//glMultMatrixd(m_transMat.data());

		foreach(CModel *m, m_modelList)
		{
			if (m_isShowModel)
			{
				m->draw();
			}
			else
			{
				if (m->isVoxelized() && m_isShowModelVoxel && m_centerModelID == m->getID())
				{
					m->drawVoxel();
					//m->drawVoxelOctree();
				}
			}
					
			if (m_isShowOBB)
			{
				//m->drawAABBox();
				m->drawOBB();
			}
		}

		if (m_isShowRG)
		{
			drawRelationGraph();
		}

		if (m_isShowModelName)
		{
			drawModelName();
		}

		//glPopMatrix();
	}
}

void CScene::buildModelDislayList()
{
	foreach(CModel *m, m_modelList)
	{
		m->buildDisplayList();
	}
}

int CScene::readModelOBB(bool bSilent)
{
	QString filename = m_sceneFilePath + m_sceneFileName + ".obb";

	std::ifstream ifs(filename.toStdString());

	if (!ifs.is_open())
	{
		if (!bSilent) Simple_Message_Box(QString("Read obb: cannot open %1").arg(filename));
		return -1;
	}

	int num(0);
	char buf[MAX_STR_BUF_SIZE];
	ifs >> buf;

	while (!ifs.eof()) {
		switch (buf[0]) {
		case 'N':
			ifs >> num;
			if (num != m_modelList.size()) {
				Simple_Message_Box("ReadOBB: Number of models does not match!");
				return -1;
			}

			for (int i = 0; i < num; i++) {
				m_modelList[i]->m_GOBB.ReadData(ifs);
			}
			break;
		default:
			ifs.getline(buf, MAX_STR_BUF_SIZE, '\n');
		}
	}

	ifs.close();
}

int CScene::writeModelOBB(bool bSilent)
{
	QString filename = m_sceneFilePath + m_sceneFileName + ".obb";

	std::ofstream  ofs(filename.toStdString());

	if (!ofs.is_open())

	{
		if (!bSilent) Simple_Message_Box(QString("Write obb: cannot open %1").arg(filename));
		return -1;
	}
		
	ofs << "N " << m_modelList.size() << "\n";
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		m_modelList[i]->m_GOBB.WriteData(ofs);
	}


	ofs.close();
	return 0;
}

void CScene::computeModelOBB()
{
	for (int i = 0; i < m_modelList.size();i++)
	{
		// fix z axis, minimum obb in xy plane
		m_modelList[i]->computeOBB(2);
	}
}

void CScene::buildRelationshipGraph()
{
	if (m_modelDistMat.IsEmpty()) {
		computeModelDistMat();
	}
	if (m_ConnStrenMat.IsEmpty()) {
		computeConnStrengthMat();
	}
	if (m_onTopList.empty()) {
		computeOnTopList();
	}
	if (m_SymGrp.empty()) {
		extractSymGroup();
	}

	m_RG.Initialize(m_modelList.size());

	// The following tests must be performed sequentially
	//	extractContactRel();
	//	extractContainRel();
	extractSupportRel();
	extractProximityRel(); // extract local communities
	pruneSupportRel();
	//doSymmetryConnection(CT_PROXIMITY);
	//doSymmetryConnection(CT_SUPPORT);
	connectCommunities();
	
	//	doSymmetryConnection(CT_WEAK);
	//uniformizeRGWeights();
}

int CScene::extractProximityRel(void)
{
	// Connect every two models whose AABBs intersect
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		for (unsigned int j = i + 1; j < m_modelList.size(); j++) {
			if (m_RG.IsEdge(i, j) || m_onTopList[i] != m_onTopList[j]) {
				continue;
			}
			if (IntersectOBBOBB(m_modelList[i]->m_GOBB, m_modelList[j]->m_GOBB)) {
				m_RG.InsertEdge(i, j, CT_PROXIMITY); // proximity connection
			}
		}
	}
	// Every model connects to its most strongly connected model
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		if (m_RG.HasEdge(i, CT_CONTACT) || m_RG.HasEdge(i, CT_SUPPORT)) { // has contact/support relation
			continue;
		}
		double dMaxCS(0);
		unsigned int iMaxID(0);
		double dMaxCS_SL(0);	// for same level models (two models are of the same level if their obbs are on top of the same model's obb)
		unsigned int iMaxID_SL(0);	// for same level models
		for (unsigned int j = 0; j < m_modelList.size(); j++) {
			if (j == i) { continue; }
			if (m_ConnStrenMat.Get(i, j) > dMaxCS) {
				dMaxCS = m_ConnStrenMat.Get(i, j);
				iMaxID = j;
			}
			if (m_onTopList[i] == m_onTopList[j] && m_ConnStrenMat.Get(i, j) > dMaxCS_SL) {	// for same level models 
				dMaxCS_SL = m_ConnStrenMat.Get(i, j);
				iMaxID_SL = j;
			}
		}
		if (dMaxCS_SL > 0) {
			m_RG.InsertEdge(i, iMaxID_SL, CT_PROXIMITY); // proximity connection
		}
		else {
			m_RG.InsertEdge(i, iMaxID, CT_PROXIMITY); // proximity connection
		}
	}
	return 0;
}

int CScene::extractSupportRel()
{

	double dT = m_SuppThresh / m_metricConvert;
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		for (unsigned int j = i + 1; j < m_modelList.size(); j++) {
			CModel *pMJ = m_modelList[j];
			MathLib::Vector3 cd;
			if (pMI->IsSupport(pMJ, true, dT, m_uprightVec)) {
				m_RG.InsertEdge(i, j, CT_SUPPORT);	// upright support
			}
		}
	}

	return 0;
}

int CScene::pruneSupportRel()
{
	// collect direct support info.
	std::vector<std::vector<int>> SuppList(m_RG.Size());	// support giver list, models that are being supported
	for (unsigned int ei = 0; ei < m_RG.ESize(); ei++) {
		if (m_RG.GetEdge(ei)->t == CT_SUPPORT) {
			CModel *pM1 = m_modelList[m_RG.GetEdge(ei)->v1];
			CModel *pM2 = m_modelList[m_RG.GetEdge(ei)->v2];
			
			//if pM1 is higher than pM2, then pM2 is in support list of pM1
			if (pM1->m_GOBB.BottomHeightDiff(pM2->m_GOBB, m_uprightVec) > 0.0) {
				SuppList[m_RG.GetEdge(ei)->v1].push_back(m_RG.GetEdge(ei)->v2);
			}
			else {
				SuppList[m_RG.GetEdge(ei)->v2].push_back(m_RG.GetEdge(ei)->v1);
			}
		}
	}

	// check nodes with more than more supporters (if multiple nodes support this node)
	// filter those who doesn't contact
	for (unsigned int i = 0; i < SuppList.size(); i++)
	{
		if (SuppList[i].size() > 1)
		{
			CModel *pM1 = m_modelList[i];
			for (int j = 0; j < SuppList[i].size();j++)
			{
				CModel *pM2 = m_modelList[SuppList[i][j]];
				
				// use a relative large support threshold for conservative pruning
				if (!pM1->IsSupport(pM2, true, 10*m_SuppThresh, m_uprightVec))
				{
					m_RG.DeleteEdge(i, SuppList[i][j]);
				}
				else
				{
					m_onTopList[i] = SuppList[i][j];
				}
			}
		}
	}

	return 0;
}

int CScene::extractContactRel()
{
	double dT = m_ContactThresh / m_metricConvert;
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		for (unsigned int j = i + 1; j < m_modelList.size(); j++) {
			CModel *pMJ = m_modelList[j];
			MathLib::Vector3 cd;
			if (pMI->IsContact(pMJ, true, dT, cd)) {
				if (Acos(Abs(cd.dot(m_uprightVec))) < 1.0) {
					m_RG.InsertEdge(i, j, CT_SUPPORT);	// upright support
				}
				else {
					m_RG.InsertEdge(i, j, CT_CONTACT);	// plain contact
				}
			}
		}
	}

	return 0;
}

int CScene::extractContainRel(void)
{
	// First extract all containment and contact relations
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		for (unsigned int j = i + 1; j<m_modelList.size(); j++) {
			CModel *pMJ = m_modelList[j];
			if (m_RG.IsEdge(i, j)) { // if it is an edge already, skip it
				continue;
			}
			if (pMI->m_GOBB.vol > pMJ->m_GOBB.vol) {
				if (pMI->IsContain(pMJ)) {
					m_RG.InsertEdge(i, j, CT_CONTAIN);	// contain
				}
			}
			else {
				if (pMJ->IsContain(pMI)) {
					m_RG.InsertEdge(i, j, CT_CONTAIN);	// contain
				}
			}
		}
	}
	return 0;
}

void CScene::computeConnStrengthMat(void)
{
	m_ConnStrenMat.Resize(m_modelList.size());
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		for (unsigned int j = i + 1; j < m_modelList.size(); j++) {
			CModel *pMJ = m_modelList[j];
			m_ConnStrenMat.Set(i, j, pMI->m_GOBB.ConnStrength_CD(pMJ->m_GOBB));
		}
	}
}

void CScene::computeOnTopList(void)
{
	m_onTopList.resize(m_modelList.size(), -1);
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		double dD(std::numeric_limits<double>::max());
		int iID(-1);
		for (unsigned int j = 0; j < m_modelList.size(); j++) {
			if (i == j) { continue; }
			CModel *pMJ = m_modelList[j];
			if (pMI->m_GOBB.IsOnTop(pMJ->m_GOBB, m_uprightVec, dD)) {
				iID = j;
			}
		}
		if (iID != -1) {
			m_onTopList[i] = iID;
		}
	}
}

void CScene::computeModelDistMat(void)
{
	m_modelDistMat.Resize(m_modelList.size());
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		double dD(std::numeric_limits<double>::max());
		for (unsigned int j = 0; j < m_modelList.size(); j++) {
			if (i == j) { continue; }
			CModel *pMJ = m_modelList[j];
			m_modelDistMat.Set(i, j, sqrt(pMI->m_GOBB.SqDistance_Approx(pMJ->m_GOBB)));
		}
	}
}

int CScene::extractSymGroup()
{
	m_SymMap.clear();
	m_SymMap.resize(m_modelList.size(), -1);
	int iSymID(0);
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		CModel *pMI = m_modelList[i];
		for (unsigned int j = i + 1; j < m_modelList.size(); j++) {
			CModel *pMJ = m_modelList[j];
			if (pMI->IsSimilar(pMJ, m_uprightVec)) {
				if (m_SymMap[i] != -1) {
					m_SymMap[j] = m_SymMap[i];
				}
				else if (m_SymMap[j] != -1) {
					m_SymMap[i] = m_SymMap[j];
				}
				else {
					m_SymMap[i] = m_SymMap[j] = iSymID++;
				}
			}
		}
	}
	if (iSymID == 0) {
		m_SymGrp.clear();
		return 0;
	}
	m_SymGrp.clear();
	m_SymGrp.resize(iSymID); // symmetry group list
	for (unsigned int i = 0; i < m_SymGrp.size(); i++) {
		for (unsigned int j = 0; j < m_SymMap.size(); j++) {
			if (m_SymMap[j] == i) {
				m_SymGrp[i].push_back(j);
			}
		}
	}
	return 0;
}

int CScene::connectCommunities(ConnType ct /*= CT_WEAK*/)
{
	//	double dHDT = m_dSuppT / m_MetricConvert;
	// Find connected components / communities

	std::vector<int> CM; // connected component map
	int iNumC(0);
	m_RG.ComputeComponentMap(CM, iNumC);
	if (iNumC == 1) {
		return 0;
	}
	std::vector<std::vector<int>> CL(iNumC); // connected component list
	for (int i = 0; i < iNumC; i++) {
		for (unsigned int j = 0; j < CM.size(); j++) {
			if (CM[j] == i) {
				CL[i].push_back(j);
			}
		}
	}
	for (unsigned int i = 0; i < CL.size(); i++) {
		double dMaxCS(0);
		unsigned int iMaxI(0), iMaxJ(0);
		double dMaxCS_SL(0);	// for same level models (two models are of the same level if their obbs are on top of the same model's obb)
		unsigned int iMaxI_SL(0), iMaxJ_SL(0);	// for same level models 
		for (unsigned int j = 0; j < CL.size(); j++) {	// for all other connected components
			if (i == j) { continue; }
			for (unsigned int ii = 0; ii < CL[i].size(); ii++) {	// check for all pairs of models between the two connected
				for (unsigned int jj = 0; jj < CL[j].size(); jj++) {
					double dCS = m_ConnStrenMat.Get(CL[i][ii], CL[j][jj]);
					if (dCS > dMaxCS) {
						dMaxCS = dCS;
						iMaxI = CL[i][ii];
						iMaxJ = CL[j][jj];
					}
					if (m_onTopList[CL[i][ii]] == m_onTopList[CL[j][jj]] && dCS > dMaxCS_SL) {	// for same level models 
						dMaxCS_SL = dCS;
						iMaxI_SL = CL[i][ii];
						iMaxJ_SL = CL[j][jj];
					}
				}
			}
		}
		if (dMaxCS_SL > 0) {	// has same level pairs of models
			m_RG.InsertEdge(iMaxI_SL, iMaxJ_SL, ct);	// weak connection
		}
		else {
			m_RG.InsertEdge(iMaxI, iMaxJ, ct);	// weak connection
		}
	}
	connectCommunities(ct);

	return 0;
}

void CScene::drawRelationGraph()
{
	if (m_modelList.empty() || m_RG.IsEmpty()) {
		return;
	}

	GLfloat red[] = { 1.0f, 0.3f, 0.3f, 1.0f };
	GLfloat blue[] = { 0.1f, 0.1f, 1.0f, 1.0f };
	GLfloat green[] = { 0.4f, 1.0f, 0.6f, 1.0f };
	GLubyte color[4] = { 0 };

	glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT | GL_HINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);

	// Centers of the FP's
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_COLOR_MATERIAL);
	glColor4fv(red);
	for (unsigned int i = 0; i < m_modelList.size(); i++) {
		MathLib::Vector3 center = m_modelList[i]->m_GOBB.cent;
		//renderSphere(center[0], center[1], center[2], PointSize3D / m_metricConvert);
	}

	// Relations indicated with lines
	glDisable(GL_LIGHTING);
	glEnable(GL_LINE_STIPPLE);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	for (unsigned int i = 0; i < m_RG.ESize(); i++) {
		const CUDGraph::Edge *e = m_RG.GetEdge(i);
		switch (e->t) {
		case CT_SUPPORT:
			glLineWidth(5.0);
			glColor3f(1.0f, 0.3f, 0.3f); //red
			break;
		case CT_CONTACT:
			glLineWidth(5.0);
			glColor3f(0.8f, 0.5f, 0.1f);
			break;
		case CT_CONTAIN:
			glLineWidth(5.0);
			glColor3f(0.8f, 0.8f, 0.0f);
			break;
		case CT_PROXIMITY:
			glLineWidth(2.0);
			glColor3f(0.8, 0.1, 0.8);  // magenta
			break;
		case CT_SYMMETRY: case CT_PROX_SYM:
			glLineWidth(3.0);
			glColor3f(0.2, 0.8, 0.8);
			break;
		case CT_SUP_SYM: case CT_CONTACT_SYM:
			glLineWidth(3.0);
			glColor3f(0.2, 0.8, 0.2);
			break;
		case CT_WEAK:
			glLineWidth(1.0);
			glColor3f(0.4, 0.4, 0.9); 
			break;
		}
		glBegin(GL_LINES);
		glVertex3dv(m_modelList[e->v1]->m_GOBB.cent.v);
		glVertex3dv(m_modelList[e->v2]->m_GOBB.cent.v);
		glEnd();
	}

	glPopAttrib();
}

int CScene::readRG(bool bSilent)
{
	QString filename = m_sceneFilePath + m_sceneFileName + ".rg";

	std::ifstream ifs(filename.toStdString());

	if (!ifs.is_open())
	{
		if(!bSilent) Simple_Message_Box(QString("Read RG: cannot open %1").arg(filename));
		return -1;
	}

	char buf[MAX_STR_BUF_SIZE];
	ifs >> buf;
	while (!ifs.eof()) {
		switch (buf[0]) {
		case 'N':
			m_RG.Clear();
			m_RG.ReadVert(ifs);
			if (m_RG.Size() != m_modelList.size()) {
				Simple_Message_Box("ReadRG: Number of models does not match!");
				return -1;
			}
			break;
		case 'E':
			m_RG.ReadEdge(ifs);
			break;
		default:
			// eat up rest of the line
			ifs.getline(buf, MAX_STR_BUF_SIZE, '\n');
			break;
		}
		ifs >> buf;
	}
	return 0;
}

int CScene::writeRG(bool bSilent)
{
	QString filename = m_sceneFilePath + m_sceneFileName + ".rg";

	std::ofstream  ofs(filename.toStdString());

	if (!ofs.is_open())
	{
		if (!bSilent) Simple_Message_Box(QString("Write obb: cannot open %1").arg(filename));
		return -1;
	}

	//////////////////////////////////////////////////////////////////////////
	m_RG.WriteVert(ofs);
	m_RG.WriteEdge(ofs);
	//////////////////////////////////////////////////////////////////////////
	return 0;
}

void CScene::testInteractSkeleton(Skeleton *sk)
{
	for (int i = 0; i < m_modelNum; i++)
	{
		m_modelList[i]->testInteractSkeleton(sk);
	}
}

void CScene::setModelTransMat(int modelID, const Eigen::Matrix4d &m)
{
	m_modelList[modelID]->setTransMat(m);
}

QVector<QString> CScene::getModelNameList()
{
	QVector<QString> nameList;

	foreach(CModel *m, m_modelList)
	{
		nameList.push_back(m->label());
	}

	return nameList;
}

void CScene::drawModelName(const QString &s, SurfaceMesh::Vector3 pos)
{
	qglviewer::Vec proj = m_drawArea->camera()->projectedCoordinatesOf(qglviewer::Vec(pos.x(), pos.y(), pos.z()));
	
	sprintf(gl_text_buf, "%s", qPrintable(s));
	drawStringQuad(proj.x - (stringWidth(gl_text_buf) * 0.5), proj.y, gl_text_buf, false);
}

void CScene::drawModelName()
{
	beginDraw2DText();

	glColor4d(1, 1, 1, 1);
	QString info = QString("test info");
	drawStringQuad(50, 50, qPrintable(info), true);

	m_drawArea->qglColor(QColor(0, 0, 255));

	foreach(CModel *m, m_modelList)
	{
		QString modelName = m->label();
		SurfaceMesh::Vector3 center = m->getTransformedOBBCenter();
		drawModelName(modelName, center);
	}

	endDraw2DText();

	//glDisable(GL_LIGHTING);
	//m_drawArea->renderText(100,100, info);
	//glEnable(GL_LIGHTING);
}

void CScene::beginDraw2DText()
{
	glGetDoublev(GL_CURRENT_COLOR, font_color);

	m_drawArea->startScreenCoordinatesSystem();

	initFont(fontImage);

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, fontTexture);

	glBegin(GL_QUADS);
}

void CScene::endDraw2DText()
{
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	m_drawArea->stopScreenCoordinatesSystem();
}

void CScene::pickModelAt(QPoint p)
{
	qglviewer::Vec origin, dir;
	m_drawArea->camera()->convertClickToLine(p, origin, dir);

	std::vector<std::pair<double, int>> pickList;

	foreach(CModel *m, m_modelList)
	{
		double depth(std::numeric_limits<double>::max());

		m->updateOBBTransMat();

		if (m->m_GOBB.PickByRayWithTrans(MathLib::Vector3(origin.x, origin.y, origin.z), MathLib::Vector3(dir.x, dir.y, dir.z), depth));
		{
			pickList.push_back(std::make_pair(depth, m->getID()));
		}
	}

	std::sort(pickList.begin(), pickList.end());

	if (pickList[0].first < 1e3)
	{
		int firstPickedModelID = pickList[0].second;
		m_modelList[firstPickedModelID]->updatePickedState();
	}
}

void CScene::setModelPicked(int modelID, bool state)
{
	m_modelList[modelID]->setPicked(state);
}

void CScene::unPickAllModels()
{
	for (int i = 0; i < m_modelList.size(); i++)
	{
		m_modelList[i]->setPicked(false);
	}
}

QVector<int> CScene::getSelectedModelIDs()
{
	QVector<int> selectIDs;

	foreach(CModel *m, m_modelList)
	{
		if (m->isPicked())
		{
			selectIDs.push_back(m->getID());
		}
	}

	return selectIDs;
}

void CScene::buildSupportHierarchy()
{
	// build on top list from loaded support info
	m_onTopList.resize(m_modelList.size(), -1);
	
	std::vector<std::vector<int>> SuppList(m_RG.Size());	// support giver list, models that are being supported
	for (unsigned int ei = 0; ei < m_RG.ESize(); ei++) {
		if (m_RG.GetEdge(ei)->t == CT_SUPPORT) {
			CModel *pM1 = m_modelList[m_RG.GetEdge(ei)->v1];
			CModel *pM2 = m_modelList[m_RG.GetEdge(ei)->v2];

			//if pM1 is higher than pM2, then pM2 is in support list of pM1
			if (pM1->m_GOBB.BottomHeightDiff(pM2->m_GOBB, m_uprightVec) > 0.0) {
				SuppList[m_RG.GetEdge(ei)->v1].push_back(m_RG.GetEdge(ei)->v2);
			}
			else {
				SuppList[m_RG.GetEdge(ei)->v2].push_back(m_RG.GetEdge(ei)->v1);
			}
		}
	}

	for (unsigned int i = 0; i < SuppList.size(); i++)
	{
		CModel *pM1 = m_modelList[i];
		for (int j = 0; j < SuppList[i].size(); j++)
		{
			CModel *pM2 = m_modelList[SuppList[i][j]];

			// use a relative large support threshold for conservative pruning
			if (pM1->IsSupport(pM2, true, 10 * m_SuppThresh, m_uprightVec))
			{
				m_onTopList[i] = SuppList[i][j];
			}
		}
	}

	// collect parent-child relationship
	for (int i = 0; i < m_onTopList.size(); i++)
	{
		int parentID = m_onTopList[i];
		if (parentID != -1)
		{
			m_modelList[i]->suppParentID = parentID;
			m_modelList[parentID]->suppChindrenList.push_back(i);
		}
	}

	// set support level
	double minHeight = 1e6;
	std::vector<double> modelBottomHeightList(m_modelNum);

	// find models that are support by the floor
	for (int i = 0; i < m_modelNum; i++)
	{
		double currHeight = m_modelList[i]->getOBBBottomHeight(m_uprightVec);
		modelBottomHeightList[i] = currHeight;

		if (currHeight < minHeight)
		{
			minHeight = currHeight;
		}

		m_modelList[i]->supportLevel = -1;
	}

	// recursively set children's level
	for (int i = 0; i < m_modelNum; i++)
	{
		if (modelBottomHeightList[i]-minHeight < 0.1)
		{
			m_modelList[i]->supportLevel = 0;

			CModel *currModel = m_modelList[i];
			
			setChildrenSupportLevel(currModel); 			
		}
	}

	// if support level is still not set, it may be hang on the wall
	// set support level of these models to be 1 and recursively set children's level
	for (int i = 0; i < m_modelNum; i++)
	{
		if (m_modelList[i]->supportLevel == -1)
		{
			m_modelList[i]->supportLevel = 1;

			setChildrenSupportLevel(m_modelList[i]);
		}
	}

	m_hasSupportHierarchy = true;
}

void CScene::setChildrenSupportLevel(CModel *m)
{
	if (m->suppChindrenList.size() == 0)
	{
		return;
	}

	else
	{
		for (int i = 0; i < m->suppChindrenList.size(); i++)
		{
			m_modelList[m->suppChindrenList[i]]->supportLevel = m->supportLevel + 1;

			// set children's support level recursively
			setChildrenSupportLevel(m_modelList[m->suppChindrenList[i]]);
		}
	}
}

void CScene::voxelizeModels()
{
	foreach(CModel *m, m_modelList)
	{
		if (!m->isVoxelized())
		{
			m->voxelize();
		}
	}
}

bool CScene::isInsideModel(SurfaceMesh::Vector3 &point, int modelID)
{
	bool isInsideModel = m_modelList[modelID]->isPointInside(point);
	return isInsideModel;
}

bool CScene::isIntersectModel(SurfaceMesh::Vector3 &startPt, SurfaceMesh::Vector3 &endPt, int modelID)
{
	SurfaceMesh::Vector3 tempEndPt = endPt;

	if (startPt == endPt)
	{
		tempEndPt = startPt + 0.05*SurfaceMesh::Vector3(0,0,1);
	}

	return m_modelList[modelID]->isSegmentIntersect(startPt, tempEndPt);
}

std::vector<double> CScene::getFloorXYRange()
{
	int floorID = getModelIdByName("floor");
	return m_modelList[floorID]->getAABBXYRange();
}

/*
int CScene::doSymmetryConnection(ConnType ct, bool bCompSCG)
{
	ConnType gct;	// group conn. type
	switch (ct) {
	case CT_PROXIMITY:
		gct = CT_PROX_SYM;
		break;
	case CT_SUPPORT:
		gct = CT_SUP_SYM;
		break;
	case CT_CONTACT:
		gct = CT_CONTACT_SYM;
		break;
	case CT_WEAK:
		gct = CT_WEAK_SYM;
		break;
	default:
		return -1;
		break;
	}
	std::vector<CSceneRG::SCGrp> &SG = m_RG.m_SCGrp;
	std::vector<int> CEL; // list of models connected to a symmetry group
	CUDGraph InsG(m_RG.Size());//, DelG(m_RG.Size());
	for (unsigned int i = 0; i < m_SymGrp.size(); i++) {
		// Find all edges connected to the current i'th symmetry group
		m_RG.GetAllNeigborEdgeList(m_SymGrp[i], ct, CEL);
		std::vector<short> EPM(m_RG.ESize(), 0);	// edge processed tag map
		// For each edge, check if all other members in the current sym. grp. can be connected to it too
		for (unsigned int k = 0; k < CEL.size(); k++) {
			if (EPM[CEL[k]] == 1) { continue; }	// avoid duplicated processing
			CUDGraph::Edge &CE = *m_RG.GetEdge(CEL[k]);
			int iMO = (m_SymMap[CE[0]] == i) ? CE[1] : CE[0]; // the id of the model that is connected to the group
			int iMG = (iMO == CE[0]) ? CE[1] : CE[0]; // the id of the model that is inside the group
			std::vector<int> SymConnL;
			SymConnL.push_back(iMG);	// make sure the first member in the list is always the originally connected one
			double dt = 0.4 * m_modelList[iMO]->m_GOBB.dl;		// distance-diff threshold
			double d0 = m_modelDistMat.Get(iMG, iMO);
			bool bSetGrp = false;
			MathLib::Vector3 dv(0, 0, 0);
			for (unsigned int j = 0; j < m_SymGrp[i].size(); j++) {
				if (m_SymGrp[i][j] == iMG) { continue; }	// check other group members than iMG
				int iEId = m_RG.GetEdgeId(m_SymGrp[i][j], iMO);
				if (iEId == -1 && ct == CT_SUPPORT) { continue; }		// I do not introduce a new support edge
				int iET = m_RG.GetEdgeTag(iEId);
				if (iEId != -1 && iET != ct && iET != gct) { EPM[iEId] = 1; continue; }	// I do not modify the type of an existing edge
				double d = m_modelDistMat.Get(m_SymGrp[i][j], iMO);
				if (Abs(d - d0) < dt) {	// similar in closest distance
					if (iEId == -1) {	// not exist
						InsG.InsertEdge(m_SymGrp[i][j], iMO, gct);	// temporarily insert an edge with x_sym type
					}
					else {			// exist
						m_RG.SetEdgeTag(iEId, gct);	// set edge type to x_sym
						EPM[iEId] = 1;	// set processed
					}
					SymConnL.push_back(m_SymGrp[i][j]);
					dv += m_modelList[m_SymGrp[i][j]]->m_GOBB.cent - m_modelList[iMO]->m_GOBB.cent;
					bSetGrp = true;
				}
			}
			dv += dv += m_modelList[iMG]->m_GOBB.cent - m_modelList[iMO]->m_GOBB.cent;
			if (bSetGrp) {	// set the edge from group member iMG to iMO as x_sym
				m_RG.SetEdgeTag(iMG, iMO, gct);
				// Break all connections/edges between any two models from SymConnL
				//				std::vector<int> EL;
				//				m_RG.GetEL(SymConnL, EL);
				//				for (unsigned int ei=0; ei<EL.size(); ei++) { DelG.InsertEdge(m_RG.GetE(EL[ei])->v1,m_RG.GetE(EL[ei])->v2); }
				// Set sym. conn. grp.
				if (bCompSCG) {
					SG.push_back(CSceneRG::SCGrp(iMO, gct));
					SG.back().ml = SymConnL;	// save member vertices
					SG.back().w = dv.magnitude();
				}
			}
			EPM[CEL[k]] = 1;	// set processed
		}
	}
	// Really inserting and deleting
	for (unsigned int ei = 0; ei < InsG.ESize(); ei++) {
		m_RG.InsertEdge(InsG.GetEdge(ei)->v1, InsG.GetEdge(ei)->v2, InsG.GetEdge(ei)->t);
	}
	//	for (unsigned int ei=0; ei<DelG.ESize(); ei++) {
	//		m_RG.DeleteEdge(DelG.GetE(ei)->v1, DelG.GetE(ei)->v2);
	//	}
	return 0;
}
*/
