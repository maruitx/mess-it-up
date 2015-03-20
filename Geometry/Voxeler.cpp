#include "qgl.h"
#include "Voxeler.h"
#include "VoxelOctree.h"

using namespace VoxelerLibrary;

#include "BoundingBox.h"
#include <stack>

#include "voxel_weld.h"

Voxeler::Voxeler( SurfaceMesh::Model * src_mesh, double voxel_size, bool verbose /*= false*/ )
{
	this->mesh = src_mesh;

    points = mesh->vertex_property<SurfaceMesh::Point>(SurfaceMesh::VPOINT);

	this->voxelSize = voxel_size;
	this->isVerbose = verbose;
	this->isReadyDraw = false;

	if(mesh == NULL)
		return;

	if(isVerbose) qDebug() << "Computing voxels..";

	// For each face in mesh
	foreach(Surface_mesh::Face f, mesh->faces())
	{
		FaceBounds fb = findFaceBounds( f );

		for(int x = fb.minX; x <= fb.maxX; x++)
		{
			for(int y = fb.minY; y <= fb.maxY; y++)
			{
				for(int z = fb.minZ; z <= fb.maxZ; z++)
				{
					Voxel v(x,y,z);

					if(isVoxelIntersects(v, f))
						voxels.push_back( v );
				}
			}
		}
	}
	
	// Combine into a set of voxels
	std::vector<size_t> xrefs;
    weld(voxels, xrefs, std::hash_VoxelerLibraryVoxel(), std::equal_to<Voxel>());

	// Add voxels to KD-tree and build
	foreach(Voxel v, voxels) 
		kd.addPoint(v);
	kd.build();

	computeBounds();

	buildOctree();

	if(isVerbose) qDebug() << "Voxel count = " << (int)voxels.size();
}

void Voxeler::update()
{
	// Compute bounds
	computeBounds();

	// Setup visualization
	setupDraw();
}

void Voxeler::computeBounds()
{
	minVox = Voxel(INT_MAX, INT_MAX, INT_MAX);
	maxVox = Voxel(-INT_MAX, -INT_MAX, -INT_MAX);

	for(int i = 0; i < (int)voxels.size(); i++)
	{
		Voxel v = voxels[i];

		minVox.toMin(v);
		maxVox.toMax(v);
	}
}

FaceBounds Voxeler::findFaceBounds( Surface_mesh::Face f )
{
	FaceBounds fb;

	double minx = 0, miny = 0, minz = 0;
	double maxx = 0, maxy = 0, maxz = 0;

	// Collect points
	QVector<SurfaceMesh::Vector3> f_vec; 
	Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
	do{ f_vec.push_back(points[vit]); } while(++vit != vend);

	minx = maxx = f_vec[0].x();
	miny = maxy = f_vec[0].y();
	minz = maxz = f_vec[0].z();

	for(int v = 0; v < 3; v++)
	{
		Vector3d vec = f_vec[v];

		if (vec.x() < minx) minx = vec.x();
		if (vec.x() > maxx) maxx = vec.x();

		if (vec.y() < miny) miny = vec.y();
		if (vec.y() > maxy) maxy = vec.y();

		if (vec.z() < minz) minz = vec.z();
		if (vec.z() > maxz) maxz = vec.z();
	}

	fb.minX = floor(minx / voxelSize);
	fb.minY = floor(miny / voxelSize);
	fb.minZ = floor(minz / voxelSize);

	fb.maxX = ceil(maxx / voxelSize);
	fb.maxY = ceil(maxy / voxelSize);
	fb.maxZ = ceil(maxz / voxelSize);

	return fb;
}

bool Voxeler::isVoxelIntersects( const Voxel& v, Surface_mesh::Face f )
{
	Vector3d center = Vector3d(v.x * voxelSize, v.y * voxelSize, v.z * voxelSize);

	double s = voxelSize * 0.5;

	BoundingBox b(center, s,s,s);

	// Collect points
	QVector<SurfaceMesh::Vector3> f_vec; 
	Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
	do{ f_vec.push_back(points[vit]); } while(++vit != vend);

	return b.containsTriangle(f_vec[0], f_vec[1], f_vec[2]);
}

void Voxeler::draw()
{
	if(!isReadyDraw)
		update();

	glEnable(GL_LIGHTING);
	glShadeModel(GL_FLAT);

	glColor3f(0, 0.8f, 0);
	//glCallList(d1);

	glDisable(GL_LIGHTING);
	glShadeModel(GL_SMOOTH);

	glColor3f(0, 1, 0);
	glLineWidth(1.0);
	glCallList(d2);

	/*for(int i = 0; i < (int) temp2.size(); i++){
		Vector3d c = temp2[i];
		c *= voxelSize;
		if(temp2[i].x() < 0)
			SimpleDraw::DrawSolidBox(c, voxelSize, voxelSize, voxelSize, 1,0,0);
	}*/

	glEnable(GL_LIGHTING);
}

void Voxeler::setupDraw()
{
	double s = voxelSize * 0.5;
	int n = (int)voxels.size();
	
	std::vector< std::vector<Vector3d> > corner(n, std::vector<Vector3d>(8));

	// Find corners
	for(int i = 0; i < n; i++)
	{
		Vector3d c = voxels[i];	c *= voxelSize;
		corner[i][0] = Vector3d(s, s, s) + c;		corner[i][1] = Vector3d(-s, s, s) + c;
		corner[i][2] = Vector3d(-s, -s, s) + c;	corner[i][3] = Vector3d(s, -s, s) + c;
		corner[i][4] = Vector3d(s, s, -s) + c;		corner[i][5] = Vector3d(-s, s, -s) + c;
		corner[i][6] = Vector3d(-s, -s, -s) + c;	corner[i][7] = Vector3d(s, -s, -s) + c;
	}

	// Save corners
	corners.clear();
	cornerIndices.clear();
	cornerCorrespond.resize(n*8);

	// Build corner_kd
	std::vector<Vector3d> cornerPnts;
	for(int i = 0; i < n; i++)
		for(int j = 0; j < 8; j++)
			cornerPnts.push_back(corner[i][j]);

	std::vector<size_t> xrefs;
    weld(cornerPnts, xrefs, std::hash_Vector3d(), std::equal_to<Vector3d>());

	foreach(Vector3d p, cornerPnts) corner_kd.addPoint(p);
	corner_kd.build();

	for(int i = 0; i < n; i++)
	{
		std::vector<int> p(8, 0);

		for(int j = 0; j < 8; j++)
		{
            p[j] = corner_kd.closest(corner[i][j]);
			cornerCorrespond[p[j]] = i; // will belong to last one
		}

		cornerIndices.push_back(p);
	}

	d1 = glGenLists(1);

	// Faces
	glNewList(d1, GL_COMPILE);
	glBegin(GL_QUADS);
	for(int i = 0; i < (int)voxels.size(); i++)
	{
		// top, left, right, bottom
		gln(0,0,1); glv(corner[i][0]); glv(corner[i][1]); glv(corner[i][2]); glv(corner[i][3]);
		gln(0,1,0); glv(corner[i][0]); glv(corner[i][1]); glv(corner[i][5]); glv(corner[i][4]);
		gln(0,-1,0); glv(corner[i][2]); glv(corner[i][3]); glv(corner[i][7]); glv(corner[i][6]);
		gln(0,0,-1); glv(corner[i][4]); glv(corner[i][5]); glv(corner[i][6]); glv(corner[i][7]);

		// front, back
		gln(1,0,0); glv(corner[i][0]); glv(corner[i][3]); glv(corner[i][7]); glv(corner[i][4]);
		gln(-1,0,0); glv(corner[i][1]); glv(corner[i][2]); glv(corner[i][6]); glv(corner[i][5]);
	}
	glEnd();
	glEndList();

	d2 = glGenLists(1);

	// Lines
	glNewList(d2, GL_COMPILE);
	glBegin(GL_LINES);
	for(int i = 0; i < (int)voxels.size(); i++)
	{
		glv(corner[i][0]);glv(corner[i][4]);glv(corner[i][1]);glv(corner[i][5]);
		glv(corner[i][2]);glv(corner[i][6]);glv(corner[i][3]);glv(corner[i][7]);
		glv(corner[i][0]);glv(corner[i][1]);glv(corner[i][2]);glv(corner[i][3]);
		glv(corner[i][0]);glv(corner[i][3]);glv(corner[i][1]);glv(corner[i][2]);
		glv(corner[i][4]);glv(corner[i][5]);glv(corner[i][6]);glv(corner[i][7]);
		glv(corner[i][4]);glv(corner[i][7]);glv(corner[i][5]);glv(corner[i][6]);
	}
	glEnd();
	glEndList();

	isReadyDraw = true;
}

void Voxeler::drawVoxels( const std::vector< Voxel > & voxels, double voxel_size )
{
	double s = voxel_size * 0.5;
	int n = (int)voxels.size();

	std::vector<Vector3d> c1(n), c2(n), c3(n), c4(n);
	std::vector<Vector3d> bc1(n), bc2(n), bc3(n), bc4(n);

	// Find corners
	for(int i = 0; i < (int)voxels.size(); i++){
		Vector3d c = voxels[i];	c *= voxel_size;
		c1[i] = Vector3d(s, s, s) + c; c2[i] = Vector3d(-s, s, s) + c;
		c3[i] = Vector3d(-s, -s, s) + c; c4[i] = Vector3d(s, -s, s) + c;
		bc1[i] = Vector3d(s, s, -s) + c; bc2[i] = Vector3d(-s, s, -s) + c;
		bc3[i] = Vector3d(-s, -s, -s) + c; bc4[i] = Vector3d(s, -s, -s) + c;
	}

	glColor3d(1,0,0);
	glLineWidth(3.0f);
	glDisable(GL_LIGHTING);

	// Lines
	glBegin(GL_LINES);
	for(int i = 0; i < (int)voxels.size(); i++){
		glv(c1[i]);glv(bc1[i]);glv(c2[i]);glv(bc2[i]);
		glv(c3[i]);glv(bc3[i]);glv(c4[i]);glv(bc4[i]);
		glv(c1[i]);glv(c2[i]);glv(c3[i]);glv(c4[i]);
		glv(c1[i]);glv(c4[i]);glv(c2[i]);glv(c3[i]);
		glv(bc1[i]);glv(bc2[i]);glv(bc3[i]);glv(bc4[i]);
		glv(bc1[i]);glv(bc4[i]);glv(bc2[i]);glv(bc3[i]);
	}
	glEnd();

	glEnable(GL_LIGHTING);
}

std::vector<Voxel> Voxeler::fillOther()
{
	std::vector<Voxel> filled;

	for(int x = minVox.x - 1; x <= maxVox.x + 1; x++){
		for(int y = minVox.y - 1; y <= maxVox.y + 1; y++){
			for(int z = minVox.z - 1; z <= maxVox.z + 1; z++){
                SurfaceMesh::Vector3 v(x,y,z);
                if(!kd.has(v))
					filled.push_back(Voxel(x,y,z));
			}
		}
	}

	return filled;
}

std::vector<Voxel> Voxeler::fillInside()
{
	printf("Computing inside, outside..");

	std::vector<Voxel> innerVoxels;

	NanoKdTree outside;
	fillOuter(outside);

	// Compute inner as complement of outside
	for(int x = minVox.x - 1; x <= maxVox.x + 1; x++){
		for(int y = minVox.y - 1; y <= maxVox.y + 1; y++){
			for(int z = minVox.z - 1; z <= maxVox.z + 1; z++){
                SurfaceMesh::Vector3 v(x,y,z);
                if(!outside.has(v)){
					innerVoxels.push_back(Voxel(x,y,z));
				}
			}
		}
	}

	return innerVoxels;
}

void Voxeler::fillOuter(NanoKdTree & outside)
{
	std::stack<Voxel> stack;

	stack.push(maxVox + Voxel(1,1,1));

	std::vector<Voxel> outterVoxels;

	QSet<QString> seenSet;

	qDebug() << "Filling outside..";

	while(!stack.empty())
	{
		// Get next square
		Voxel c = stack.top(); // get current voxel
		stack.pop();

		// Bad: using strings for now
		QString cellString = QString("%1,%2,%3").arg(c.x).arg(c.y).arg(c.z);

		SurfaceMesh::Vector3 p(c.x, c.y, c.z);

		// Base case:
		if( !kd.has(p) && !seenSet.contains(cellString) )
		{
			seenSet.insert( cellString );

			outside.addPoint(p);

			// Visit neighbors
			if(c.x < maxVox.x + 1) stack.push( c + Voxel( 1, 0, 0) );
			if(c.y < maxVox.y + 1) stack.push( c + Voxel( 0, 1, 0) );
			if(c.z < maxVox.z + 1) stack.push( c + Voxel( 0, 0, 1) );

			if(c.x > minVox.x - 1) stack.push( c + Voxel(-1, 0, 0) );
			if(c.y > minVox.y - 1) stack.push( c + Voxel( 0,-1, 0) );
			if(c.z > minVox.z - 1) stack.push( c + Voxel( 0, 0,-1) );
		}
	}

	outside.build();

	qDebug() << "Outside voxels filled!";
}

std::vector<Voxel> Voxeler::Intersects(Voxeler * other)
{
	std::vector<Voxel> intersection;

	Voxeler *minVoxeler = this, *maxVoxeler = other;

	// Swap with minimum
	if(other->voxels.size() < this->voxels.size()){
		minVoxeler = other;
		maxVoxeler = this;
	}

	for(int i = 0; i < (int) minVoxeler->voxels.size(); i++)
	{
		Voxel v = minVoxeler->voxels[i];

        SurfaceMesh::Vector3 vv(v.x, v.y, v.z);
        if(maxVoxeler->kd.has(vv))
			intersection.push_back(v);
	}

	return intersection;
}

std::map<int, Voxel> Voxeler::around(SurfaceMesh::Point p)
{
	std::map<int, Voxel> result;

	int x = p.x() / voxelSize;
	int y = p.y() / voxelSize;
	int z = p.z() / voxelSize;

	for(int i = -1; i <= 1; i += 1){
		for(int j = -1; j <= 1; j += 1){
			for(int k = -1; k <= 1; k += 1){
				Voxel v(x + i, y + j, z + k);

				Vector3d vpos(v.x, v.y, v.z);

                SurfaceMesh::Vector3 vv(v.x, v.y, v.z);

                if(kd.has(vv)){
					int idx = kd.closest(vpos);
					result[idx] = v;
				}
			}
		}
	}

	return result;
}

void Voxeler::grow()
{
	int N = (int)voxels.size();

	std::vector<Voxel> newVox;

	for(int i = 0; i < N; i++)
	{
		Voxel curVoxel = voxels[i];

		for(int x = -1; x <= 1; x += 1){
			for(int y = -1; y <= 1; y++){
				for(int z = -1; z <= 1; z++){
					Voxel v(curVoxel.x + x, curVoxel.y + y, curVoxel.z + z);
					newVox.push_back(v);
				}
			}
		}
	}

	// Combine into set of voxels
	std::vector<size_t> xrefs;
    weld(voxels, xrefs, std::hash_VoxelerLibraryVoxel(), std::equal_to<Voxel>());

	// Clear old, add new points and build
	kd.cloud.pts.clear();
	foreach(Voxel v, voxels) kd.addPoint(v);
	kd.build();

	printf("\nVoxler grown from (%d) to (%d).\n", N, (int)voxels.size());

	isReadyDraw = false;
}

std::vector< SurfaceMesh::Point > Voxeler::getCorners( int vid )
{
	std::vector< SurfaceMesh::Point > result;

	for(int i = 0; i < 8; i++)
		result.push_back(corners[cornerIndices[vid][i]]);

	return result;
}

int Voxeler::getClosestVoxel( Vector3d point )
{
	return cornerCorrespond[ corner_kd.closest(point) ];
}

int Voxeler::getEnclosingVoxel( Vector3d point )
{
	int N = (int)voxels.size();

	double s = voxelSize * 0.5;

	double minDist = DBL_MAX;
	int closestVoxel = -1;

	for(int i = 0; i < N; i++)
	{
		Voxel curVoxel = voxels[i];

		SurfaceMesh::Point voxelCenter(curVoxel.x * s, curVoxel.y * s, curVoxel.z * s);

		double curDist = (voxelCenter - point).norm();

		if(curDist < minDist){
			closestVoxel = i;
			minDist = curDist;
		}
	}

	return closestVoxel;
}

int Voxeler::getVoxelIndex( Voxel v )
{
	for(int i = 0; i < (int) voxels.size(); i++)
	{
		Voxel w = voxels[i];

		if(w.x == v.x && w.y == v.y && w.z == v.z)
			return i;
	}

	return -1;
}

std::vector< SurfaceMesh::Point > Voxeler::getVoxelCenters()
{
	std::vector< SurfaceMesh::Point > pnts;

	for(int i = 0; i < (int) voxels.size(); i++){
		Voxel w = voxels[i];
		pnts.push_back(SurfaceMesh::Point(w.x * voxelSize, w.y* voxelSize, w.z* voxelSize) );
	}

	return pnts;
}

bool VoxelerLibrary::Voxeler::isIntersectWithPoint(SurfaceMesh::Point p)
{
	// to do: add intersection test

	return true;
}

void VoxelerLibrary::Voxeler::buildOctree()
{
	int numVoxelPerNode = 10;

	m_voxelOctree = new VoxelOctree(10, this);

	m_isBuiltOctree = true;
}

void VoxelerLibrary::Voxeler::drawOctree()
{
	m_voxelOctree->draw(0, 0, 1);
}

bool VoxelerLibrary::Voxeler::isIntersectSegment(const SurfaceMesh::Vec3d &startPt, const SurfaceMesh::Vec3d &endPt)
{
	return m_voxelOctree->intersectSegment(startPt, endPt);
}
