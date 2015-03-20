#pragma once

#include "Voxel.h"

#include "NanoKdTree.h"

#define glv(v) glVertex3dv(v.data())
#define gln glNormal3d

class VoxelOctree;

namespace VoxelerLibrary{

class Voxeler
{
private:
    SurfaceMesh::Model * mesh;
    NanoKdTree kd;
	Vector3VertexProperty points;

	// Special voxels
    NanoKdTree outerVoxels, innerVoxels;

public:
    Voxeler( SurfaceMesh::Model * src_mesh, double voxel_size, bool verbose = false);

	FaceBounds findFaceBounds( SurfaceMesh::Model::Face f );
	bool isVoxelIntersects( const Voxel & v, Surface_mesh::Face f );
	
	void update();
	void computeBounds();

	// Grow larger by one voxel
	void grow();

	// Find inside and outside of mesh surface
	std::vector< Voxel > fillOther();
    std::vector< Voxel > fillInside();
    void fillOuter(NanoKdTree & outside);

	// Intersection
	std::vector<Voxel> Intersects(Voxeler * other);
	std::map<int, Voxel> around(SurfaceMesh::Point p);
	bool isIntersectWithPoint(SurfaceMesh::Point p);
	bool isIntersectSegment(const SurfaceMesh::Vec3d &startPt, const SurfaceMesh::Vec3d &endPt);

	// Visualization:
	void draw();
	void setupDraw();
	static void drawVoxels( const std::vector< Voxel > & voxels, double voxel_size = 1.0);

    NanoKdTree corner_kd;
	std::vector< SurfaceMesh::Point > corners;
	std::vector< std::vector<int> > cornerIndices;
	std::vector< int > cornerCorrespond;
	std::vector< SurfaceMesh::Point > getCorners(int vid);
	int getClosestVoxel(Vector3d point);
	int getEnclosingVoxel( Vector3d point );

	std::vector< SurfaceMesh::Point > getVoxelCenters();

	std::vector< Voxel > voxels;
	int getVoxelIndex(Voxel v);
	uint d1, d2;
	bool isVerbose;
	bool isReadyDraw;

	double voxelSize;

	Voxel minVox;
	Voxel maxVox;

public:
	void buildOctree();
	void drawOctree();

	VoxelOctree *m_voxelOctree;
	bool m_isBuiltOctree;
};

}
