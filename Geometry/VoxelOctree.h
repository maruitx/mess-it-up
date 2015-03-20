#pragma once

#include <cmath>
#include <set>
#include <stack>
#include <vector>
#include <list>
#include <QSet>

#include "BoundingBox.h"

#include "Voxeler.h"

class VoxelOctree
{
public:
	VoxelOctree();
	~VoxelOctree();

	VoxelOctree(int voxPerNode, VoxelerLibrary::Voxeler *voxeler);
	
	void initBuild();

	void newNode(int depth, double x, double y, double z);
	void build(int depth = 0);

	void draw(double r, double g, double b, double lineWidth = 1.0);
	void DrawBox(const Vec3d& center, float width, float length, float height, float r, float g, float b, float lineWidth);

	bool intersectSegment(const Vec3d &startPt, const Vec3d &endPt);

public:
	VoxelerLibrary::Voxeler *m_voxeler;
	std::vector<VoxelOctree> m_children;
	VoxelOctree *m_parent;

	BoundingBox m_boundingBox;
	int m_voxelPerNode;

	std::vector<BoundingBox> m_voxelBoundingBoxData;


};

