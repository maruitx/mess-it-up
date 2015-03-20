#include "qgl.h"
#include "VoxelOctree.h"

VoxelOctree::VoxelOctree()
{
}

VoxelOctree::VoxelOctree(int voxPerNode, VoxelerLibrary::Voxeler *voxeler)
{
	m_voxelPerNode = voxPerNode;
	m_voxeler = voxeler;

	// convert voxeler to bounding boxes
	int voxelNum = m_voxeler->voxels.size();
	m_voxelBoundingBoxData.resize(voxelNum);

	for (int i = 0; i < voxelNum; i++)
	{
		double voxelSize = m_voxeler->voxelSize;
		Vector3d c = m_voxeler->voxels[i];	c *= voxelSize;

		BoundingBox bb(c, 0.5*voxelSize, 0.5*voxelSize, 0.5*voxelSize);

		m_voxelBoundingBoxData[i] = bb;
	}

	// Create a big box
	BoundingBox bb;
	bb.computeFromBoundingBoxes(m_voxelBoundingBoxData);

	// Transform and scale to node's coordinates
	double largeSize = qMax(bb.xExtent, qMax(bb.yExtent, bb.zExtent));

	largeSize *= 1.1;

	// Define our bounding box
	this->m_boundingBox = BoundingBox(bb.center, largeSize, largeSize, largeSize);

	// Build the tree
	this->build();

	// Connect children with parent
	std::stack<VoxelOctree*> childStack;
	childStack.push(this);
	while (!childStack.empty())
	{
		VoxelOctree * curr = childStack.top(); childStack.pop();

		for (int i = 0; i < (int)curr->m_children.size(); i++)
		{
			curr->m_children[i].m_parent = curr;

			childStack.push(&curr->m_children[i]);
		}
	}
}

VoxelOctree::~VoxelOctree()
{
}

void VoxelOctree::initBuild()
{

}

void VoxelOctree::newNode(int depth, double x, double y, double z)
{
	double newExtent = m_boundingBox.xExtent / 2.0;

	Vec3d center;

	center.x() = m_boundingBox.center.x() + (newExtent * x);
	center.y() = m_boundingBox.center.y() + (newExtent * y);
	center.z() = m_boundingBox.center.z() + (newExtent * z);

	BoundingBox bb(center, newExtent, newExtent, newExtent);

	// Add child
	m_children.push_back(VoxelOctree());
	VoxelOctree * child = &m_children.back();

	child->m_boundingBox = bb;
	child->m_voxelPerNode = m_voxelPerNode;

	// Collect voxels inside child's bounding box
	for (int i = 0; i < m_voxelBoundingBoxData.size(); i++)
	{
		if (bb.containsBoundingBox(m_voxelBoundingBoxData[i]))
		{
			child->m_voxelBoundingBoxData.push_back(m_voxelBoundingBoxData[i]);
		}
	}

	// build child
	child->build(depth + 1);
}

void VoxelOctree::build(int depth /*= 0*/)
{
	if (m_voxelBoundingBoxData.size() > m_voxelPerNode)
	{
		if (depth < 8)
		{
			// Subdivide to 8 nodes
			newNode(depth, -1, -1, -1);
			newNode(depth, 1, -1, -1);
			newNode(depth, -1, 1, -1);
			newNode(depth, 1, 1, -1);
			newNode(depth, -1, -1, 1);
			newNode(depth, 1, -1, 1);
			newNode(depth, -1, 1, 1);
			newNode(depth, 1, 1, 1);
		}
	}
}

void VoxelOctree::draw(double r, double g, double b, double lineWidth /*= 1.0*/)
{
	DrawBox(m_boundingBox.center, m_boundingBox.xExtent, m_boundingBox.yExtent, m_boundingBox.zExtent, r, g, b, lineWidth);

	for (std::vector<VoxelOctree>::iterator child = m_children.begin(); child != m_children.end(); child++)
		child->draw(r, g, b, lineWidth);
}

void VoxelOctree::DrawBox(const Vec3d& center, float width, float length, float height, float r, float g, float b, float lineWidth)
{
	Vec3d  c1, c2, c3, c4;
	Vec3d  bc1, bc2, bc3, bc4;

	c1 = Vec3d(width, length, height) + center;
	c2 = Vec3d(-width, length, height) + center;
	c3 = Vec3d(-width, -length, height) + center;
	c4 = Vec3d(width, -length, height) + center;

	bc1 = Vec3d(width, length, -height) + center;
	bc2 = Vec3d(-width, length, -height) + center;
	bc3 = Vec3d(-width, -length, -height) + center;
	bc4 = Vec3d(width, -length, -height) + center;

	glDisable(GL_LIGHTING);

	glColor3f(r, g, b);
	glLineWidth(lineWidth);

	glBegin(GL_LINES);
	glVertex3dv(c1.data()); glVertex3dv(bc1.data());
	glVertex3dv(c2.data()); glVertex3dv(bc2.data());
	glVertex3dv(c3.data()); glVertex3dv(bc3.data());
	glVertex3dv(c4.data()); glVertex3dv(bc4.data());
	glVertex3dv(c1.data()); glVertex3dv(c2.data());
	glVertex3dv(c3.data()); glVertex3dv(c4.data());
	glVertex3dv(c1.data()); glVertex3dv(c4.data());
	glVertex3dv(c2.data()); glVertex3dv(c3.data());
	glVertex3dv(bc1.data()); glVertex3dv(bc2.data());
	glVertex3dv(bc3.data()); glVertex3dv(bc4.data());
	glVertex3dv(bc1.data()); glVertex3dv(bc4.data());
	glVertex3dv(bc2.data()); glVertex3dv(bc3.data());
	glEnd();
	glEnable(GL_LIGHTING);
}

bool VoxelOctree::intersectSegment(const Vec3d &startPt, const Vec3d &endPt)
{
	if (this->m_boundingBox.intersectSegment(startPt, endPt))
	{
		std::stack<VoxelOctree*> s;
		s.push(this);

		while (!s.empty())
		{
			VoxelOctree * curTree = s.top();
			s.pop();

			if (curTree->m_children.size() == 0)
			{
				for (int bid = 0; bid < m_voxelBoundingBoxData.size(); bid++)
				{
					if (m_voxelBoundingBoxData[bid].intersectSegment(startPt, endPt))
					{
						return true;
					}
				}
			}

			// Do following if child size > 0
			for (std::vector<VoxelOctree>::iterator child = curTree->m_children.begin(); child != curTree->m_children.end(); child++)
			{
				if (child->m_boundingBox.intersectSegment(startPt, endPt))
				{
					s.push(&(*child));
				}
			}
		}
	}

	return false;
}
