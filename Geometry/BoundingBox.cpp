#include "BoundingBox.h"

BoundingBox::BoundingBox()
{
	this->center = Vector3d(0,0,0);

	this->xExtent = 0;
	this->yExtent = 0;
	this->zExtent = 0;

	axis[0] = Vector3d(1, 0, 0);
	axis[1] = Vector3d(0, 1, 0);
	axis[2] = Vector3d(0, 0, 1);
}

BoundingBox::BoundingBox( const Vector3d& c, double x, double y, double z )
{
	this->center = c;

	this->xExtent = x;
	this->yExtent = y;
	this->zExtent = z;

	Vector3d corner(x/2, y/2, z/2);

	vmin = center - corner;
	vmax = center + corner;

	axis[0] = Vector3d(1, 0, 0);
	axis[1] = Vector3d(0, 1, 0);
	axis[2] = Vector3d(0, 0, 1);
}

BoundingBox& BoundingBox::operator=( const BoundingBox& other )
{
	this->center = other.center;

	this->xExtent = other.xExtent;
	this->yExtent = other.yExtent;
	this->zExtent = other.zExtent;

	this->vmax = other.vmax;
	this->vmin = other.vmin;

	this->axis[0] = other.axis[0];
	this->axis[1] = other.axis[1];
	this->axis[2] = other.axis[2];

	return *this;
}

/*void BoundingBox::computeFromTris( const StdVector<BaseTriangle*>& tris )
{
	vmin = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	vmax = Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);

	double minx = 0, miny = 0, minz = 0;
	double maxx = 0, maxy = 0, maxz = 0;

	minx = maxx = tris[0]->vec(0).x();
	miny = maxy = tris[0]->vec(0).y();
	minz = maxz = tris[0]->vec(0).z();

	for (int i = 0; i < (int)tris.size(); i++)
	{
		for(int v = 0; v < 3; v++)
		{
			Vector3d vec = tris[i]->vec(v);

			if (vec.x() < minx) minx = vec.x();
			if (vec.x() > maxx) maxx = vec.x();
			if (vec.y() < miny) miny = vec.y();
			if (vec.y() > maxy) maxy = vec.y();
			if (vec.z() < minz) minz = vec.z();
			if (vec.z() > maxz) maxz = vec.z();
		}
	}

	vmax = Vector3d(maxx, maxy, maxz);
	vmin = Vector3d(minx, miny, minz);

	this->center = (vmin + vmax) / 2.0;

	this->xExtent = abs(vmax.x() - center.x());
	this->yExtent = abs(vmax.y() - center.y());
	this->zExtent = abs(vmax.z() - center.z());
}*/

/*bool BoundingBox::intersects( const Ray& ray ) const
{
    double rhs;
    double fWdU[3];
    double fAWdU[3];
    double fDdU[3];
    double fADdU[3];
    double fAWxDdU[3];

    Vector3d UNIT_X(1.0, 0.0, 0.0);
    Vector3d UNIT_Y(0.0, 1.0, 0.0);
    Vector3d UNIT_Z(0.0, 0.0, 1.0);

    Vector3d diff = ray.origin - center;
    Vector3d wCrossD = cross(ray.direction , diff);

    fWdU[0] = dot(ray.direction , UNIT_X);
    fAWdU[0] = abs(fWdU[0]);
    fDdU[0] = dot(diff , UNIT_X);
    fADdU[0] = abs(fDdU[0]);
    if (fADdU[0] > xExtent && fDdU[0] * fWdU[0] >= 0.0)		return false;

    fWdU[1] = dot(ray.direction , UNIT_Y);
    fAWdU[1] = abs(fWdU[1]);
    fDdU[1] = dot(diff , UNIT_Y);
    fADdU[1] = abs(fDdU[1]);
    if (fADdU[1] > yExtent && fDdU[1] * fWdU[1] >= 0.0)		return false;

    fWdU[2] = dot(ray.direction , UNIT_Z);
    fAWdU[2] = abs(fWdU[2]);
    fDdU[2] = dot(diff , UNIT_Z);
    fADdU[2] = abs(fDdU[2]);
    if (fADdU[2] > zExtent && fDdU[2] * fWdU[2] >= 0.0)		return false;

    fAWxDdU[0] = abs(dot(wCrossD , UNIT_X));
    rhs = yExtent , fAWdU[2] + zExtent * fAWdU[1];
    if (fAWxDdU[0] > rhs)		return false;

    fAWxDdU[1] = abs(dot(wCrossD , UNIT_Y));
    rhs = xExtent * fAWdU[2] + zExtent * fAWdU[0];
    if (fAWxDdU[1] > rhs)		return false;

    fAWxDdU[2] = abs(dot(wCrossD , UNIT_Z));
    rhs = xExtent * fAWdU[1] + yExtent * fAWdU[0];
    if (fAWxDdU[2] > rhs)		return false;

    return true;
}*/

BoundingBox::BoundingBox( const Vector3d& fromMin, const Vector3d& toMax )
{
	vmin = fromMin;
	vmax = toMax;

	this->center = (vmin + vmax) / 2.0;

	this->xExtent = abs(vmax.x() - center.x());
	this->yExtent = abs(vmax.y() - center.y());
	this->zExtent = abs(vmax.z() - center.z());

	axis[0] = Vector3d(1, 0, 0);
	axis[1] = Vector3d(0, 1, 0);
	axis[2] = Vector3d(0, 0, 1);
}

std::vector<Vector3d> BoundingBox::getCorners()
{
    std::vector<Vector3d> corners;

	Vector3d x = (Vector3d(1,0,0) * xExtent);
	Vector3d y = (Vector3d(0,1,0) * yExtent);
	Vector3d z = (Vector3d(0,0,1) * zExtent);

	Vector3d c = center + x + y + z;

	corners.push_back(c);
	corners.push_back(c - (x*2));
	corners.push_back(c - (y*2));
	corners.push_back(c - (x*2) - (y*2));

	corners.push_back(corners[0] - (z*2));
	corners.push_back(corners[1] - (z*2));
	corners.push_back(corners[2] - (z*2));
	corners.push_back(corners[3] - (z*2));

	return corners;
}

/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-M�ller                              */
bool BoundingBox::containsTriangle( const Vector3d& tv0, const Vector3d& tv1, const Vector3d& tv2 ) const
{
	Vector3d boxcenter(center);
	Vector3d boxhalfsize(xExtent, yExtent, zExtent);

	int X = 0, Y = 1, Z = 2;

	/*    use separating axis theorem to test overlap between triangle and box */
	/*    need to test for overlap in these directions: */
	/*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
	/*       we do not even need to test these) */
	/*    2) normal of the triangle */
	/*    3) crossproduct(edge from tri, {x,y,z}-directin) */
	/*       this gives 3x3=9 more tests */
	Vector3d v0,v1,v2;
	double min,max,p0,p1,p2,rad,fex,fey,fez;
	Vector3d normal,e0,e1,e2;

	/* This is the fastest branch on Sun */
	/* move everything so that the box center is in (0,0,0) */
	v0=tv0-boxcenter;
	v1=tv1-boxcenter;
	v2=tv2-boxcenter;
	
	/* compute triangle edges */
	e0=v1-v0;      /* tri edge 0 */
	e1=v2-v1;      /* tri edge 1 */
	e2=v0-v2;      /* tri edge 2 */
	
	/* Bullet 3:  */
	/*  test the 9 tests first (this was faster) */
	fex = fabsf(e0[X]);
	fey = fabsf(e0[Y]);
	fez = fabsf(e0[Z]);
	AXISTEST_X01(e0[Z], e0[Y], fez, fey);
	AXISTEST_Y02(e0[Z], e0[X], fez, fex);
	AXISTEST_Z12(e0[Y], e0[X], fey, fex);
	fex = fabsf(e1[X]);
	fey = fabsf(e1[Y]);
	fez = fabsf(e1[Z]);
	AXISTEST_X01(e1[Z], e1[Y], fez, fey);
	AXISTEST_Y02(e1[Z], e1[X], fez, fex);
	AXISTEST_Z0(e1[Y], e1[X], fey, fex);
	fex = fabsf(e2[X]);
	fey = fabsf(e2[Y]);
	fez = fabsf(e2[Z]);
	AXISTEST_X2(e2[Z], e2[Y], fez, fey);
	AXISTEST_Y1(e2[Z], e2[X], fez, fex);
	AXISTEST_Z12(e2[Y], e2[X], fey, fex);
	
	/* Bullet 1: */
	/*  first test overlap in the {x,y,z}-directions */
	/*  find min, max of the triangle each direction, and test for overlap in */
	/*  that direction -- this is equivalent to testing a minimal AABB around */
	/*  the triangle against the AABB */
	/* test in X-direction */
	FINDMINMAX(v0[X],v1[X],v2[X],min,max);
	if(min>boxhalfsize[X] || max<-boxhalfsize[X]) return 0;
	/* test in Y-direction */
	FINDMINMAX(v0[Y],v1[Y],v2[Y],min,max);
	if(min>boxhalfsize[Y] || max<-boxhalfsize[Y]) return 0;
	/* test in Z-direction */
	FINDMINMAX(v0[Z],v1[Z],v2[Z],min,max);
	if(min>boxhalfsize[Z] || max<-boxhalfsize[Z]) return 0;
	
	/* Bullet 2: */
	/*  test if the box intersects the plane of the triangle */
	/*  compute plane equation of triangle: normal*x+d=0 */
	normal = cross(e0, e1);

	if(!planeBoxOverlap(normal,v0,boxhalfsize)) return 0;
	return 1;   /* box and triangle overlaps */
}

bool BoundingBox::intersectsBoundingBox( const BoundingBox& bb ) const
{
	if (center.x() + xExtent < bb.center.x() - bb.xExtent || center.x() - xExtent > bb.center.x() + bb.xExtent)
		return false;
	else if (center.y() + yExtent < bb.center.y() - bb.yExtent || center.y() - yExtent > bb.center.y() + bb.yExtent)
		return false;
	else if (center.z() + zExtent < bb.center.z() - bb.zExtent || center.z() - zExtent > bb.center.z() + bb.zExtent)
		return false;
	else
		return true;
}

bool BoundingBox::intersectsSphere( const Vector3d& sphere_center, double radius )
{
	if (abs(center.x() - sphere_center.x()) < radius + xExtent
		&& abs(center.y() - sphere_center.y()) < radius + yExtent
		&& abs(center.z() - sphere_center.z()) < radius + zExtent)
		return true;

	return false;
}

bool BoundingBox::contains( const Vector3d& point ) const
{
	return abs(center.x() - point.x()) < xExtent
		&& abs(center.y() - point.y()) < yExtent
		&& abs(center.z() - point.z()) < zExtent;
}

Vector3d BoundingBox::Center()
{
	return center;
}

void BoundingBox::Offset( double s )
{
	Offset( Vector3d(s,s,s));
}

void BoundingBox::Offset( Vector3d delta )
{
	*this = BoundingBox(vmin - delta, vmax + delta);
}

double BoundingBox::Diag()
{
	return (vmin - vmax).norm();
}

bool BoundingBox::containsBoundingBox(BoundingBox &bb)
{
	if (abs(center.x() - bb.center.x()) < xExtent - bb.xExtent
		&& abs(center.y() - bb.center.y()) < yExtent - bb.yExtent
		&& abs(center.z() - bb.center.z()) < zExtent - bb.zExtent)
		return true;

	return false;
}

void BoundingBox::computeFromBoundingBoxes(const std::vector<BoundingBox> &boxes)
{
	vmin = Vec3d(DBL_MAX, DBL_MAX, DBL_MAX);
	vmax = -vmin;

	double minx = 0, miny = 0, minz = 0;
	double maxx = 0, maxy = 0, maxz = 0;

	minx = maxx = boxes[0].vmin[0];
	miny = maxy = boxes[0].vmin[1];
	minz = maxz = boxes[0].vmin[2];

	for (int i = 0; i < boxes.size(); i++)
	{
		Vec3d bvmin = boxes[i].vmin;
		Vec3d bvmax = boxes[i].vmax;

		if (bvmin[0] < minx) minx = bvmin[0];
		if (bvmin[1] < miny) miny = bvmin[1];
		if (bvmin[2] < minz) minz = bvmin[2];

		if (bvmax[0] > maxx) maxx = bvmax[0];
		if (bvmax[0] > maxy) maxy = bvmax[1];
		if (bvmax[0] > maxz) maxz = bvmax[2];
	}

	vmax = Vec3d(maxx, maxy, maxz);
	vmin = Vec3d(minx, miny, minz);

	this->center = (vmin + vmax) / 2.0;

	this->xExtent = abs(vmax.x() - center.x());
	this->yExtent = abs(vmax.y() - center.y());
	this->zExtent = abs(vmax.z() - center.z());
}

bool BoundingBox::intersectSegment(const Vector3d &startPt, const Vector3d &endPt)
{
	Vector3d segmentCenter = 0.5*(startPt + endPt);
	Vector3d segmentDirection = endPt - startPt;
	double segmentExtent = ((double)0.5)*segmentDirection.norm();

	double AWdU[3], ADdU[3], AWxDdU[3], RHS;

	Vector3d diff = segmentCenter - this->center;

	AWdU[0] = std::fabs(segmentDirection.dot(this->axis[0]));
	ADdU[0] = std::fabs(diff.dot(this->axis[0]));
	RHS = this->xExtent + segmentExtent*AWdU[0];
	if (ADdU[0] > RHS)
	{
		return false;
	}

	AWdU[1] = std::fabs(segmentDirection.dot(this->axis[1]));
	ADdU[1] = std::fabs(diff.dot(this->axis[1]));
	RHS = this->yExtent + segmentExtent*AWdU[1];
	if (ADdU[1] > RHS)
	{
		return false;
	}

	AWdU[2] = std::fabs(segmentDirection.dot(this->axis[2]));
	ADdU[2] = std::fabs(diff.dot(this->axis[2]));
	RHS = this->zExtent + segmentExtent*AWdU[2];
	if (ADdU[2] > RHS)
	{
		return false;
	}

	Vector3d WxD = segmentDirection.cross(diff);

	AWxDdU[0] = std::fabs(WxD.dot(this->axis[0]));
	RHS = this->yExtent * AWdU[2] + this->zExtent * AWdU[1];
	if (AWxDdU[0] > RHS)
	{
		return false;
	}

	AWxDdU[1] = std::fabs(WxD.dot(this->axis[1]));
	RHS = this->xExtent * AWdU[2] + this->zExtent * AWdU[0];
	if (AWxDdU[1] > RHS)
	{
		return false;
	}

	AWxDdU[2] = std::fabs(WxD.dot(this->axis[2]));
	RHS = this->xExtent * AWdU[1] + this->yExtent * AWdU[0];
	if (AWxDdU[2] > RHS)
	{
		return false;
	}

	return true;
}
