#pragma once

#include <float.h>
#include <qgl.h>
#include <QVector3D>
#include "RenderObject.h"

#include "SurfaceMeshModel.h"
using namespace SurfaceMesh;
using namespace Eigen;

#define glVertQt(v) glVertex3d(v.x(), v.y(), v.z())
#define glColorQt(c) glColor4d(c.redF(), c.greenF(), c.blueF(), c.alphaF())
#define glv(v) glVertex3dv(v.data())

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

const double PointRadius3D = 0.01;

// Custom QVector3D
class QVector3: public QVector3D{
public:
	QVector3(){ setX(0);setY(0);setZ(0); }
	QVector3(double x, double y, double z){ this->setX(x);this->setY(y);this->setZ(z); }
	QVector3 (const QVector3D& v){ this->setX(v.x());this->setY(v.y());this->setZ(v.z()); }
	QVector3 (const Vector3d& v){ this->setX(v.x());this->setY(v.y());this->setZ(v.z()); }
	operator const Vector3d() { return Vector3d(x(),y(),z()); }
	operator const QVector3D() { return *this; }
};

class PolygonSoup : public RenderObject::Base{
	QVector< QVector<QVector3> > polys;
	QVector< QVector3 > polys_normals;
	QVector< QColor > polys_colors;

public:
	PolygonSoup():RenderObject::Base(1, Qt::black){}

	void clear(){
		polys.clear();
		polys_normals.clear();
		polys_colors.clear();
	}

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

	void draw(){

        glEnable(GL_LIGHTING);
		glEnable(GL_BLEND);

        drawTris();
        drawQuads();

        glDisable(GL_LIGHTING);

		// Draw borders as lines and points to force drawing something
		glLineWidth(2.0f);
		glColor4d(0,0,0,1);
		glBegin(GL_LINES);
		foreach(QVector<QVector3> poly, polys){
			for(int i = 0; i < (int) poly.size(); i++){
				glVertQt(poly[i]);
				glVertQt(poly[(i+1) % poly.size()]);
			}
		}
		glEnd();

		glPointSize(3.0f);
		glBegin(GL_POINTS);
		foreach(QVector<QVector3> poly, polys){
			for(int i = 0; i < (int) poly.size(); i++)
				glVertQt(poly[i]);
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}

	void drawTris(bool isColored = true){
		glBegin(GL_TRIANGLES);
		for(int i = 0; i < (int) polys.size(); i++)
		{
			if(polys[i].size() != 3) continue;
			else{
				glNormal3d(polys_normals[i].x(),polys_normals[i].y(),polys_normals[i].z());
				if(isColored) glColorQt(polys_colors[i]);
				for(int p = 0; p < 3; p++) glVertQt(polys[i][p]);
			}
		}
		glEnd();
	}

	void drawQuads(bool isColored = true){
		glBegin(GL_QUADS);
		for(int i = 0; i < (int) polys.size(); i++)
		{
			if(polys[i].size() != 4) continue;
			else{
				glNormal3d(polys_normals[i].x(),polys_normals[i].y(),polys_normals[i].z());
				if(isColored) glColorQt(polys_colors[i]);
				for(int p = 0; p < 4; p++) glVertQt(polys[i][p]);
			}
		}
		glEnd();
	}

	void addPoly(const QVector<QVector3>& points, const QColor& c = Qt::red){
		if(points.size() < 3) return;
		else
			polys.push_back(points);

		// Compute normal from 3 points
		polys_normals.push_back(QVector3D::crossProduct(points[1] - points[0], points[2] - points[0]).normalized());
		polys_colors.push_back(c);
	}
};

class LineSegments : public RenderObject::Base{
	QVector< QPair<QVector3,QVector3> > lines;
	QVector< QColor > lines_colors;
public:
	LineSegments():RenderObject::Base(1, Qt::black){}

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

	void draw(){
		glDisable(GL_LIGHTING);

		glLineWidth(_size);
		glBegin(GL_LINES);
		for(int i = 0; i < (int) lines.size(); i++){
			glColorQt(lines_colors[i]);
			glVertQt(lines[i].first);
			glVertQt(lines[i].second);
		}
		glEnd();

		glPointSize(_size+2);
		glBegin(GL_POINTS);
		for(int i = 0; i < (int) lines.size(); i++){
			glColorQt(lines_colors[i]);
			glVertQt(lines[i].first);
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}

	void addLine(const QVector3& p1, const QVector3& p2, const QColor& c = Qt::blue){
		lines.push_back( qMakePair(p1,p2) );
		lines_colors.push_back(c);
	}

	void addLines()
	{

	}
};

class PointSoup : public RenderObject::Base{
	QVector< QVector3 > points;
	QVector< QVector3 > normals;
	QVector< QColor > points_colors;
public:
	PointSoup(float size = 6.0f):RenderObject::Base(size, Qt::black){}

	void clear(){
		points.clear();
		points_colors.clear();
	}

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

	void draw(){
		glDisable(GL_LIGHTING);
		if(normals.size()) glEnable(GL_LIGHTING);

		glPointSize(_size);
		glBegin(GL_POINTS);
		for(int i = 0; i < (int) points.size(); i++){
			if(normals.size()) glNormal3d(normals[i].x(),normals[i].y(),normals[i].z());
			glColorQt(points_colors[i]);
			glVertQt(points[i]);
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}

	void addPointNormal(const QVector3& p, const QVector3& n, const QColor& c = Qt::blue){
		addPoint(p,c);
		normals.push_back(n);
	}

	void addPoint(const QVector3& p, const QColor& c = Qt::blue){
		points.push_back(p);
		points_colors.push_back(c);
	}

    size_t count(){
        return points.size();
    }
};

class VectorSoup : public RenderObject::Base{
    QVector< QPair<QVector3,QVector3> > vectors;
    QVector< double > vectorLengths;
    double maxLen;
public:
    VectorSoup(const QColor& c = Qt::green):RenderObject::Base(1, c){ maxLen = -DBL_MAX; }

	void clear(){
		vectors.clear();
		vectorLengths.clear();
		maxLen = -DBL_MAX;
	}

    void addVector(const QVector3& start, const QVector3& direction){
        vectors.push_back( qMakePair(start,direction) );
        double l = direction.length();
        vectorLengths.push_back(l);
        maxLen = qMax(l,maxLen);
    }

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

    void draw(float thickness = 1.0f){
        glDisable(GL_LIGHTING);
        glLineWidth(thickness);
        glBegin(GL_LINES);
        for(int i = 0; i < (int) vectors.size(); i++){
            // Color
            //double d = vectorLengths[i] / maxLen;
            //QColor c( _color.red() * d, _color.green() * d, _color.blue() * d );
            glColorQt(this->_color.lighter());

            // Line
            glVertQt(vectors[i].first);
            glVertQt((vectors[i].first + vectors[i].second));
        }
        glEnd();

        glPointSize(thickness * 2);
        glBegin(GL_POINTS);
        for(int i = 0; i < (int) vectors.size(); i++){
            // Color
            //double d = vectorLengths[i] / maxLen;
            //QColor c( _color.red() * d, _color.green() * d, _color.blue() * d );
            glColorQt(this->_color);

            // Point
            glVertQt((vectors[i].first + vectors[i].second));
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
};


class PlaneSoup : public RenderObject::Base{
	QVector< QPair<QVector3,QVector3> > planes;
	double scale;
public:
	PlaneSoup(double s = 1.0, const QColor& c = Qt::green):RenderObject::Base(1, c)
	{ scale = s; }

	void addPlane(const QVector3& center, const QVector3& normal){
		planes.push_back( qMakePair(center, normal) );
	}

	void draw(QGLWidget &widget){this->draw();widget;}

	void draw(){
		glDisable(GL_LIGHTING);

		// Rendering options
		glEnable (GL_POINT_SMOOTH);
		glEnable (GL_LINE_SMOOTH);
		glEnable (GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);

		float color[4];
		glGetFloatv(GL_CURRENT_COLOR, color);

		// Bake geometry
		int c = planes.size();
		std::vector<QVector3> n(c),u(c),v(c),p1(c),p2(c),p3(c),p4(c);
		for(int i = 0; i < c; i++){
			n[i] = planes[i].second;
			u[i] = scale * orthogonalVector(n[i]).normalized();
			v[i] = scale * QVector3D::crossProduct(n[i], u[i]).normalized();

			p1[i] = QVector3(planes[i].first + (u[i] + v[i]));
			p2[i] = QVector3(planes[i].first + (-u[i] + v[i]));
			p3[i] = QVector3(planes[i].first + (-v[i] + u[i]));
			p4[i] = QVector3(planes[i].first + (-v[i] + -u[i]));
		}

		// Draw Borders
		glColor4f(color[0]*0.8, color[1]*0.8, color[2]*0.8, 0.5);
		glLineWidth(2.0);
		glPolygonMode(GL_FRONT,GL_LINE);
		glBegin(GL_QUADS);
		for(int i = 0; i < c; i++)
		{
			glVertQt(p1[i]);
			glVertQt(p2[i]);
			glVertQt(p4[i]);
			glVertQt(p3[i]);
		}
		glEnd();

		// Draw Center
		glColor3f(color[0], color[1], color[2]);
		glPointSize(4.0);
		glBegin(GL_POINTS);
		for(int i = 0; i < c; i++) glVertQt(planes[i].first);
		glEnd();
		glPointSize(8.0);
		glColor4f(1, 1, 1, 0.5);
		glBegin(GL_POINTS);
		for(int i = 0; i < c; i++) glVertQt(planes[i].first);
		glEnd();

		// Draw Normal
		glBegin(GL_LINES);
		for(int i = 0; i < c; i++)
		{
			Vector3d center = planes[i].first;
			glv(center);
			glv(Surface_mesh::Vector3(center + ((Vector3d)n[i] * 0.2 * scale)));
		}
		glEnd();

		// Draw Transparent Fills
		glColor4f(color[0], color[1], color[2], 0.05f);
		glPolygonMode(GL_FRONT,GL_FILL);
		glBegin(GL_QUADS);
		for(int i = 0; i < c; i++)
		{
			glVertQt(p1[i]);
			glVertQt(p2[i]);
			glVertQt(p4[i]);
			glVertQt(p3[i]);
		}
		glEnd();

		glDisable(GL_BLEND);
		glEnable(GL_LIGHTING);
	}

	static QVector3 orthogonalVector(const QVector3& n) {
		if ((abs(n.y()) >= 0.9 * abs(n.x())) &&
			abs(n.z()) >= 0.9 * abs(n.x())) return QVector3(0.0, -n.z(), n.y());
		else if ( abs(n.x()) >= 0.9 * abs(n.y()) &&
			abs(n.z()) >= 0.9 * abs(n.y()) ) return QVector3(-n.z(), 0.0, n.x());
		else return QVector3(-n.y(), n.x(), 0.0);
	}
};

class FrameSoup : public RenderObject::Base{
	QVector< QVector<QVector3> > frames;
	bool isFlip;

public:
	FrameSoup(float scale, bool flip = false, const QColor& c = Qt::green):RenderObject::Base(scale, c)
	{
		this->isFlip = flip;
	}

	void addFrame(const QVector3& X, const QVector3& Y, const QVector3& Z, const QVector3& position){
		QVector<QVector3> frame;
		if(isFlip)
		{
			frame.push_back(Z);
			frame.push_back(Y);
			frame.push_back(X);
		}
		else
		{
			frame.push_back(X);
			frame.push_back(Y);
			frame.push_back(Z);
		}
		frame.push_back(position);
		frames.push_back(frame);
	}

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

	void draw()
	{
		drawFrame();
	}

	void drawFrame( int colorOffset = 0 )
	{
		QColor frameColors[] = { QColor(255,0,0), QColor(0,255,0), QColor(0,0,255) };
		int ci = colorOffset;

		glDisable(GL_LIGHTING);
		glLineWidth(1);
		glBegin(GL_LINES);
		for(int i = 0; i < (int) frames.size(); i++){
			QVector3 X=frames[i][0],Y=frames[i][1],Z=frames[i][2];
			QVector3 pos=frames[i][3];
			glColorQt(frameColors[(ci+0)%3]); glVertQt(pos); glVertQt((pos + X * _size));
			glColorQt(frameColors[(ci+1)%3]); glVertQt(pos); glVertQt((pos + Y * _size));
			glColorQt(frameColors[(ci+2)%3]); glVertQt(pos); glVertQt((pos + Z * _size));
		}
		glEnd();

		glPointSize(3);
		glBegin(GL_POINTS);
		for(int i = 0; i < (int) frames.size(); i++){
			QVector3 X=frames[i][0],Y=frames[i][1],Z=frames[i][2];
			QVector3 pos=frames[i][3];
			glColorQt(frameColors[(ci+0)%3]); glVertQt((pos + X * _size));
			glColorQt(frameColors[(ci+1)%3]); glVertQt((pos + Y * _size));
			glColorQt(frameColors[(ci+2)%3]); glVertQt((pos + Z * _size));
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}
};

static void renderSphere(float cx, float cy, float cz, float r)
{
	#ifndef M_PI
	#define M_PI       3.14159265358979323846
	#endif

	#ifndef M_PI_2
	#define M_PI_2     1.57079632679489661923
	#endif

	const int p = 24;

	float theta1 = 0.0, theta2 = 0.0, theta3 = 0.0;
	float ex = 0.0f, ey = 0.0f, ez = 0.0f;
	float px = 0.0f, py = 0.0f, pz = 0.0f;
	GLfloat vertices[p*6+6], normals[p*6+6], texCoords[p*4+4];

	for(int i = 0; i < p/2; ++i){
		theta1 = i * (M_PI*2) / p - M_PI_2;
		theta2 = (i + 1) * (M_PI*2) / p - M_PI_2;

		for(int j = 0; j <= p; ++j){
			theta3 = j * (M_PI*2) / p;

			ex = cosf(theta2) * cosf(theta3);
			ey = sinf(theta2);
			ez = cosf(theta2) * sinf(theta3);
			px = cx + r * ex;
			py = cy + r * ey;
			pz = cz + r * ez;

			vertices[(6*j)+(0%6)] = px;
			vertices[(6*j)+(1%6)] = py;
			vertices[(6*j)+(2%6)] = pz;

			normals[(6*j)+(0%6)] = ex;
			normals[(6*j)+(1%6)] = ey;
			normals[(6*j)+(2%6)] = ez;

			texCoords[(4*j)+(0%4)] = -(j/(float)p);
			texCoords[(4*j)+(1%4)] = 2*(i+1)/(float)p;

			ex = cosf(theta1) * cosf(theta3);
			ey = sinf(theta1);
			ez = cosf(theta1) * sinf(theta3);
			px = cx + r * ex;
			py = cy + r * ey;
			pz = cz + r * ez;

			vertices[(6*j)+(3%6)] = px;
			vertices[(6*j)+(4%6)] = py;
			vertices[(6*j)+(5%6)] = pz;

			normals[(6*j)+(3%6)] = ex;
			normals[(6*j)+(4%6)] = ey;
			normals[(6*j)+(5%6)] = ez;

			texCoords[(4*j)+(2%4)] = -(j/(float)p);
			texCoords[(4*j)+(3%4)] = 2*i/(float)p;
		}

		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);

		glTexCoordPointer(2, GL_FLOAT, 0, texCoords);
		glNormalPointer(GL_FLOAT, 0, normals);
		glVertexPointer(3, GL_FLOAT, 0, vertices);

		glDrawArrays(GL_TRIANGLE_STRIP, 0, (p+1)*2);

		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	}
}

class SphereSoup : public RenderObject::Base{
	QVector< QVector3 > centers;
	QVector< float > radii;
	QVector< QColor > colors;
public:
	SphereSoup(const QColor& c = Qt::yellow):RenderObject::Base(1, c){}

	void clear(){
		centers.clear();
		radii.clear();
		colors.clear();
	}

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

	void draw(){
		glEnable(GL_LIGHTING);
		for(int i = 0; i < (int) centers.size(); i++){
			Vector3d c = centers[i];
			glColorQt(colors[i]);
			renderSphere(c[0],c[1],c[2],radii[i]);
		}
	}

	void addSphere(const QVector3& center, float radius = 0.1f){
		centers.push_back(center);
		radii.push_back(radius);
		colors.push_back(_color);
	}

	void addSphere(const QVector3& center, float radius, const QColor& c){
		centers.push_back(center);
		radii.push_back(radius);
		colors.push_back(c);
	}
};

class CubeSoup : public RenderObject::Base{
	QVector< QVector3 > centers;
	QVector< float > lengths;
	QVector< QColor > colors;
	bool isWireframe;

public:
	CubeSoup(float defaultSize = 1.0, bool is_wireframe = true):RenderObject::Base(defaultSize, Qt::black)
	{ isWireframe = is_wireframe; }

	void clear(){
		centers.clear();
		lengths.clear();
		colors.clear();
	}

	void drawCube(QVector3 center, double length = 1.0)
	{
		static GLdouble n[6][3] =
		{{-1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0},
		{1.0, 0.0, 0.0},
		{0.0, -1.0, 0.0},
		{0.0, 0.0, 1.0},
		{0.0, 0.0, -1.0}};

		static GLint faces[6][4] =
		{{0, 1, 2, 3},
		{3, 2, 6, 7},
		{7, 6, 5, 4},
		{4, 5, 1, 0},
		{5, 6, 2, 1},
		{7, 4, 0, 3}};

		GLdouble v[8][3]; GLint i;

		v[0][0] = v[1][0] = v[2][0] = v[3][0] = -length / 2;
		v[4][0] = v[5][0] = v[6][0] = v[7][0] = length / 2;
		v[0][1] = v[1][1] = v[4][1] = v[5][1] = -length / 2;
		v[2][1] = v[3][1] = v[6][1] = v[7][1] = length / 2;
		v[0][2] = v[3][2] = v[4][2] = v[7][2] = -length / 2;
		v[1][2] = v[2][2] = v[5][2] = v[6][2] = length / 2;

		glPushMatrix();
		glTranslatef(center.x(), center.y(), center.z());

		for (i = 0; i < 6; i++) 
		{
			glBegin(GL_QUADS);
			glNormal3dv(&n[i][0]);
			glVertex3dv(&v[faces[i][0]][0]);
			glVertex3dv(&v[faces[i][1]][0]);
			glVertex3dv(&v[faces[i][2]][0]);
			glVertex3dv(&v[faces[i][3]][0]);
			glEnd();
		}

		glPopMatrix();
	}

    void draw(QGLWidget &widget){ this->draw(); Q_UNUSED(widget) }

	void draw()
	{
		glEnable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		
		if(isWireframe){
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDisable(GL_LIGHTING);
		}

		for(int i = 0; i < (int) centers.size(); i++){
			glColorQt(colors[i]);
			drawCube(centers[i],lengths[i]);
		}

		glEnable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	void addCube(const QVector3& center, float length = 1, const QColor& c = Qt::yellow){
		centers.push_back(center);
		lengths.push_back(length * _size);
		colors.push_back(c);
	}
};

static QColor qtColdColor(double value, double min = 0.0, double max = 1.0){
	unsigned char rgb[3];
	value-=min;
	if(value==HUGE_VAL)
	{rgb[0]=rgb[1]=rgb[2]=255;}
	else if(value<0)
	{rgb[0]=rgb[1]=rgb[2]=0;}
	else if(value<max)
	{rgb[0]=0;rgb[1]=0;rgb[2]=(unsigned char)(255*value/max);}
	else {rgb[0]=rgb[1]=0;rgb[2]=255;}
	return QColor(rgb[0],rgb[1],rgb[2]);
}

static QColor qtJetColorMap(double value, double min = 0.0, double max = 1.0)
{
    unsigned char rgb[3];
    unsigned char c1=144;
    float max4=(max-min)/4;
    value-=min;
    if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
    else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
    else if(value<max4)
    {rgb[0]=0;rgb[1]=0;rgb[2]=c1+(unsigned char)((255-c1)*value/max4);}
    else if(value<2*max4)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max4)/max4);rgb[2]=255;}
    else if(value<3*max4)
    {rgb[0]=(unsigned char)(255*(value-2*max4)/max4);rgb[1]=255;rgb[2]=255-rgb[0];}
    else if(value<max)
    {rgb[0]=255;rgb[1]=(unsigned char)(255-255*(value-3*max4)/max4);rgb[2]=0;}
    else {rgb[0]=255;rgb[1]=rgb[2]=0;}
    return QColor(rgb[0],rgb[1],rgb[2]);
}

#include <time.h>
static std::vector<double> randomColor()
{
    std::vector<double> color;

    float r = float( qMin(((rand() % 225) + 30), 255) ) / 255.0f;
    float g = float( qMin(((rand() % 230) + 25), 255) ) / 255.0f;
    float b = float( qMin(((rand() % 235) + 20), 255) ) / 255.0f;

    color.push_back(r);
    color.push_back(g);
    color.push_back(b);
    color.push_back(1.0);

    return color;
}

static std::vector< std::vector<double> > randomColors( int count )
{
	srand(time(NULL));

	std::vector< std::vector<double> > colors(count);
    for (int i = 0; i < count; i++)
        colors[i] = randomColor();
	return colors;
}

static QColor qRandomColor()
{
    std::vector<double> c = randomColor();
    return QColor::fromRgbF( c[0], c[1], c[2], c[3] );
}

static QColor qRandomColor2(double saturation = 0.5, double val = 0.95)
{
	double golden_ratio_conjugate = 0.618033988749895;
	double h = ((double)rand() / RAND_MAX);
	h += golden_ratio_conjugate;
	h = fmod(h, 1.0);
	return QColor::fromHsvF(h, saturation, val);
}

// Helper functions
double inline uniformRand(double a = 0.0, double b = 1.0){
	double len = b - a;
	return ((double)rand()/RAND_MAX) * len + a;
}

// for mid = 0, colors are centered around Red
static QColor qRandomColor3(double mid = 0, double range = 0.5, double saturation = 1.0, double val = 1.0)
{
	// Bound checks
	mid = qMax(0.0, qMin(1.0, mid));
	range = qMin(0.5, range);

	double h = uniformRand( -range + mid, range + mid );

	h = fmod(1.0 + h, 1.0);

	return QColor::fromHsvF(h, saturation, val);
}
