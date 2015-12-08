#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
    // Implement ray - bounding box intersection test
    // If the ray intersected the bouding box within the range given by
    // t0, t1, update t0 and t1 with the new intersection times.
    double totalMin, totalMax;
    double minx, miny, minz, maxx, maxy, maxz;

    // we need to use the sign somehow
    minx = r.sign[0] ?
           (max[0] - r.o[0]) * r.inv_d[0]:
           (min[0] - r.o[0]) * r.inv_d[0];
    miny = r.sign[1] ?
           (max[1] - r.o[1]) * r.inv_d[1]:
           (min[1] - r.o[1]) * r.inv_d[1];
    minz = r.sign[2] ?
           (max[2] - r.o[2]) * r.inv_d[2]:
           (min[2] - r.o[2]) * r.inv_d[2];

    maxx = r.sign[0] ?
           (min[0] - r.o[0]) * r.inv_d[0]:
           (max[0] - r.o[0]) * r.inv_d[0];
    maxy = r.sign[1] ?
           (min[1] - r.o[1]) * r.inv_d[1]:
           (max[1] - r.o[1]) * r.inv_d[1];
    maxz = r.sign[2] ?
           (min[2] - r.o[2]) * r.inv_d[2]:
           (max[2] - r.o[2]) * r.inv_d[2];

    totalMin = std::max({minx, miny, minz});
    totalMax = std::min({maxx, maxy, maxz});

    if (totalMin <= totalMax) {
        t0 = totalMin;
        t1 = totalMax;
        return true;
    }

    return false;
  
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
	glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CMU462
