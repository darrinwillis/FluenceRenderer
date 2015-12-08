#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {
    // compute the bounding box of the triangle
    double minx, miny, minz, maxx, maxy, maxz;
    Vector3D va, vb, vc, minCorner, maxCorner;

    va = mesh->positions[v1];
    vb = mesh->positions[v2];
    vc = mesh->positions[v3];

    minx = min({va[0], vb[0], vc[0]});
    miny = min({va[1], vb[1], vc[1]});
    minz = min({va[2], vb[2], vc[2]});
    maxx = max({va[0], vb[0], vc[0]});
    maxy = max({va[1], vb[1], vc[1]});
    maxz = max({va[2], vb[2], vc[2]});
    minCorner = Vector3D(minx, miny, minz);
    maxCorner = Vector3D(maxx, maxy, maxz);

    return BBox(minCorner, maxCorner);
}

bool Triangle::intersect(const Ray& r) const {
    Vector3D va, vb, vc, e1, e2, s, sxe2, e1xd, col, solution;
    double denom, u, v, t;
    va = mesh->positions[v1];
    vb = mesh->positions[v2];
    vc = mesh->positions[v3];

    e1 = vb - va;
    e2 = vc - va;
    s = r.o - va;
    sxe2 = cross(s, e2);
    e1xd = cross(e1, r.d);
    denom = dot(e1xd, e2);

    // Check for triangle parallel to viewing ray
    if (denom == 0) {
        return false;
    }

    col[0] = -dot(sxe2, r.d);
    col[1] = dot(e1xd, s);
    col[2] = -dot(sxe2, e1);

    solution = (1 / denom) * col;

    u = solution[0];
    v = solution[1];
    t = solution[2];
    if (u >= 0 &&
        u <= 1 &&
        v >= 0 &&
        v <= 1 &&
        u + v <= 1 &&
        u + v >= 0 &&
        r.min_t <= t &&
        r.max_t >= t) {
        // Set all of the structures
        r.max_t = t;
        return true;
    }
    return false;
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
    Vector3D va, vb, vc, e1, e2, s, sxe2, e1xd, col, solution;
    double denom, u, v, t;
    va = mesh->positions[v1];
    vb = mesh->positions[v2];
    vc = mesh->positions[v3];

    e1 = vb - va;
    e2 = vc - va;
    s = r.o - va;
    sxe2 = cross(s, e2);
    e1xd = cross(e1, r.d);
    denom = dot(e1xd, e2);

    // Check for triangle parallel to viewing ray
    if (denom == 0) {
        return false;
    }

    col[0] = -dot(sxe2, r.d);
    col[1] = dot(e1xd, s);
    col[2] = -dot(sxe2, e1);

    solution = (1 / denom) * col;

    u = solution[0];
    v = solution[1];
    t = solution[2];
    if (u >= 0 &&
        u <= 1 &&
        v >= 0 &&
        v <= 1 &&
        u + v <= 1 &&
        u + v >= 0 &&
        r.min_t <= t &&
        r.max_t >= t) {
        // Set all of the structures
        r.max_t = t;
        isect->t = t;
        isect->primitive = this;
        Vector3D inorm = (u * mesh->normals[v2] +
                          v * mesh->normals[v3] +
                          (1.f - u - v) * mesh->normals[v1]);
        inorm.normalize();
        // If the norm of the ray and the triangle are in the same direction
        // then we want to flip the norm. The dot product will be positive
        // if they are pointing in the same direction.
        isect->n = (dot(inorm, r.d) > 0) ? -inorm : inorm;
        isect->bsdf = get_bsdf();
        return true;
    }
    return false;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CMU462
