#include "circle.h"

#include <cmath>
#include <assert.h>

#include  "../bsdf.h"

using namespace std;

namespace CMU462 { namespace StaticScene {

bool Circle::test(const Ray& r, double& t1, double& t2) const {
    // Return true if there are intersections and writing the
    // smaller of the two intersection times in t1 and the larger in t2.
    Vector2D centerMinusO = r.o - o;
    double oDotD = dot(centerMinusO, r.d);
    double term2 = oDotD * oDotD - centerMinusO.norm2() + r2;

    if (term2 < 0) {
        // There are no real roots, thus no intersection
        return false;
    }

    double sqrtTerm2 = sqrt(term2);
    t1 = -oDotD + sqrtTerm2;
    t2 = -oDotD - sqrtTerm2;

    return true;

}

bool Circle::intersect(const Ray& r) const {
    double t1, t2;

    if (test(r, t1, t2)) {
        double minT = min(t1, t2);

        if (r.min_t < minT &&
            r.max_t > minT) {
            r.max_t = minT;
            return true;
        }
    }
    return false;


}

bool Circle::intersect(const Ray& r, Intersection *i) const {
    double t1, t2;

    if (test(r, t1, t2)) {
        double minT = min(t1, t2);

        if (r.min_t < minT &&
            r.max_t > minT) {
            Vector2D normVec;
            // Intersection point minus sphere center
            normVec = r.o + minT * r.d - o;
            normVec.normalize();
            r.max_t = minT;
            i->t = minT;
            i->primitive = this;
            i->n = normVec;
            i->bsdf = get_bsdf();
            return true;
        }
    }
    return false;

}

void Circle::draw(const Color& c) const {
  //Misc::draw_sphere_opengl(o, r, c);
}

void Circle::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CMU462
