#ifndef CMU462_RAY_H
#define CMU462_RAY_H

#include "CMU462/CMU462.h"
#include "CMU462/vector2D.h"
#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/spectrum.h"

namespace CMU462 {


struct Ray {
  size_t depth;  ///< depth of the Ray

  Vector2D o;  ///< origin
  Vector2D d;  ///< direction
  mutable double min_t; ///< treat the ray as a segment (ray "begin" at max_t)
  mutable double max_t; ///< treat the ray as a segment (ray "ends" at max_t)

  Vector2D inv_d;  ///< component wise inverse
  int sign[2];     ///< fast ray-bbox intersection

  /**
   * Constructor.
   * Create a ray instance with given origin and direction.
   * \param o origin of the ray
   * \param d direction of the ray
   * \param depth depth of the ray
   */
    Ray(const Vector2D& o, const Vector2D& d, int depth = 0)
        : o(o), d(d), min_t(0.0), max_t(INF_D), depth(depth) {
    inv_d = Vector2D(1 / d.x, 1 / d.y);
    sign[0] = (inv_d.x < 0);
    sign[1] = (inv_d.y < 0);
  }

  /**
   * Constructor.
   * Create a ray instance with given origin and direction.
   * \param o origin of the ray
   * \param d direction of the ray
   * \param max_t max t value for the ray (if it's actually a segment)
   * \param depth depth of the ray
   */
    Ray(const Vector2D& o, const Vector2D& d, double max_t, int depth = 0)
        : o(o), d(d), min_t(0.0), max_t(max_t), depth(depth) {
    inv_d = Vector2D(1 / d.x, 1 / d.y);
    sign[0] = (inv_d.x < 0);
    sign[1] = (inv_d.y < 0);
  }


  /**
   * Returns the point t * |d| along the ray.
   */
  inline Vector2D at_time(double t) const { return o + t * d; }

  /**
   * Returns the result of transforming the ray by the given transformation
   * matrix.
   */
  Ray transform_by(const Matrix3x3& t) const {
    const Vector3D& newO = t * Vector3D(o.x, o.y, 1.0);
    return Ray((newO / newO.z).to2D(), (t * Vector3D(d.x, d.y, 0.0)).to2D());
  }
};

// structure used for logging rays for subsequent visualization
struct LoggedRay {

    LoggedRay(const Ray& r, double hit_t)
        : o(r.o), d(r.d), hit_t(hit_t) {}

    Vector2D o;
    Vector2D d;
    double hit_t;
};

}  // namespace CMU462

#endif  // CMU462_RAY_H
