#ifndef CMU462_BBOX_H
#define CMU462_BBOX_H

#include <utility>
#include <algorithm>

#include "CMU462/CMU462.h"

#include "ray.h"

namespace CMU462 {

/**
 * Axis-aligned bounding box.
 * An AABB is given by two positions in space, the min and the max. An addition
 * component, the extent of the bounding box is stored as it is useful in a lot
 * of the operations on bounding boxes.
 */
struct BBox {

  Vector2D max;	    ///< min corner of the bounding box
  Vector2D min;	    ///< max corner of the bounding box
  Vector2D extent;  ///< extent of the bounding box (min -> max)

  /**
   * Constructor.
   * The default constructor creates a new bounding box which contains no
   * points.
   */
  BBox() {
    max = Vector2D(-INF_D, -INF_D);
    min = Vector2D( INF_D,  INF_D);
    extent = max - min;
  }

  /**
   * Constructor.
   * Creates a bounding box that includes a single point.
   */
  BBox(const Vector2D& p) : min(p), max(p) { extent = max - min; }

  /**
   * Constructor.
   * Creates a bounding box with given bounds.
   * \param min the min corner
   * \param max the max corner
   */
  BBox(const Vector2D& min, const Vector2D& max) :
       min(min), max(max) { extent = max - min; }

  /**
   * Constructor.
   * Creates a bounding box with given bounds (component wise).
   */
  BBox(const double minX, const double minY,
       const double maxX, const double maxY) {
    min = Vector2D(minX, minY);
    max = Vector2D(maxX, maxY);
		extent = max - min;
  }

  /**
   * Expand the bounding box to include another (union).
   * If the given bounding box is contained within *this*, nothing happens.
   * Otherwise *this* is expanded to the minimum volume that contains the
   * given input.
   * \param bbox the bounding box to be included
   */
  void expand(const BBox& bbox) {
    min.x = std::min(min.x, bbox.min.x);
    min.y = std::min(min.y, bbox.min.y);
    max.x = std::max(max.x, bbox.max.x);
    max.y = std::max(max.y, bbox.max.y);
    extent = max - min;
  }

  /**
   * Expand the bounding box to include a new point in space.
   * If the given point is already inside *this*, nothing happens.
   * Otherwise *this* is expanded to a minimum volume that contains the given
   * point.
   * \param p the point to be included
   */
  void expand(const Vector2D& p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    extent = max - min;
  }

  Vector2D centroid() const {
    return (min + max) / 2;
  }

  /**
   * Compute the surface area of the bounding box.
   * \return surface area of the bounding box.
   */
  double perimeter() const {
    if (empty()) return 0.0;
    return 2 * (extent.x + extent.y);
  }

  /**
   * Check if bounding box is empty.
   * Bounding box that has no size is considered empty. Note that since
   * bounding box are used for objects with positive volumes, a bounding
   * box of zero size (empty, or contains a single vertex) are considered
   * empty.
   */
  bool empty() const {
    return min.x > max.x || min.y > max.y;
  }

  /**
   * Ray - bbox intersection.
   * Intersects ray with bounding box, does not store shading information.
   * \param r the ray to intersect with
   * \param t0 lower bound of intersection time
   * \param t1 upper bound of intersection time
   */
  bool intersect(const Ray& r, double& t0, double& t1) const;

  /**
   * Draw box wireframe with OpenGL.
   * \param c color of the wireframe
   */
  void draw(Color c) const;
};

std::ostream& operator<<(std::ostream& os, const BBox& b);

} // namespace CMU462

#endif // CMU462_BBOX_H
