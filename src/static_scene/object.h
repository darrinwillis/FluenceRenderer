#ifndef CMU462_STATICSCENE_OBJECT_H
#define CMU462_STATICSCENE_OBJECT_H

#include "scene.h"

namespace CMU462 { namespace StaticScene {

/**
 * A sphere object.
 */
class CircleObject : public SceneObject {
 public:

  /**
  * Constructor.
  * Construct a static sphere for rendering from given parameters
  */
  CircleObject(const Vector2D& o, double r, BSDF* bsdf);

  /**
  * Get all the primitives (Circle) in the sphere object.
  * Note that Circle reference the sphere object for the actual data.
  * \return all the primitives in the sphere object
  */
  std::vector<Primitive*> get_primitives() const;

  /**
   * Get the BSDF of the surface material of the sphere.
   * \return BSDF of the surface material of the sphere
   */
  BSDF* get_bsdf() const;

  Vector2D o; ///< origin
  double r;   ///< radius

private:

  BSDF* bsdf; ///< BSDF of the sphere objects' surface material

}; // class CircleObject


} // namespace StaticScene
} // namespace CMU462

#endif // CMU462_STATICSCENE_OBJECT_H
