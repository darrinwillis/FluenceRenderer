#include "vector3D.h"

namespace CMU462 {

  std::ostream& operator<<( std::ostream& os, const Vector3D& v ) {
    os << "(" << v.x << "," << v.y << "," << v.z << ")";
    return os;
  }

  Vector2D Vector3D::to2D() {
      return Vector2D(x, y);
  }

} // namespace CMU462
