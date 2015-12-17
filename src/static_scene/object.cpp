#include "object.h"
#include "circle.h"

#include <vector>
#include <iostream>
#include <unordered_map>

using std::vector;
using std::unordered_map;

namespace CMU462 { namespace StaticScene {

// Circle object //

CircleObject::CircleObject(const Vector2D& o, double r, BSDF* bsdf) {

  this->o = o;
  this->r = r;
  this->bsdf = bsdf;
  
}

std::vector<Primitive*> CircleObject::get_primitives() const {
  std::vector<Primitive*> primitives;
  primitives.push_back(new Circle(this,o,r));
  return primitives;
}

BSDF* CircleObject::get_bsdf() const {
  return bsdf;
}


} // namespace StaticScene
} // namespace CMU462
