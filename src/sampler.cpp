#include "sampler.h"

namespace CMU462 {

// Uniform Sampler2D Implementation //

Vector2D UniformGridSampler2D::get_sample() const {
    double x = double(std::rand()) / RAND_MAX;
    double y = double(std::rand()) / RAND_MAX;
    x *= 0.5;
    y *= 0.5;

    return Vector2D(x, y);
}

Vector2D UniformSemiCircleSampler2D::get_sample() const {
    double theta = double(std::rand()) / RAND_MAX * 2 * PI;
    double x, y;
    x = cos(theta);
    y = abs(sin(theta));

    return Vector2D(x, y);
}

Vector2D UniformCircleSampler2D::get_sample() const {
    double theta = double(std::rand()) / RAND_MAX * 2 * PI;
    double x, y;
    x = cos(theta);
    y = sin(theta);

    return Vector2D(x, y);
}

// Uniform Hemisphere Sampler3D Implementation //

Vector3D UniformHemisphereSampler3D::get_sample() const {

  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  return Vector3D(xs, ys, zs);

}

Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  float f;
  return get_sample(&f);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const {
  // You may implement this, but don't have to.
  return Vector3D(0, 0, 1);
}


} // namespace CMU462
