#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix2x2& o2w, const Vector2D& n) {

    Vector2D y = Vector2D(n.x, n.y);
    Vector2D h = y;
    if (fabs(h.x) <= fabs(h.y)) {
        h.x = 1.0;
    } else {
        h.y = 1.0;
    }

    y.normalize();
    Vector2D x = Vector2D(y.y, -y.x);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector2D& wo, const Vector2D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector2D& wo, Vector2D* wi, float* pdf) {
    *wi = sampler.get_sample();
    *pdf = 1.0 / (2.0 * PI);
    return f(wo, *wi);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector2D& wo, const Vector2D& wi) {
    Vector2D wt;
    reflect(wo, &wt);
    if (wt == wi){
        return reflectance * (1.0 / cos_theta(wo));
    } else {
        return Spectrum();
    }
}

Spectrum MirrorBSDF::sample_f(const Vector2D& wo, Vector2D* wi, float* pdf) {
    reflect(wo, wi);
    // Mirrors always reflect in the same direction
    *pdf = 1.0;///(2*PI);
    return reflectance * (1.0 / cos_theta(wo));
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector2D& wo, const Vector2D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector2D& wo, Vector2D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector2D& wo, const Vector2D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector2D& wo, Vector2D* wi, float* pdf) {

  // TODO:
  // Implement RefractionBSDF

  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector2D& wo, const Vector2D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector2D& wo, Vector2D* wi, float* pdf) {
    // Compute Fresnel coefficient and either reflect or refract based on it.
    double rpar, rperp;
    refract(wo, wi, ior);
  return Spectrum();
}

void BSDF::reflect(const Vector2D& wo, Vector2D* wi) {
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
}

bool BSDF::refract(const Vector2D& wo, Vector2D* wi, float ior) {
    // Use Snell's Law to refract wo surface and store result ray in wi.
    // Return false if refraction does not occur due to total internal reflection
    // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
    // ray entering the surface through vacuum.

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector2D& wo, const Vector2D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector2D& wo, Vector2D* wi) {
  *wi  = sampler.get_sample();
  return Spectrum();
}

} // namespace CMU462
