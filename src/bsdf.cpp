#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    *wi = sampler.get_sample();
    *pdf = 1.0 / (2.0 * PI);
    return f(wo, *wi);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    Vector3D wt;
    reflect(wo, &wt);
    if (wt == wi){
        return reflectance * (1.0 / cos_theta(wo));
    } else {
        return Spectrum();
    }
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    reflect(wo, wi);
    // Mirrors always reflect in the same direction
    *pdf = 1.0;///(2*PI);
    return reflectance * (1.0 / cos_theta(wo));
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO:
  // Implement RefractionBSDF

  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    // Compute Fresnel coefficient and either reflect or refract based on it.
    double rpar, rperp;
    refract(wo, wi, ior);
  return Spectrum();
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
    *wi = -wo + Vector3D(0, 0, 2.0*wo[2]);
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
    // Use Snell's Law to refract wo surface and store result ray in wi.
    // Return false if refraction does not occur due to total internal reflection
    // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
    // ray entering the surface through vacuum.

    if (wo[2] < 0) {
        if (1.0 - (1.0/ior)*(1.0/ior)*(1.0 - (*wi)[2] * (*wi)[2]) < 0.0) {
            return false;
        }
    } else {
        // Light is entering the surface
        Vector3D phi = Vector3D(wo[0], wo[1], 0.0);
        phi.normalize();
        double cosThetaT = sqrt(1.0 - (ior/1.0)*(ior/1.0)*(1.0 - wo[2]*wo[2]));
        double inCosOverOut = wo[2] / cosThetaT;
        *wi = Vector3D(wo[0] * inCosOverOut, wo[1] * inCosOverOut, cosThetaT);
    }
    return true;

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CMU462