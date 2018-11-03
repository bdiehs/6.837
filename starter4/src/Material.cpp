#include "Material.h"

float clamp(const Vector3f& A, const Vector3f& B) {
    float out = Vector3f::dot(A, B);
    return (out > 0) ? out : 0;
}

Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
    // TODO implement Diffuse and Specular phong terms
    Vector3f I_diffuse =  clamp(dirToLight, hit.getNormal()) * lightIntensity * _diffuseColor;
    Vector3f reflected_ray = 2 * Vector3f::dot(-ray.getDirection(), hit.getNormal()) * hit.getNormal() + ray.getDirection();
    Vector3f I_specular = pow(clamp(dirToLight, reflected_ray), _shininess) * lightIntensity * _specularColor;

    return I_diffuse + I_specular;
}

