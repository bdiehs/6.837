#include "Object3D.h"

bool Sphere::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER

    // We provide sphere intersection code for you.
    // You should model other intersection implementations after this one.

    // Locate intersection point ( 2 pts )
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f origin = rayOrigin - _center;      //Ray origin in the sphere coordinate

    float a = dir.absSquared();
    float b = 2 * Vector3f::dot(dir, origin);
    float c = origin.absSquared() - _radius * _radius;

    // no intersection
    if (b * b - 4 * a * c < 0) {
        return false;
    }

    float d = sqrt(b * b - 4 * a * c);

    float tplus = (-b + d) / (2.0f*a);
    float tminus = (-b - d) / (2.0f*a);

    // the two intersections are at the camera back
    if ((tplus <= tmin) && (tminus <= tmin)) {
        return false;
    }

    float t = 10000;
    // the two intersections are at the camera front
    if (tminus > tmin) {
        t = tminus;
    }

    // one intersection at the front. one at the back 
    if ((tplus > tmin) && (tminus < tmin)) {
        t = tplus;
    }

    if (t < h.getT()) {
        Vector3f normal = r.pointAtParameter(t) - _center;
        normal = normal.normalized();
        h.set(t, this->material, normal);
        return true;
    }
    // END STARTER
    return false;
}

// Add object to group
void Group::addObject(Object3D *obj) {
    m_members.push_back(obj);
}

// Return number of objects in group
int Group::getGroupSize() const {
    return (int)m_members.size();
}

bool Group::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER
    // we implemented this for you
    bool hit = false;
    for (Object3D* o : m_members) {
        if (o->intersect(r, tmin, h)) {
            hit = true;
        }
    }
    return hit;
    // END STARTER
}


Plane::Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
    // TODO implement Plane constructor
    material = m;
    _normal = normal;
    _d = d;
}
bool Plane::intersect(const Ray &r, float tmin, Hit &h) const
{
    // TODO implement
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &norm_dir = r.getDirection();

    float t = (_d - Vector3f::dot(r.getOrigin(), _normal))/Vector3f::dot(r.getDirection(), _normal);
    if (t <= tmin) {
        return false;
    }

    if (t < h.getT()) {
        h.set(t, this->material, _normal);
        return true;
    }
    // END STARTER
    return false;

    return false;
}
bool Triangle::intersect(const Ray &r, float tmin, Hit &h) const 
{
    // TODO implement

    float ax = _v[0].x(), bx = _v[1].x(), cx = _v[2].x();
    float ay = _v[0].y(), by = _v[1].y(), cy = _v[2].y();
    float az = _v[0].z(), bz = _v[1].z(), cz = _v[2].z();
    float Rdx = r.getDirection().x(), Rdy = r.getDirection().y(), Rdz = r.getDirection().z();
    float Rox = r.getOrigin().x(), Roy = r.getOrigin().y(), Roz = r.getOrigin().z();
    Matrix3f A(ax-bx, ax-cx, Rdx, ay-by, ay-cy, Rdy, az-bz, az-cz, Rdz);
    Vector3f B(ax-Rox, ay-Roy, az-Roz);

    Vector3f sol = A.inverse() * B;

    float beta = sol.x(), gamma = sol.y(), t = sol.z();
    float alpha = 1-beta-gamma;

    if (alpha<0 || beta<0 || gamma<0) {
        return false;
    }

    if (t <= tmin) {
        return false;
    }

    if (t < h.getT()) {
        Vector3f normal = alpha * _normals[0] + beta * _normals[1] + gamma * _normals[2];
        h.set(t, this->material, normal.normalized());
        return true;
    }

    return false;
}


Transform::Transform(const Matrix4f &m,
    Object3D *obj) : _object(obj) {
    // TODO implement Transform constructor
    _m = m;
}
bool Transform::intersect(const Ray &r, float tmin, Hit &h) const
{
    // TODO implement
    Matrix4f invm = _m.inverse();
    Ray newRay((invm*Vector4f(r.getOrigin(), 1)).xyz(), (invm*Vector4f(r.getDirection(), 0)).xyz());
    Hit h2;
    h2.set(h.getT(), h.getMaterial(), h.getNormal());
    if (_object->intersect(newRay, tmin/_m.determinant(), h2)) {
        Vector3f normal = (invm.transposed() * Vector4f(h2.getNormal(), 0)).xyz().normalized();
        h.set(h2.getT(), h2.getMaterial(), normal);
        return true;
    }

    return false;
}