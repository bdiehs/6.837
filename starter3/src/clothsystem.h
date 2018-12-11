#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vector>
#include "tuple.h"

#include "particlesystem.h"

typedef tuple< unsigned, 3 > Tup3u;
typedef tuple< unsigned, 2 > Tup2u;
struct AABB{
    // type 1
    int type;
    AABB* first;
    AABB* second;
    Tup3u t1;
    Tup3u t2;

    AABB(int t, AABB* f, AABB* s) {
        type = t;
        first=f;
        second=s;
    }

    AABB(Tup3u tt1, Tup3u tt2) {
        type=2;
        t1=tt1;
        t2=tt2;
    }

};

class ClothSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    ClothSystem();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

    // inherits
    // std::vector<Vector3f> m_vVecState;
    std::vector<std::pair<int, int>> springs;
    std::vector<float> spring_ks;
    std::vector<float> spring_rs;

    std::vector< Vector3f > VN;
    std::vector< Tup3u > VF;
    std::vector< Tup2u > VE;

    void recomputeNormals();
    void timeStep(float h);

private:
    void addSpring(int ind1, int ind2, float k, float r);
    void sphereColl(std::vector<Vector3f> &newXV, float stepSize);
    void floorColl(std::vector<Vector3f> &newXV, float stepSize);
    void selfColl(std::vector<Vector3f> &newXV, std::vector<Vector3f> &avg_V, float stepSize);
};


#endif
