#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1
    std::vector<Vector3f> X = particleSystem->getState();
    std::vector<Vector3f> f = particleSystem->evalF(X);
    std::vector<Vector3f> newX;
    for (int i=0; i<X.size(); i++) {
        newX.push_back(X[i] + stepSize * f[i]);
    }
    particleSystem->setState(newX);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1
    std::vector<Vector3f> X = particleSystem->getState();
    std::vector<Vector3f> f0 = particleSystem->evalF(X);
    std::vector<Vector3f> nextX;
    for (int i=0; i<X.size(); i++) {
        nextX.push_back(X[i]+stepSize * f0[i]);
    }
    std::vector<Vector3f> f1 = particleSystem->evalF(nextX);
    std::vector<Vector3f> newX;
    for (int i=0; i<X.size(); i++) {
        newX.push_back(X[i] + 0.5 * stepSize * (f0[i] + f1[i]));
    }
    particleSystem->setState(newX);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
    std::vector<float> ws={0.5, 0.5, 1};
    std::vector<Vector3f> X = particleSystem->getState();
    std::vector<std::vector<Vector3f>> ks;
    ks.push_back(particleSystem->evalF(X));
    for (int i=0; i<3; i++) {
        std::vector<Vector3f> nextX;
        for (int j=0; j<X.size(); j++) {
            nextX.push_back(X[j] + ws[i] * stepSize * ks.back()[j]);
        }
        ks.push_back(particleSystem->evalF(nextX));
    }
    std::vector<Vector3f> newX;
    for (int i=0; i<X.size(); i++) {
        newX.push_back(X[i] + (stepSize/6) * (ks[0][i] + 2*ks[1][i] + 2*ks[2][i] + ks[3][i]));
    }
    particleSystem->setState(newX);
}

