#include "pendulumsystem.h"

#include <cassert>
#include <iostream>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float drag = -0.1f;
const Vector3f gravity(0, -9.81f, 0);


PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
    // TODO 4.3 Extend to multiple particles
    m_vVecState.push_back(Vector3f(0, 0, 0));
    m_vVecState.push_back(Vector3f(0, 0, 0));
    masses.push_back(1);
    for (int i=0; i<NUM_PARTICLES-1; i++) {
        m_vVecState.push_back(Vector3f(rand_uniform(-0.5f, 0.5f), rand_uniform(-2, -1), 0));
        m_vVecState.push_back(Vector3f(0, 0, 0));
        masses.push_back(1);
    }
    for (int i=0; i<NUM_PARTICLES; i++) {
        for (int j=i+1; j<NUM_PARTICLES; j++) {
            springs.push_back(std::make_pair(i, j));
            spring_ks.push_back(3);
            spring_rs.push_back(rand_uniform(1, 3));
        }
    }
//    for (auto t:m_vVecState) {t.print();}
    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f(state.size());
    // TODO 4.1: implement evalF
    //  - gravity
    //  - viscous drag
    //  - springs
    for (int i=0; 2*i<state.size(); i++) {
        f[2*i] = state[2*i+1];
        f[2*i+1] = drag * state[2*i+1] / masses[i];
    }
    for (int i=0; i<springs.size(); i++) {
        int a = springs[i].first, b = springs[i].second;
        float k = spring_ks[i], r = spring_rs[i];
        Vector3f disp = state[2*a]-state[2*b];
        Vector3f force = -k * (disp.abs() - r) * disp/disp.abs();
        f[2*a+1] += force / masses[a];
        f[2*b+1] -= force / masses[b];
    }
    f[0] = Vector3f(0, 0, 0);
    f[1] = Vector3f(0, 0, 0);
    for (auto v:f) {v.print();}
    std::cout<<"\n";
    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3
    for (int i=0; 2*i<m_vVecState.size(); i++) {
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2*i]));
        drawSphere(0.075f, 10, 10);
    }
    // example code. Replace with your own drawing  code
//    gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
//    drawSphere(0.075f, 10, 10);
}
