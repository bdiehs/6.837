#include <iostream>
#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"

 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;

const float drag = -0.1f;
const Vector3f gravity(0, -0.1f, 0);

ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
    int c=0;
    for (int j=0; j<H; j++) {
        for (int i=0; i<W; i++) {
            m_vVecState.push_back(Vector3f(0.25f*i, 0.25f*j, 0));
            m_vVecState.push_back(Vector3f(0, 0, 0));
            if (i<W-1) {
                addSpring(c, c+1, 10, 0.25);
                if (i<W-2) {
                    addSpring(c, c+2, 10, 0.5);
                }
            }
            if (j<H-1) {
                addSpring(c, c+W, 10, 0.25);
                if (j<H-2) {
                    addSpring(c, c+2*W, 10, 0.5);
                }
            }
            if (i<W-1 && j<H-1) {
                addSpring(c, c+W+1, 10, sqrt(0.125f));
                addSpring(c+1, c+W, 10, sqrt(0.125f));
            }
            c++;
        }
    }
}

void ClothSystem::addSpring(int ind1, int ind2, float k, float r) {
    springs.push_back(std::make_pair(ind1, ind2));
    spring_ks.push_back(k);
    spring_rs.push_back(r);
}

std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f(state.size());
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
    for (int i=0; 2*i<state.size(); i++) {
        f[2*i] = state[2*i+1];
        f[2*i+1] = gravity + drag * state[2*i+1];
    }
    for (int i=0; i<springs.size(); i++) {
        int a = springs[i].first, b = springs[i].second;
        float k = spring_ks[i], r = spring_rs[i];
        Vector3f disp = state[2*a]-state[2*b];
        Vector3f force = -k * (disp.abs() - r) * disp/disp.abs();
        f[2*a+1] += force;
        f[2*b+1] -= force;
    }
    f[2*W*H-2*W+1] = Vector3f::ZERO;
    f[2*W*H-1] = Vector3f::ZERO;
    return f;
}

// gets the rotation matrix that aligns (0, 1, 0) with v
Matrix4f rot_matrix(Vector3f& v) {
    float a=v.x(), b=v.y(), c=v.z();
    Matrix4f m(1-(a*a/(1+b)), a, -a*c/(b+1), 0, -a, b, -c, 0, -a*c/(b+1), c, 1-(c*c/(1+b)), 0, 0, 0, 0, 1);
    return m;
}

void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
    float w = 0.2f;
    Vector3f O(0.4f, 1, 0);
    gl.updateModelMatrix(Matrix4f::translation(O));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, 0, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, -w, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(0, -w, 0)));
    drawSphere(0.04f, 8, 8);

    for (int i=0; 2*i<m_vVecState.size(); i++) {
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2*i]));
        drawSphere(0.04f, 8, 8);
    }
    for (int i=0; i<springs.size(); i++) {
        int a=springs[i].first, b=springs[i].second;
//        std::cout << a << " " << b << "\n";
//        m_vVecState[2*a].print(); m_vVecState[2*b].print();
        Vector3f cyl_norm = (m_vVecState[2*b]-m_vVecState[2*a]).normalized();
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2*a])*rot_matrix(cyl_norm));
        drawCylinder(8, 0.01f, (m_vVecState[2*b]-m_vVecState[2*a]).abs());
    }
//    std::cout<<"\n\n\n\n\n";
    // EXAMPLE: This shows you how to render lines to debug the spring system.
    //
    //          You should replace this code.
    //
    //          Since lines don't have a clearly defined normal, we can't use
    //          a regular lighting model.
    //          GLprogram has a "color only" mode, where illumination
    //          is disabled, and you specify color directly as vertex attribute.
    //          Note: enableLighting/disableLighting invalidates uniforms,
    //          so you'll have to update the transformation/material parameters
    //          after a mode change.
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
    glLineWidth(3.0f);
    rec.draw(GL_LINES);

    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END
}

