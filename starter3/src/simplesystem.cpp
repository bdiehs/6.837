#include "simplesystem.h"

#include "camera.h"
#include "vertexrecorder.h"


SimpleSystem::SimpleSystem()
{
    // TODO 3.2 initialize the simple system
    m_vVecState.push_back(Vector3f(1, 0, 0));
}

std::vector<Vector3f> SimpleSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;

    // TODO 3.2: implement evalF
    // for a given state, evaluate f(X,t)
    for (Vector3f v : state) {
        f.push_back(Vector3f(-2*v.y(), 2*v.x(), 0));
    }
    return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw(GLProgram& gl)
{

    // TODO 3.2: draw the particle. 
    //           we provide code that draws a static sphere.
    //           you should replace it with your own
    //           drawing code.
    //           In this assignment, you must manage two
    //           kinds of uniforms before you draw
    //            1. Update material uniforms (color)
    //            2. Update transform uniforms
    //           GLProgram is a helper object that has
    //           methods to set the uniform state.

    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
//    Vector3f pos(1, 0, 0); //YOUR PARTICLE POSITION
//    gl.updateModelMatrix(Matrix4f::translation(pos));
//    drawSphere(0.075f, 10, 10);
    for (Vector3f v : getState()) {
        gl.updateModelMatrix(Matrix4f::translation(v));
        drawSphere(0.075, 10, 10);
    }
}
