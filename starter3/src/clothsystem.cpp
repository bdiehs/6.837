#include <iostream>
#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"

 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;

const float drag = -0.2f;
const Vector3f gravity(0, -0.4f, 0);

ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
    float r = 0.4;
    int c=0;
    for (int j=0; j<H; j++) {
        for (int i=0; i<W; i++) {
            m_vVecState.push_back(Vector3f(-r*j+1.6f, 2, r*i-1.6f));
            m_vVecState.push_back(Vector3f(0, 0, 0));
            if (i<W-1) {
                addSpring(c, c+1, 20, r);
                if (i<W-2) {
                    addSpring(c, c+2, 20, 2*r);
                }
            }
            if (j<H-1) {
                addSpring(c, c+W, 20, r);
                if (j<H-2) {
                    addSpring(c, c+2*W, 20, 2*r);
                }
            }
            if (i<W-1 && j<H-1) {
                addSpring(c, c+W+1, 20, r*sqrt(2.0f));
                addSpring(c+1, c+W, 20, r*sqrt(2.0f));
            }
            c++;
        }
    }
    for (int j=0; j<H-1; j++) {
        for (int i=0; i<W-1; i++) {
            VF.push_back(Tup3u(j*W+i, j*W+i+1, (j+1)*W+i));
            VF.push_back(Tup3u(j*W+i+1, (j+1)*W+i+1, (j+1)*W+i));
            VE.push_back(Tup2u(j*W+i, j*W+i+1));
            VE.push_back(Tup2u(j*W+i+1, (j+1)*W+i));
            VE.push_back(Tup2u((j+1)*W+i, j*W+i));
        }
    }
}

void ClothSystem::addSpring(int ind1, int ind2, float k, float r) {
    springs.push_back(std::make_pair(ind1, ind2));
    spring_ks.push_back(k);
    spring_rs.push_back(r);
}

void ClothSystem::recomputeNormals() {
    VN.clear();
    for (int j=0; j<H; j++) {
        for (int i=0; i<W; i++) {
            Vector3f normal(0, 0, 0);
            for (int jj=j-1; jj<j+2; jj++) {
                for (int ii=i-1; ii<i+2; ii++) {
                    if (0<=jj && jj<H && 0<=ii && ii<W && (ii != i || jj != j)) {
                        normal += (m_vVecState[2*(j*W+i)] - m_vVecState[2*(jj*W+ii)]).normalized();
                    }
                }
            }
            normal.normalize();
            VN.push_back(normal);
        }
    }
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
//    std::vector<int> stiff_indices;
    for (int i=0; i<springs.size(); i++) {
        int a = springs[i].first, b = springs[i].second;
        float k = spring_ks[i], r = spring_rs[i];
        Vector3f disp = state[2*a]-state[2*b];
        Vector3f force = -k * (disp.abs() - r) * disp / disp.abs();
        if (disp.abs() > r * 1.1) {
            f[2 * a + 1] += 2*force;
            f[2 * b + 1] -= 2*force;
//            stiff_indices.push_back(a);
//            stiff_indices.push_back(b);
        } else {
            f[2 * a + 1] += force;
            f[2 * b + 1] -= force;
        }
    }
//    for (int ind : stiff_indices) {
//        f[2*ind+1] = Vector3f::ZERO;
//    }
    f[W*H-W+1] = Vector3f::ZERO;
//    f[W*H-1] = Vector3f::ZERO;
//
//    f[1]=Vector3f::ZERO;
//    f[2*W*H-1]=Vector3f::ZERO;

    return f;
}

// gets the rotation matrix that aligns (0, 1, 0) with v
Matrix4f rot_matrix(Vector3f& v) {
    float a=v.x(), b=v.y(), c=v.z();
    Matrix4f m(1-(a*a/(1+b)), a, -a*c/(b+1), 0, -a, b, -c, 0, -a*c/(b+1), c, 1-(c*c/(1+b)), 0, 0, 0, 0, 1);
    return m;
}



// sphere rigid object, will replace later

struct Sphere {
    std::vector< Vector3f > VV;
    std::vector< Vector3f > VN;
    std::vector< Tup3u > VF;

    Sphere() {
        float M_PIf = 3.141592f;
        int slices = 10, stacks = 10;
        float r = 0.4f;
        float phistep = M_PIf * 2 / slices;
        float thetastep = M_PIf / stacks;

        for (int vi = 0; vi < stacks; ++vi) { // vertical loop
            float theta = vi * thetastep;
            float theta_next = (vi + 1) * thetastep;

            float z = r * cosf(theta);
            float z_next = r*cosf(theta_next);
            for (int hi = 0; hi < slices; ++hi) { // horizontal loop
                float phi = hi * phistep;
                float phi_next = (hi + 1) * phistep;

                float x = r * cosf(phi) * sinf(theta);
                float y = r * sinf(phi) * sinf(theta);

                Vector3f p1(r * cosf(phi) * sinf(theta), r * sinf(phi) * sinf(theta), z);
                Vector3f p2(r * cosf(phi_next) * sinf(theta), r * sinf(phi_next) * sinf(theta), z);
                Vector3f p3(r * cosf(phi_next) * sinf(theta_next), r * sinf(phi_next) * sinf(theta_next), z_next);
                Vector3f p4(r * cosf(phi) * sinf(theta_next), r * sinf(phi) * sinf(theta_next), z_next);

                Vector3f n1 = p1.normalized();
                Vector3f n2 = p2.normalized();
                Vector3f n3 = p3.normalized();
                Vector3f n4 = p4.normalized();

                VV.push_back(p1);
                VN.push_back(n1);
                VF.push_back(Tup3u(vi*slices+hi, vi*slices+hi+1, (vi+1)*slices+hi+1));
                VF.push_back(Tup3u(vi*slices+hi, (vi+1)*slices+hi+1, (vi+1)*slices+hi));


//                rec.record(p1, n1); rec.record(p2, n2); rec.record(p3, n3);
//                rec.record(p1, n1); rec.record(p3, n3); rec.record(p4, n4);

            }
        }
    }

    void draw() {
        drawSphere(0.5, 10, 10);
//        VertexRecorder rec;
//        for (int i=0; i<(int)VF.size(); i++)
//        {
//            rec.record(VV[VF[i][0]], VN[VF[i][0]]);
//            rec.record(VV[VF[i][1]], VN[VF[i][1]]);
//            rec.record(VV[VF[i][2]], VN[VF[i][2]]);
//        }
//        rec.draw();
    }
};

void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    gl.updateModelMatrix(Matrix4f::identity());
    gl.updateMaterial(Vector3f(0.5f, 0.1f, 0.3f));
    Sphere().draw();
    const Vector3f CLOTH_COLOR(0.5f, 0.9f, 0.7f);
    gl.updateMaterial(CLOTH_COLOR);

    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
//    float w = 0.2f;
//    Vector3f O(0.4f, 1, 0);
//    gl.updateModelMatrix(Matrix4f::translation(O));
//    drawSphere(0.04f, 8, 8);
//    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, 0, 0)));
//    drawSphere(0.04f, 8, 8);
//    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, -w, 0)));
//    drawSphere(0.04f, 8, 8);
//    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(0, -w, 0)));
//    drawSphere(0.04f, 8, 8);

    for (int i=0; 2*i<m_vVecState.size(); i++) {
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2*i]));
        drawSphere(0.04f, 8, 8);
    }
    for (int i=0; i<springs.size(); i++) {
        int a=springs[i].first, b=springs[i].second;
        Vector3f cyl_norm = (m_vVecState[2*b]-m_vVecState[2*a]).normalized();
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[2*a])*rot_matrix(cyl_norm));
        drawCylinder(8, 0.01f, (m_vVecState[2*b]-m_vVecState[2*a]).abs());
    }

//    gl.enableLighting();
    gl.updateModelMatrix(Matrix4f::identity());
    VertexRecorder rec;
    for (int i=0; i<(int)VF.size(); i++)
    {
        rec.record(m_vVecState[2*VF[i][0]], VN[VF[i][0]], CLOTH_COLOR);
        rec.record(m_vVecState[2*VF[i][2]], VN[VF[i][2]], CLOTH_COLOR);
        rec.record(m_vVecState[2*VF[i][1]], VN[VF[i][1]], CLOTH_COLOR);
    }
    rec.draw();

//    gl.disableLighting();

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
//    gl.disableLighting();
//    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
//    VertexRecorder rec;
//    rec.record(O, CLOTH_COLOR);
//    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
//    rec.record(O, CLOTH_COLOR);
//    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
//
//    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
//    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
//
//    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
//    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
//    glLineWidth(3.0f);
//    rec.draw(GL_LINES);

//    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END
}

void ClothSystem::sphereColl(std::vector<Vector3f> &newXV, float stepSize) {
    std::vector<Vector3f> XV=m_vVecState;
    for (int i=0; 2*i<newXV.size(); i++) {
        Vector3f newx = newXV[2*i];
        Vector3f newv = newXV[2*i+1];
        if (newx.abs()<0.5) {
            Vector3f normal = newx.normalized();
            newXV[2*i] = normal * 0.5;
            float dv_n = -Vector3f::dot((newXV[2*i+1]-XV[2*i+1])/stepSize, normal);
            newXV[2*i+1] -= Vector3f::dot(normal, newv) * normal;
            float k_fric = 1.0f*dv_n/newXV[2*i+1].abs();
            if (k_fric<1) {
                newXV[2*i+1] -= k_fric*newXV[2*i+1];
            } else {
                newXV[2*i+1] = Vector3f::ZERO;
            }
        }
    }
    for (int i=0; i<VF.size(); i++) {
        int i1 = VF[i][0], i2=VF[i][1], i3=VF[i][2];
        Vector3f x1=newXV[2*i1], x2=newXV[2*i2], x3=newXV[2*i3];
        Vector3f n = Vector3f::cross(x1-x2, x2-x3);
        n.normalize();
        if (Vector3f::dot(x1, n)<0) {
            n=-n;
        }
        if (abs(Vector3f::dot(x1, n))<0.5) {
            Vector3f x13=x1-x3, x23=x2-x3, x43=-x3;
            Matrix2f m(Vector3f::dot(x13, x13), Vector3f::dot(x13, x23), Vector3f::dot(x13, x23), Vector3f::dot(x23, x23));
            Vector2f b(Vector3f::dot(x13, x43), Vector3f::dot(x23, x43));
            Vector2f a = m.inverse() * b;
            float w1=a.x(), w2=a.y(), w3=1-w1-w2;
            if (w1<0 || w2<0 || w3<0) {
                continue;
            }
            newXV[2*i1] += (0.5-Vector3f::dot(x1, n))*n*w1;
            newXV[2*i2] += (0.5-Vector3f::dot(x1, n))*n*w2;
            newXV[2*i3] += (0.5-Vector3f::dot(x1, n))*n*w3;

            Vector3f newv = newXV[2*i1+1]*w1 + newXV[2*i2+1]*w2 + newXV[2*i3+1]*w3;
            float I = Vector3f::dot(newv, n);
            float dv_n = -Vector3f::dot((newXV[2*i1+1]-XV[2*i1+1])*w1 + (newXV[2*i1+1]-XV[2*i1+1])*w2 + (newXV[2*i1+1]-XV[2*i1+1])*w3, n)/stepSize;
            if (I<0) {
                Vector3f totalv = -I * n / (w1 * w1 + w2 * w2 + w3 * w3);
                newXV[2 * i1 + 1] += totalv * w1;
                newXV[2 * i2 + 1] += totalv * w2;
                newXV[2 * i3 + 1] += totalv * w3;
            }
            Vector3f tanv = newv - I*n;
            float k_fric = 1.0f*dv_n/tanv.abs();
            Vector3f I_fric;
            if (k_fric<1) {
                I_fric = -k_fric*tanv;
            } else {
                I_fric = -tanv/(w1 * w1 + w2 * w2 + w3 * w3);
            }
            newXV[2 * i1 + 1] += I_fric * w1;
            newXV[2 * i2 + 1] += I_fric * w2;
            newXV[2 * i3 + 1] += I_fric * w3;
        }
    }
}

void ClothSystem::floorColl(std::vector<Vector3f> &newXV, float stepSize) {

    std::vector<Vector3f> XV=m_vVecState;
    for (int i=0; 2*i<newXV.size(); i++) {
        Vector3f newx = newXV[2*i];
        Vector3f newv = newXV[2*i+1];
        if (newx.y()<-1.0) {
            newXV[2*i][1] = -1.0f;
            float dv_n = -Vector3f::dot((newXV[2*i+1]-XV[2*i+1])/stepSize, Vector3f(0, 1, 0));
            newXV[2*i+1] -= Vector3f::dot(Vector3f(0, 1, 0), newv) * Vector3f(0, 1, 0);
            float k_fric = 0.5f*dv_n/newXV[2*i+1].abs();
            if (k_fric<1) {
                newXV[2*i+1] -= k_fric*newXV[2*i+1];
            } else {
                newXV[2*i+1] = Vector3f::ZERO;
            }
        }
    }

}

void ClothSystem::selfColl(std::vector<Vector3f> &newXV, std::vector<Vector3f> &avg_V, float stepSize) {
    std::vector<Vector3f> XV=m_vVecState;
    for (Tup3u t : VF) {
        int i1 = t[0], i2=t[1], i3=t[2];
        Vector3f x1=XV[2*i1], x2=XV[2*i2], x3=XV[2*i3];
        Vector3f v1=avg_V[i1], v2=avg_V[i2], v3=avg_V[i3];
        for (int i=0; 2*i<XV.size(); i++) {
            Vector3f x4 = XV[2*i], v4=avg_V[i];
            if (i==t[0] || i==t[1] || i==t[2]) {
                continue;
            }
            Vector3f n = Vector3f::cross(x2-x1, x3-x2);
            n.normalize();
            if (abs(Vector3f::dot(x4-x1, n)) < 0.025) {
                Vector3f x13 = x1 - x3, x23 = x2 - x3, x43 = x4 - x3;
                Matrix2f m(Vector3f::dot(x13, x13), Vector3f::dot(x13, x23), Vector3f::dot(x13, x23), Vector3f::dot(x23, x23));
                Vector2f b(Vector3f::dot(x13, x43), Vector3f::dot(x23, x43));
                Vector2f a = m.inverse() * b;
                float w1=a.x(), w2=a.y(), w3=1-w1-w2;
                if (w1<-0.04 || w2<-0.04 || w3<-0.04) {
                    continue;
                }
                Vector3f xx = w1*x1+w2*x2+w3*x3;
                Vector3f vp = w1*v1 + w2*v2 + w3*v3;
                Vector3f v = v4;
                float Ic = (v-vp).abs();
                float I_bar = Ic/(1+w1*w1+w2*w2+w3*w3);
                n = xx - x4;
                n.normalize();
                avg_V[i1] += w1*(I_bar) * n;
                avg_V[i2] += w2*(I_bar) * n;
                avg_V[i3] += w3*(I_bar) * n;
                avg_V[i] -= I_bar * n;
            }
        }
    }
//    for (int i=0; i<VE.size(); i++) {
//        for (int j=i+1; i<VE.size(); j++) {
//            int i1 = VE[i][0], i2=VE[i][1], i3=VE[j][0], i4=VE[j][1];
//            if ((i1==i3 || i1==i4) || (i2=i3 || i2==i4)) {
//                continue;
//            }
//            Vector3f x1=XV[2*i1], x2=XV[2*i2], x3=XV[2*i3], x4=XV[2*i4];
//            Vector3f v1=XV[2*i1+1], v2=XV[2*i2+1], v3=XV[2*i3+1], v4=XV[2*i4];
//            Vector3f x21=x2-x1, x43=x4-x3, x31=x3-x1;
//            if (Vector3f::cross(x21, x43).abs()<0.075) {
//                Matrix2f m(Vector3f::dot(x21, x21), -Vector3f::dot(x21, x43), -Vector3f::dot(x21, x43), Vector3f::dot(x43, x43));
//                Vector2f b(Vector3f::dot(x21, x31), -Vector3f::dot(x43, x31));
//                Vector2f a = m.inverse()*b;
//                if (a[0] < 0 || a[0]>1 || a[1]<0 || a[1]>1) {
//                    continue;
//                }
//                Vector3f n = (1-a[0])*x1+a[0]*x2 - (1-a[1])*x3-a[1]*x4;
//                n.normalized();
//                Vector3f v12 = (1-a[0])*v1+a[0]*v2, v34 = (1-a[1])*v3+a[1]*v4;
//                float Ic = (v12-v34).abs();
//            }
//        }
//    }

}

void ClothSystem::timeStep(float stepSize) {

    Sphere sphere;

    std::vector<float> ws={0.5, 0.5, 1};
    std::vector<Vector3f> XV = getState();
    std::vector<std::vector<Vector3f>> ks;
    ks.push_back(evalF(XV));
    for (int i=0; i<3; i++) {
        std::vector<Vector3f> nextXV;
        for (int j=0; j<XV.size(); j++) {
            nextXV.push_back(XV[j] + ws[i] * stepSize * ks.back()[j]);
        }
        ks.push_back(evalF(nextXV));
    }
    std::vector<Vector3f> newXV;

    // back euler
    std::vector<Vector3f> nextXV(XV);
    for (int t=0; t<10; t++) {
        std::vector<Vector3f> f_y = evalF(nextXV);
        for (int j = 0; j < XV.size(); j++) {
            nextXV[j]=nextXV[j] + stepSize*f_y[j];
        }
    }
    std::vector<Vector3f> f_next = evalF(nextXV);



    for (int i=0; i<XV.size(); i++) {
        newXV.push_back(XV[i] + (stepSize/6) * (ks[0][i] + 2*ks[1][i] + 2*ks[2][i] + ks[3][i]));
//        newXV.push_back(XV[i] + stepSize*f_next[i]);
    }

//    sphereColl(newXV, stepSize);
//    floorColl(newXV, stepSize);

    std::vector<Vector3f> avg_V;
    for (int i=0; 2*i<XV.size(); i++) {
        avg_V.push_back((newXV[2*i]-XV[2*i])/stepSize);
    }

    selfColl(newXV, avg_V, stepSize);




    for (int i=0; 2*i<newXV.size(); i++) {

        newXV[2*i]=XV[2*i]+avg_V[i]*stepSize;
        newXV[2*i+1]=avg_V[i];
        if (i==W*H/2-W/2-1) {
            newXV[2*i+1]=Vector3f::ZERO;
        }
    }

    int M=30;
    float k=stepSize/(2*M);
    for (int i=0; i<M; i++) {
        std::vector<Vector3f> u_prime = evalF(newXV);
        for (int j=0; 2*j<XV.size(); j++) {
            newXV[2*j+1] = newXV[2*j+1]+k*u_prime[2*j+1];
        }
    }

    setState(newXV);

    recomputeNormals();
}