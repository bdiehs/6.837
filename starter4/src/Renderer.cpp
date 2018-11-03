#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
{
}

void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;

    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.
    Camera* cam = _scene.getCamera();
    for (int y = 0; y < h; ++y) {
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            Ray r = cam->generateRay(Vector2f(ndcx, ndcy));

            Hit h;
            Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, h);

            image.setPixel(x, y, color);
            nimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                dimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }
    // END SOLN

    // save the files 
    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}



Vector3f
Renderer::traceRay(const Ray &r,
    float tmin,
    int bounces,
    Hit &h) const
{
    // The starter code only implements basic drawing of sphere primitives.
    // You will implement phong shading, recursive ray tracing, and shadow rays.

    // TODO: IMPLEMENT 
//    if (_scene.getGroup()->intersect(r, tmin, h)) {
//        return h.getMaterial()->getDiffuseColor();
//    } else {
//        return Vector3f(0, 0, 0);
//    };

    if (_scene.getGroup()->intersect(r, tmin, h)) {
        Vector3f I_direct = _scene.getAmbientLight() * h.getMaterial()->getDiffuseColor();
        Vector3f p = r.pointAtParameter(h.getT());
        Vector3f tolight, intensity;
        float distToLight;
        for (int i = 0; i < _scene.getNumLights(); i++) {
            Light *light = _scene.getLight(i);
            light->getIllumination(p, tolight, intensity, distToLight);

            // Compute ray from point to light source to detect shadow
            Ray shadow_ray(p, tolight);
            Hit shadow_h;
            if (_args.shadows && _scene.getGroup()->intersect(shadow_ray, 0.01, shadow_h)) {
                continue;
            }
            I_direct += h.getMaterial()->shade(r, h, tolight, intensity);
        }
        if (bounces == 0) {
            return I_direct;
        }
        Vector3f R = -2 * Vector3f::dot(h.getNormal(), r.getDirection()) * h.getNormal() + r.getDirection();    // Reflected vector
        Vector3f k_specular = h.getMaterial()->getSpecularColor();
        Hit new_h;
        Vector3f I_indirect = traceRay(Ray(p, R), 0.01, bounces-1, new_h);

        return I_direct + k_specular * I_indirect;
    } else {
        return _scene.getBackgroundColor(r.getDirection());
    };

}

