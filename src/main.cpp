#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

const int width  = 800;
const int height = 800;
const int depth = 255;

Model *model = NULL;

Vec3f light_dir(0,-1,1);
Vec3f camera(0,0,5);
Vec3f eye(1,1,3);
Vec3f center(0,0,0);
Vec3f up(0,1,0);

struct GouraudShader : public IShader {
    Vec2f varying_uv[3];     // written by vertex shader, read by fragment shader
    Matrix uniform_M;
    Matrix uniform_MIT;

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec3f v =  model->vert(iface, nthvert);
        Vec4f gl_Vertex = Vec4f(v.x,v.y,v.z,1); // read the vertex from .obj file
        gl_Vertex = matrixToVector(Viewport*Projection*ModelView*vectorToMatrix(gl_Vertex));     // transform it to screen coordinates
        varying_uv[nthvert] = model->uv(iface,nthvert);
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec2f uv = Vec2f(varying_uv[0][0]*bar[0]+varying_uv[1][0]*bar[1]+varying_uv[2][0]*bar[2],varying_uv[0][1]*bar[0]+varying_uv[1][1]*bar[1]+varying_uv[2][1]*bar[2]);
        Vec3f v = model->normal(uv);
        Vec3f n = projectionVector(uniform_MIT,model->normal(uv));
        Vec3f l = projectionVector(uniform_M,light_dir);
        Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
        float diff = std::max(0.f, n*l);
        color = model->diffuse(uv);
        for (int i=0; i<3; i++) color[i] = std::min<float>(5 + color[i]*(diff + .4*spec), 255);
        return false;                              // no, we do not discard this pixel
    }

    void calcul_uniform()
    {
        uniform_M = Projection*ModelView;
        uniform_MIT = uniform_M.invert_transpose();
    }
};

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    lookat(eye, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4, depth);
    projection(-1.f/(eye-center).norm());
    light_dir.normalize();

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);


    GouraudShader shader;

    shader.calcul_uniform();
    int i;
    for (i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec4f pts[3];
        for (int j=0; j<3; j++)
        {
            pts[j] = shader.vertex(i, j);
        }
        triangle(pts, shader, image, zbuffer, model);
    }
    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

