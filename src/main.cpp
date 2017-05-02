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

Vec3f light_dir(1,1,1);
Vec3f camera(0,0,5);
Vec3f eye(1,1,3);
Vec3f center(0,0,0);
Vec3f up(0,1,0);

struct GouraudShader : public IShader {
    mat<2,3,float>varying_uv;     // written by vertex shader, read by fragment shader
    mat<4,3,float> varying_tri; // triangle coordinates (clip coordinates), written by VS, read by FS
    mat<3,3,float> varying_nrm; // normal per vertex to be interpolated by FS
    mat<3,3,float> ndc_tri;     // triangle in normalized device coordinates

    virtual Vec4f vertex(int iface, int nthvert) {

        Vec3f v =  model->vert(iface, nthvert);
        Vec4f gl_Vertex = (Viewport*Projection*ModelView*embed<4>(model->vert(iface, nthvert)));     // transform it to screen coordinates
        varying_uv.set_col(nthvert,model->uv(iface,nthvert));
        varying_nrm.set_col(nthvert, proj<3>((Projection*ModelView).invert_transpose()*embed<4>(model->normal(iface, nthvert), 0.f)));
        varying_tri.set_col(nthvert, gl_Vertex);
        ndc_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f bn = (varying_nrm*bar).normalize();
        Vec2f uv = varying_uv*bar;

        mat<3,3,float> A;
        A[0] = ndc_tri.col(1) - ndc_tri.col(0);
        A[1] = ndc_tri.col(2) - ndc_tri.col(0);
        A[2] = bn;

        mat<3,3,float> AI = A.invert();
        Vec3f i = AI * Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
        Vec3f j = AI * Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);

        mat<3,3,float> B;
        B.set_col(0,i.normalize());
        B.set_col(1,j.normalize());
        B.set_col(2,bn);

        Vec3f n = (B*model->normal(uv)).normalize();

        float diff = std::max(0.f, n*light_dir);
        color = model->diffuse(uv)*diff;
        //or (int i=0; i<3; i++) color[i] = std::min<float>(5 + color[i]*(diff + .4*spec), 255);
        return false;                             // no, we do not discard this pixel
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

