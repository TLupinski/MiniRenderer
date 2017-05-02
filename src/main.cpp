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
TGAImage *shadowbuffer, *zbuffer;

Vec3f light_dir(1,0,1);
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
struct ShadowShader : public IShader {
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()
    mat<4,4,float> uniform_Mshadow; // transform framebuffer screen coordinates to shadowbuffer screen coordinates
    mat<2,3,float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
    mat<3,3,float> varying_tri; // triangle coordinates before Viewport transform, written by VS, read by FS

    ShadowShader(Matrix M, Matrix MIT, Matrix MS)
    {
        uniform_M = M;
        uniform_MIT = MIT;
        uniform_Mshadow =MS;
    }

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_Vertex = Viewport*Projection*ModelView*embed<4>(model->vert(iface, nthvert));
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec4f sb_p = uniform_Mshadow*embed<4>(varying_tri*bar); // corresponding point in the shadow buffer
        sb_p = sb_p/sb_p[3];
        int idx = int(sb_p[0]) + int(sb_p[1])*width; // index in the shadowbuffer array
        float shadow = shadow = .3+.7*((int)shadowbuffer->get(sb_p[0],sb_p[1]).bgra[0]<sb_p[2]+43.34); // magic coeff to avoid z-fighting
        Vec2f uv = varying_uv*bar;                 // interpolate uv for the current pixel
        Vec3f n = proj<3>(uniform_MIT*embed<4>(model->normal(uv))).normalize(); // normal
        Vec3f l = proj<3>(uniform_M  *embed<4>(light_dir        )).normalize(); // light vector
        Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
        float diff = std::max(0.f, n*l);
        TGAColor c = model->diffuse(uv);
        for (int i=0; i<3; i++) color[i] = std::min<float>(5 + c[i]*shadow*(1.2*diff + spec), 255);
        return false;
    }
};
struct DepthShader : public IShader {
    mat<3,3,float> varying_tri;

    DepthShader() : varying_tri() {}

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = Viewport*Projection*ModelView*gl_Vertex;          // transform it to screen coordinates
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f p = varying_tri*bar;
        color = TGAColor(255, 255, 255)*(p.z/depth);
        return false;
    }
};

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj"); //*/"obj/diablo3_pose.obj");
    }

    light_dir.normalize();
    lookat(light_dir, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4, depth);
    projection(0);

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage depthshadow(width, height, TGAImage::RGB);
    zbuffer = new TGAImage(width, height, TGAImage::GRAYSCALE);
    shadowbuffer = new TGAImage(width, height, TGAImage::GRAYSCALE);

    DepthShader depthshader;
    Vec4f screen_coords[3];
    for (int i=0; i<model->nfaces(); i++) {
        for (int j=0; j<3; j++) {
            screen_coords[j] = depthshader.vertex(i, j);
        }
        triangle(screen_coords, depthshader, depthshadow, *shadowbuffer, model);
    }
    shadowbuffer->flip_vertically();
    shadowbuffer->write_tga_file("shadow.tga");
    /*for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            std::cerr << (int)shadowbuffer->get(i,j).bgra[0] << " ";
        }
    }*/

    Matrix M = Viewport*Projection*ModelView;
    lookat(eye, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4, depth);
    projection(-1.f/(eye-center).norm());

    Matrix MIT = (Projection*ModelView).invert_transpose();
    Matrix SM = M*(Viewport*Projection*ModelView).invert();

    ShadowShader shader(ModelView, MIT, SM);
    int i;
    for (i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec4f pts[3];
        for (int j=0; j<3; j++)
        {
            pts[j] = shader.vertex(i, j);
        }
        triangle(pts, shader, image, *zbuffer, model);
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

