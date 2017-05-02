#ifndef __OURGL_H__
#define __OURGL_H__

#include "tgaimage.h"
#include "geometry.h"
#include "model.h"

const int width  = 800;
const int height = 800;
const int depth = 255;

extern Matrix ModelView;
extern Matrix Viewport;
extern Matrix Projection;

void viewport(int x, int y, int w, int h, int depth);
void projection(float coeff=0.f); // coeff = -1/c
void lookat(Vec3f eye, Vec3f center, Vec3f up);

struct IShader {
    virtual ~IShader();
    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};

void triangle(Vec4f *pts, IShader &shader, TGAImage &image, float *zbuffer, Model *model);

#endif // __OURGL_H__
