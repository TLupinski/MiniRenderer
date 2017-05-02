#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"

Matrix ModelView;
Matrix Viewport;
Matrix Projection;

IShader::~IShader() {}

void viewport(int x, int y, int w, int h, int depth) {
    Viewport = Matrix::identity();
    Viewport[0][3] = x+w/2.f;
    Viewport[1][3] = y+h/2.f;
    Viewport[2][3] = depth/2.f;

    Viewport[0][0] = w/2.f;
    Viewport[1][1] = h/2.f;
    Viewport[2][2] = depth/2.f;
}

void projection(float coef)
{
    Projection = Matrix::identity();
    Projection[3][2] = coef;
}

void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye-center).normalize();
    Vec3f x = cross(up,z).normalize();
    Vec3f y = cross(z,x).normalize();
    Matrix Minv = Matrix::identity();
    Matrix Tr   = Matrix::identity();
    for (int i=0; i<3; i++) {
        Minv[0][i] = x[i];
        Minv[1][i] = y[i];
        Minv[2][i] = z[i];
        Tr[i][3] = -center[i];
    }
    ModelView = Minv*Tr;
}

Vec3f barycentric(Vec4f A, Vec4f B, Vec4f C, Vec2i P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec4f *pts, IShader &shader, TGAImage &image, float *zbuffer, Model *model)
{
    Vec2f bboxmin(std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
    Vec2f limit(image.get_width()-1,image.get_height()-1);
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::max(0.f,      std::min(bboxmin[j], pts[i][j]/pts[i][3]));
            bboxmax[j] = std::min(limit[j], std::max(bboxmax[j], pts[i][j]/pts[i][3]));
        }
    }
    Vec2i P;
    TGAColor color;
    for (P.x=(int)bboxmin.x; P.x<=(int)bboxmax.x; P.x++) {
        for (P.y=(int)bboxmin.y; P.y<=(int)bboxmax.y; P.y++) {
            Vec3f c = barycentric((pts[0]/pts[0][3]), (pts[1]/pts[1][3]), (pts[2]/pts[2][3]), P);
            float z = pts[0][2]*c.x + pts[1][2]*c.y + pts[2][2]*c.z;
            float w = pts[0][3]*c.x + pts[1][3]*c.y + pts[2][3]*c.z;
            int frag_depth = std::max(0, std::min(255, int(z/w+.5)));
            if (c.x<0 || c.y<0 || c.z<0 || zbuffer[P.x+ P.y*width] > frag_depth) continue;
            bool discard = shader.fragment(c, color);
            if (!discard)
            {
                zbuffer[P.x+ P.y*width] = frag_depth;
                image.set(P.x, P.y, color);
            }
        }
    }
}

