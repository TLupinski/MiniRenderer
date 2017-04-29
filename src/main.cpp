#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const int width  = 800;
const int height = 800;
const int depth = 255;

Model *model = NULL;
int *zbuffer = NULL;
Vec3f light_dir(0,0,-1);
Vec3f camera(0,0,5);
Vec3f eye(1,1,3);
Vec3f center(0,0,0);
Matrix ModelView;

Vec3f matrixToVector(Matrix m)
{
    return Vec3f(m[0][0]/m[3][0],m[1][0]/m[3][0],m[2][0]/m[3][0]);
}

Matrix vectorToMatrix(Vec3f v)
{
    Matrix m = Matrix::identity(4);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.f;
    return m;
}

Matrix viewport(int x, int y, int w, int h) {
    Matrix m = Matrix::identity(4);
    m[0][3] = x+w/2.f;
    m[1][3] = y+h/2.f;
    m[2][3] = depth/2.f;

    m[0][0] = w/2.f;
    m[1][1] = h/2.f;
    m[2][2] = depth/2.f;
    return m;
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
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

void triangle(Vec3f *pts, Vec2i *uv, TGAImage &image, float intensity, float *zbuffer)
{
    Vec2f bboxmin(std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
    Vec2i bboxmin_uv(std::numeric_limits<int>::max(),std::numeric_limits<int>::max());
    Vec2i bboxmax_uv(-std::numeric_limits<int>::max(),-std::numeric_limits<int>::max());
    Vec2f limit(image.get_width()-1,image.get_height()-1);
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::max(0.f,      std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(limit[j], std::max(bboxmax[j], pts[i][j]));
            bboxmin_uv[j] = std::min(bboxmin_uv[j], uv[i][j]);
            bboxmax_uv[j] = std::max(bboxmax_uv[j], uv[i][j]);
        }
    }
    Vec3f P;
    Vec2i uvP;
    uvP.x = bboxmin_uv.x;
    uvP.y = bboxmin_uv.y;
    for (P.x=(int)bboxmin.x; P.x<=bboxmax.x; P.x++) {
        uvP.y = bboxmin_uv.y;
        for (P.y=(int)bboxmin.y; P.y<=bboxmax.y; P.y++)
        {
            Vec3f barycentre = barycentric(pts[0], pts[1], pts[2], P);
            if (barycentre.x >= 0 && barycentre.y >= 0  && barycentre.z >= 0)
            {
                P.z = 0;
                for (int i = 0; i <3 ; i++)
                {
                    P.z += pts[i].z*barycentre[i];
                }
                if (zbuffer[(int)(P.x + P.y*width)] < P.z)
                {
                    TGAColor color = model->diffuse(uvP);
                    color.r *= intensity;
                    color.g *= intensity;
                    color.b *= intensity;
                    zbuffer[int(P.x+P.y*width)] = P.z;
                    image.set(P.x, P.y, color);
                }
            }
            uvP.y++;
        }
        uvP.x++;
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}

void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye-center).normalize();
    Vec3f x = cross(up,z).normalize();
    Vec3f y = cross(z,x).normalize();
    Matrix Minv = Matrix::identity(4);
    Matrix Tr   = Matrix::identity(4);
    for (int i=0; i<3; i++) {
        Minv[0][i] = x[i];
        Minv[1][i] = y[i];
        Minv[2][i] = z[i];
        Tr[i][3] = -center[i];
    }
    ModelView = Minv*Tr;
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    float *zbuffer = new float[width*height];

    Matrix projection = Matrix::identity(4);
    projection[3][2] = -1.f/camera.z;
    Matrix viewPort = viewport(width/8, height/8, width*3/4, height*3/4);
    ModelView = Matrix::identity(4);

    Vec3f up(0,10,0);
    lookat(eye,center,up);

    TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f pts[3], txt[3], world_coords[3];
        for (int i=0; i<3; i++)
        {
            Vec3f v = model->vert(face[i]);
            world_coords[i] = v;
            pts[i] = matrixToVector(viewPort*projection*ModelView*vectorToMatrix(v));
        }
        Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
        n.normalize();
        float intensity = n*light_dir;
        if (intensity>0) {
            Vec2i uv[3];
            for (int k=0; k<3; k++) {
                uv[k] = model->uv(i, k);
            }
            triangle(pts, uv, image, intensity, zbuffer);
        }
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

