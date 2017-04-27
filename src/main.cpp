#include <vector>
#include <cmath>
#include <cstdlib>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green   = TGAColor(0, 255,  0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color) {
    bool steep = false;
    float x0,y0,x1,y1;
    x0 = t0.x;
    y0 = t0.y;
    x1 = t1.x;
    y1 = t1.y;
    if (std::abs(x0-x1)<std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x=x0; x<=x1; x++) {
        float t = (x-x0)/(float)(x1-x0);
        int y = y0*(1.-t) + y1*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
    if (t0.y == t1.y && t1.y == t2.y){
        line(t0,t1,image,color);
        line(t0,t2,image,color);
    } else {
        if (t0.y>t1.y) std::swap(t0, t1);
        if (t0.y>t2.y) std::swap(t0, t2);
        if (t1.y>t2.y) std::swap(t1, t2);
        int y, ymin = (int) t0.y, ymoy = (int) t1.y, ymax = (int) t2.y;
        int long_height = t2.y - t0.y;
        int short_height = t1.y - t0.y+1;
        if (short_height != 0)
        {
            for (y = ymin; y <= ymoy; y++)
            {
                float long_pourcent = (float)(y-ymin)/long_height;
                float short_pourcent = (float)(y-ymin)/short_height;
                Vec2i pos1 = t0 + (t2 - t0)*long_pourcent;
                pos1.y = y;
                Vec2i pos2 = t0 + (t1 - t0)*short_pourcent;
                pos2.y = y;
                line(pos1,pos2,image,color);
            }
        }
        short_height = t2.y - t1.y+1;
        if (short_height != 0)
        {
            for (y = ymoy; y <= ymax; y++)
            {
                float long_pourcent = (float)(y-ymin)/long_height;
                float short_pourcent = (float)(y-ymoy)/short_height;
                Vec2i pos1 = t0 + (t2 - t0)*long_pourcent;
                pos1.y = y;
                Vec2i pos2 = t1 + (t2 - t1)*short_pourcent;
                pos2.y = y;
                line(pos1,pos2,image,color);
            }
        }
    }
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    Vec3f light_dir(0,0,-1);


    TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j=0; j<3; j++) {
            Vec3f v = model->vert(face[j]);
            world_coords[j] = v;
            screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
        }
        Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
        n.normalize();
        float intensity = n*light_dir;
        if (intensity>0) {
            triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
        }
    }
    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

