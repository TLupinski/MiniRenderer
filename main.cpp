#include "tgaimage.h"
#include <cstdlib>
#include <cmath>
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if(std::abs(x0-x1)<std::abs(y0-y1)){
        steep = true;
        std::swap(x0,y0);
        std::swap(x1,y1);
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0,y1);
    }
    int dx = x1 - x0;
    int dy = y1 - y0;
    int derror2 = std::abs(dy)*2;
    int error2 = 0;
    int y = y0;
    for (int x=x0; x<=x1; x++) {
        if (steep){
            image.set(x, y, color);
        } else {
            image.set(y,x,color);
        }
        error2 += derror2;
        if (error2 > dx){
            y += (y1>y0?1:-1);
            error2 -= dx*2;
        }
    }
    std::cerr << "Nombre d'erreurs : " << error2;
}

int main(int argc, char** argv) {
    if (2==argc) {
            model = new Model(argv[1]);
        } else {
            model = new Model("obj/african_head.obj");
        }

    TGAImage image(100, 100, TGAImage::RGB);
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
	return 0;
}

