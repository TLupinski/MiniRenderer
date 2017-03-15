#ifndef MODEL_H
#define MODEL_H

class model
{
private:
    std::vector<Vec3f> vtxs;
    std::vector<std::vector<int> > faces;
public:
    model();
};

#endif // MODEL_H
