#pragma once
struct Rect
{
    Rect(){};
    Rect(float x, float y, float w, float h)
    {
        this->x = x;
        this->y = y;
        this->w = w;
        this->h = h;
    }

    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;

};

class QuadTree
{
public:
    QuadTree(Rect boundary)
    {
        this->boundary = boundary;
    };

private:
    bool isSubdivided = false;
    Rect boundary;
};