#pragma once

struct Vector2
{
    Vector2();
    Vector2(float x, float y);

    void set(float x, float y);
    void set(const Vector2& position);
    
    Vector2 operator+ (const Vector2& vec) const;
    Vector2 operator- (const Vector2& vec) const;
    Vector2 operator- () const;
    void operator+= (const Vector2& vec);
    void operator-= (const Vector2& vec);
    void operator*= (float scalar);
    Vector2 operator* (float scalar) const;
    Vector2 operator/ (float scalar) const;
    float magnitude();
    Vector2 normalize();
    Vector2 normal();
    Vector2 rotate(float radians);
    static float dot(Vector2& a, Vector2& b);
    static float distance(const Vector2& a, const Vector2& b);

    float x = 0;
    float y = 0;
};
