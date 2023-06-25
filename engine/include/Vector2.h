#pragma once
#include <core/Math.h>

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
    bool operator==(const Vector2& vec) const;
    bool operator!=(const Vector2& vec) const;
    float magnitude();
    Vector2 normalize();
    Vector2 normal();
    Vector2 perpendicular();
    Vector2 rotate(float radians);
    float length();
    float lengthSquared();
    static float cross(Vector2& a, Vector2& b);
    static Vector2 cross(Vector2& a, float& s);
    static Vector2 cross(float& s, Vector2& a);
    static float dot(Vector2& a, Vector2& b);
    static float distance(const Vector2& a, const Vector2& b);
    static float distanceSquared(const Vector2& a, const Vector2& b);
    static bool nearlyEqual(const Vector2& a, const Vector2& b, float epsilon=Math::EPSILON);

    float x = 0;
    float y = 0;
};

inline Vector2 operator* (float scalar, const Vector2 vec)
{
    return vec * scalar;
}