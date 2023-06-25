#include "Vector2.h"
#include <cmath>

Vector2::Vector2()
{
    x = 0;
    y = 0;
}


Vector2::Vector2(float x, float y)
{
    this->set(x, y);
}

void Vector2::set(float x, float y)
{
    this->x = x;
    this->y = y;
}

void Vector2::set(const Vector2& position)
{
    this->x = position.x;
    this->y = position.y;
}

Vector2 Vector2::operator+(const Vector2& vec) const
{
    return Vector2(
        x + vec.x,
        y + vec.y
    ); 
}

Vector2 Vector2::operator-(const Vector2& vec) const
{
    return Vector2(
        x - vec.x,
        y - vec.y
    ); 
}

Vector2 Vector2::operator-() const
{
    return Vector2(
        -x,
        -y
    ); 
}

void Vector2::operator+=(const Vector2& vec)
{
    x += vec.x;
    y += vec.y;
}

void Vector2::operator-=(const Vector2& vec)
{
    x -= vec.x;
    y -= vec.y;
}

void Vector2::operator*=(float scalar)
{
    x *= scalar;
    y *= scalar;
}


Vector2 Vector2::operator*(float scalar) const
{
    return Vector2(
        x * scalar,
        y * scalar
    ); 
}

Vector2 Vector2::operator/(float scalar) const
{
    return Vector2(
        x / scalar,
        y / scalar
    ); 
}
bool Vector2::operator==(const Vector2& vec) const
{
    return (x == vec.x) && (y == vec.y);
}

bool Vector2::operator!=(const Vector2& vec) const
{
    return (x != vec.x) && (y != vec.y);
}

float Vector2::magnitude()
{
    return sqrt((x*x) + (y*y));
}

Vector2 Vector2::normalize()
{
    float mag = magnitude();

    if(mag == 0)
        return Vector2(0,0);

    return *this / magnitude();
}

Vector2 Vector2::perpendicular()
{
    return Vector2(-y, x);
}

Vector2 Vector2::normal()
{
    //Vector that is perpendicular to this vector
    return perpendicular().normalize();
}

float Vector2::dot(Vector2& a, Vector2& b)
{
    return (a.x * b.x) + (a.y * b.y);
}

float Vector2::cross(Vector2& a, Vector2& b)
{
    return a.x * b.y - a.y * b.x;
}

Vector2 Vector2::cross(float& s, Vector2& a)
{
    return Vector2(-s * a.y, s * a.x);
}

Vector2 Vector2::cross(Vector2& a, float& s)
{
    return Vector2(s * a.y, -s * a.x);
}

float Vector2::distance(const Vector2& a, const Vector2& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;

    return sqrt((dx * dx) + (dy * dy));
}

float Vector2::distanceSquared(const Vector2& a, const Vector2& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;

    return (dx * dx) + (dy * dy);
}

Vector2 Vector2::rotate(float radians)
{
    //x' = x cos θ − y sin θ
    //y' = x sin θ + y cos θ
    float cosAngle = std::cos(radians);
    float sinAngle = std::sin(radians);

    Vector2 rotatedVector = Vector2(
        (x * cosAngle) - (y * sinAngle), 
        (x * sinAngle) + (y * cosAngle));

    return rotatedVector;
}

float Vector2::length()
{
    return std::sqrt((x * x) + (y * y));
}

float Vector2::lengthSquared()
{
    return (x * x) + (y * y);
}

bool Vector2::nearlyEqual(const Vector2& a, const Vector2& b, float epsilon)
{
    return Math::nearlyEqual(a.x, b.x, epsilon) && Math::nearlyEqual(a.y, b.y, epsilon);
}