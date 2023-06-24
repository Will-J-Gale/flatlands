#pragma once
#include <iostream>
#include <string>

class Vector2;

inline void loginfo(const std::string& text)
{
    std::cout << text << std::endl;
}

inline void loginfo(const float& number)
{
    std::cout << std::to_string(number) << std::endl;
}

template<typename T>
inline void loginfo(std::initializer_list<T> numbers)
{
    std::string text;

    for(T number : numbers)
    {
        text += std::to_string(number);
        text += " ";
    }

    loginfo(text);
}

inline void loginfo(Vector2 vector)
{
    std::string text;

    text += std::to_string(vector.x);
    text += " ";
    text += std::to_string(vector.y);
    text += " ";

    loginfo(text);
}