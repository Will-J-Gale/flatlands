#pragma once
#include <iostream>
#include <string>

inline void log(const std::string& text)
{
    std::cout << text << std::endl;
}

inline void log(const float& number)
{
    std::cout << std::to_string(number) << std::endl;
}

template<typename T>
inline void log(std::initializer_list<T> numbers)
{
    std::string text;

    for(T number : numbers)
    {
        text += std::to_string(number);
        text += " ";
    }

    log(text);
}