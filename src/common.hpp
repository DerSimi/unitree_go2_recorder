#pragma once

#include <cstdarg>
#include <iostream>

const std::string PREFIX = "\033[38;5;250m[\033[0m\033[38;5;45mMuJoCo Extractor\033[0m\033[38;5;250m]\033[0m";

inline void println(const char *fmt, ...)
{
    constexpr size_t BUF_SIZE = 2048;
    char buffer[BUF_SIZE];

    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, BUF_SIZE, fmt, args);
    va_end(args);

    std::cout << PREFIX << " " << buffer << std::endl;
}