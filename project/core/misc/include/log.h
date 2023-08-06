#pragma once

#include <sstream>
#include <ostream>
#include <tuple>
#include <vector>
#include <array>
#include <stdexcept>
#include <unistd.h>
#include <cstdio>
#include <iostream>

static const std::string COLOR_RED = "\x1b[31m";
static const std::string COLOR_YELLOW = "\x1b[33m";
static const std::string COLOR_CYAN = "\x1b[36m";
static const std::string RESET_COLOR = "\x1b[0m";


namespace bmpf {
// форматирование простого значения
    template<typename T>
    void formatValue(std::ostream &s, T value) {
        s << value;
    }

// форматирование набора значений
    template<typename T, std::size_t N>
    void formatValue(std::ostream &s, std::array<T, N> const &arr) {
        s << "[";
        for (std::size_t i = 0; i < arr.size() - 1; ++i)
            s << arr[i] << ", ";
        if (!arr.empty())
            s << arr[arr.size() - 1];
        s << "]";
    }

// вывод простого значения
    inline void printLog(std::ostream &s) {
    }

// вывод набора значений
    template<class A1, class ... Atail>
    inline void printLog(std::ostream &s, A1 a, Atail ... tail) {
        formatValue(s, a);
        printLog(s, tail...);
    }

// информация
    template<class ... Args>
    inline void infoMsg(Args ... args) {
        std::stringstream ss;
        printLog(ss, COLOR_CYAN + "[info]" + RESET_COLOR, args..., "\n");
        auto s = ss.str();
        ssize_t t = write(STDOUT_FILENO, s.data(), s.size());
    }

// предупреждение
    template<class ... Args>
    inline void warnMsg(Args ... args) {
        std::stringstream ss;
        printLog(ss, COLOR_YELLOW + "[warn]" + RESET_COLOR, args..., "\n");
        auto s = ss.str();
        ssize_t t = write(STDOUT_FILENO, s.data(), s.size());
    }

// ошибка
    template<class ... Args>
    inline void errMsg(Args ... args) {
        std::stringstream ss;
        printLog(ss, COLOR_RED + "[error]" + RESET_COLOR, args..., "\n");
        auto s = ss.str();
        ssize_t t = write(STDOUT_FILENO, s.data(), s.size());
    }

}
