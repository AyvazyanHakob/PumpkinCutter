#pragma once
#include <tuple>

template<class T>
struct Point3 {
    T x, y, z;
};
template<class T>
bool operator<(const Point3<T>& a, const Point3<T>& b) {
    return std::make_tuple(a.x, a.y, b.z) < std::make_tuple(b.x, b.y, b.z);
}
template<class T>
bool operator==(const Point3<T>& a, const Point3<T>& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}
template<class T>
Point3<T> operator+(const Point3<T>& a, const Point3<T>& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
template<class T>
Point3<T> operator-(const Point3<T>& a, const Point3<T>& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
template<class T>
Point3<T> operator*(double k, const Point3<T>& a) {
    return {a.x*k, a.y*k, a.z*k};
}
template<class T>
Point3<T> operator^(const Point3<T>& a, const Point3<T>& b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
template<class T>
T operator*(const Point3<T>& a, const Point3<T>& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}


template<class T>
double length(const Point3<T>& a) {
    return sqrt(a*a);
}
template<class T>
Point3<T> normalized(const Point3<T>& a) {
    return (1/length(a)) * a;
}
template<class T>
void normalize(Point3<T>& a) {
    a = (1/length(a)) * a;
}
template<class T>
Point3<T> perp(Point3<T>& a) {
    Point3<T> ret = (a ^ Point3<T>{1, 0, 0});
    if (ret == Point3<T>{0, 0, 0}) ret = (a ^ Point3<T>{0, 1, 0});
    return ret;
}
