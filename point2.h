#pragma once
#include <vector>

template<class T>
struct Point2 {
    T x, y;
};
template<class T>
bool operator==(const Point2<T>& a, const Point2<T>& b) {
    return a.x == b.x && a.y == b.y;
}
template<class T>
Point2<T> operator+(const Point2<T>& a, const Point2<T>& b) {
    return {a.x + b.x, a.y + b.y};
}
template<class T>
Point2<T> operator-(const Point2<T>& a, const Point2<T>& b) {
    return {a.x - b.x, a.y - b.y};
}
template<class T>
Point2<T> operator*(double k, const Point2<T>& a) {
    return {a.x*k, a.y*k};
}
template<class T>
T operator^(const Point2<T>& a, const Point2<T>& b) {
    return a.x*b.y - b.x*a.y;
}
template<class T>
T operator*(const Point2<T>& a, const Point2<T>& b) {
    return a.x*b.x + a.y*b.y;
}
template<class T>
double length(const Point2<T>& a) {
    return sqrt(a*a);
}

template<class T>
double intersection(Point2<T> A, Point2<T> B, Point2<T> C, Point2<T> D) {
    return ((C - A) ^ (D - C)) / ((B - A) ^ (D - C));
}
template<int S1, int E1, int S2, int E2, class T>
bool intersect(Point2<T> A, Point2<T> B, Point2<T> C, Point2<T> D) {
    bool ans = true;
    double i = intersection(A, B, C, D);
    if (S1 == 0) ans &= (i >= 0);
    else if (S1 == 1) ans &= (i > 0);
    if (E1 == 0) ans &= (i <= 1);
    else if (E1 == 1) ans &= (i < 1);
    double j = intersection(C, D, A, B);
    if (S2 == 0) ans &= (j >= 0);
    else if (S2 == 1) ans &= (j > 0);
    if (E2 == 0) ans &= (j <= 1);
    else if (E2 == 1) ans &= (j < 1);
    return ans;
}
template<class T>
bool intersect(Point2<T> A, Point2<T> B, Point2<T> P) {
    if ((B - A) ^ (P - A)) return false;
    return (P - A) * (P - B) <= 0;
}

template<class T>
int pointPolygonTest(std::vector<Point2<T>> polygon, Point2<T> A) {
    polygon.push_back(polygon[0]);
    Point2<T> B, C, D;
    B = A + Point2<T>{1, 0};
    int cnt = 0;
    for (int i = 0; i+1 < polygon.size(); i++) {
        Point2<T> C = polygon[i];
        Point2<T> D = polygon[i+1];
        if (intersect(C, D, A)) {
            cnt = -1;
            break;
        }
        if (C.y == D.y) continue;
        if (C.y > D.y) std::swap(C, D);
        cnt += intersect<0, 2, 0, 1>(A, B, C, D);
    }
    if (cnt == -1) return 0; // boundary
    if (cnt & 1) return +1; // inside
    return -1; // outside
}
