#pragma once
#include <array>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>

template<typename T>
T NULLREFERENCE;

extern const double PI = 3.14159265358979323846;
extern const double eps = 1e-12;



//          2D POINT STRUCT

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
    /*
     * Boundary  0
     * Inside   +1
     * Outside  -1
     */
    if (cnt == -1) return 0;
    if (cnt & 1) return +1;
    return -1;
}


//              3D POINT STRUCT

template<class T>
struct Point {
    T x, y, z;
};
template<class T>
    bool operator<(const Point<T>& a, const Point<T>& b) {
    return std::make_tuple(a.x, a.y, b.z) < std::make_tuple(b.x, b.y, b.z);
}
template<class T>
bool operator==(const Point<T>& a, const Point<T>& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}
template<class T>
Point<T> operator+(const Point<T>& a, const Point<T>& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
template<class T>
Point<T> operator-(const Point<T>& a, const Point<T>& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
template<class T>
Point<T> operator*(double k, const Point<T>& a) {
    return {a.x*k, a.y*k, a.z*k};
}
template<class T>
Point<T> operator^(const Point<T>& a, const Point<T>& b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
template<class T>
T operator*(const Point<T>& a, const Point<T>& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}


template<class T>
double length(const Point<T>& a) {
    return sqrt(a*a);
}
template<class T>
Point<T> normalized(const Point<T>& a) {
    return (1/length(a)) * a;
}
template<class T>
void normalize(Point<T>& a) {
    a = (1/length(a)) * a;
}
template<class T>
Point<T> perp(Point<T>& a) {
    Point<T> ret = (a ^ Point<T>{1, 0, 0});
    if (ret == Point<T>{0, 0, 0}) ret = (a ^ Point<T>{0, 1, 0});
    return ret;
}






typedef std::array<int, 3> Triangle;
struct Mesh {
    std::vector<Point<double>> vertices;
    std::vector<Triangle> polygons;
};

bool readMesh(const std::string& filename, Mesh& mesh)
{
    mesh.vertices.clear();
    mesh.polygons.clear();
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open input file" << std::endl;
        return false;
    }
    std::string cur;
    while (file >> cur) {
        if (cur == "v") {
            Point<double> point;
            file >> point.x >> point.y >> point.z;
            mesh.vertices.push_back(point);
        } else if (cur == "f") {
            Triangle triangle;
            file >> triangle[0] >> triangle[1] >> triangle[2];
            triangle[0]--;
            triangle[1]--;
            triangle[2]--;
            mesh.polygons.push_back(triangle);
        }
    }
    return true;
}
bool writeMesh(const std::string& filename, const Mesh& mesh)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open input file" << std::endl;
        return false;
    }
    for (Point p : mesh.vertices) {
        file << "v " << p.x << " " << p.y << " " << p.z << std::endl;
    }
    for (Triangle p : mesh.polygons) {
        file << "f " << p[0]+1 << " " << p[1]+1 << " " << p[2]+1 << std::endl;
    }
    return true;
}

void drawLine(Mesh& a, Point<double> v1, Point<double> v2)
{
    int ind = (int)a.vertices.size();
    a.vertices.push_back(v1);
    a.vertices.push_back(v2);
    v1.x += 0.9;
    v1.z += 0.9;
    a.vertices.push_back(v1);
    a.polygons.push_back({ind, ind+1, ind+2});
}
void drawPoint(Mesh& a, Point<double> v)
{
    int ind = (int)a.vertices.size();
    double sz = 0.2;
    a.vertices.push_back({v.x-sz, v.y-sz, v.z});
    a.vertices.push_back({v.x+sz, v.y-sz, v.z});
    a.vertices.push_back({v.x, v.y+sz, v.z});
    a.vertices.push_back({v.x, v.y, v.z+sz});
    a.polygons.push_back({ind+0, ind+1, ind+2});
    a.polygons.push_back({ind+0, ind+1, ind+3});
    a.polygons.push_back({ind+1, ind+2, ind+3});
    a.polygons.push_back({ind+2, ind+0, ind+3});
}

Mesh combine(const Mesh& a, const Mesh& b)
{
    Mesh c = a;
    for (Point p : b.vertices) {
        c.vertices.push_back(p);
    }
    for (Triangle p : b.polygons) {
        p[0] += (int)a.vertices.size();
        p[1] += (int)a.vertices.size();
        p[2] += (int)a.vertices.size();
        c.polygons.push_back(p);
    }
    return c;
}

// intersection of plane (TA, TB, TC) and line (U, V);
bool intersect  (Point<double> TA, Point<double> TB, Point<double> TC,
               Point<double> U, Point<double> V,
               Point<double>& ret, double& x)
{
    Point<double> O = TA;
    Point<double> n = (TB-TA) ^ (TC-TA);
    if (V*n == 0) return false;
    x = ((O*n - U*n) / (V*n));
    ret = U + x*V;
    return true;
}

bool liesOnAngle(Point<double> A, Point<double> B, Point<double> C) {
    return (A^C) * (B^C) <= 0;
}
bool liesOnTriangle(Point<double> A, Point<double> B, Point<double> C, Point<double> O) {
    return liesOnAngle(B-A, C-A, O) && liesOnAngle(A-C, B-C, O);
}
int sideRelativePlane(Point<double> A, Point<double> B, Point<double> C, Point<double> O) {
    double side = (O-A)*normalized((B-A)^(C-A));
    if (side < -eps) return -1;
    if (side > +eps) return +1;
    return 0;
}
