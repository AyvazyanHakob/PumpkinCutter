#pragma once
#include <array>
#include <cmath>
#include <fstream>

template<typename T>
T NULLREFERENCE;

typedef std::array<int, 3> Triangle;

extern const double PI = 3.14159265358979323846;
extern const double eps = 1e-6;

template<class T>
struct Point {
    T x, y, z;
};
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

template<class T>
bool operator==(const Point<T>& a, const Point<T>& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}





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
    double side = (O-A)*((B-A)^(C-A));
    if (side < -eps) return -1;
    if (side > +eps) return +1;
    return 0;
}
