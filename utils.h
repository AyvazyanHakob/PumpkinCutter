#pragma once
#include <array>
#include <fstream>
#include <vector>
#include <iostream>

#include "point2.h"
#include "point3.h"

// NULLREFERENCE TEMPLATE
template<typename T>
T NULLREFERENCE;

// CONSTANTS
extern const double PI = 3.14159265358979323846;
extern const double eps = 1e-12;

// TYPEDEFS
typedef std::array<int, 3> Triangle;
typedef Point2<double> Point2d;
typedef Point3<double> Point3d;

// Mesh
struct Mesh {
    std::vector<Point3d> vertices;
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
            Point3d point;
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
    for (Point3 p : mesh.vertices) {
        file << "v " << p.x << " " << p.y << " " << p.z << std::endl;
    }
    for (Triangle p : mesh.polygons) {
        file << "f " << p[0]+1 << " " << p[1]+1 << " " << p[2]+1 << std::endl;
    }
    return true;
}

void drawLine(Mesh& a, Point3d v1, Point3d v2)
{
    int ind = (int)a.vertices.size();
    a.vertices.push_back(v1);
    a.vertices.push_back(v2);
    v1.x += 0.9;
    v1.z += 0.9;
    a.vertices.push_back(v1);
    a.polygons.push_back({ind, ind+1, ind+2});
}
void drawPoint(Mesh& a, Point3d v)
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
    for (Point3 p : b.vertices) {
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
bool intersect  (Point3d TA, Point3d TB, Point3d TC,
               Point3d U, Point3d V,
               Point3d& ret, double& x)
{
    Point3d O = TA;
    Point3d n = (TB-TA) ^ (TC-TA);
    if (V*n == 0) return false;
    x = ((O*n - U*n) / (V*n));
    ret = U + x*V;
    return true;
}

bool liesOnAngle(Point3d A, Point3d B, Point3d C) {
    return (A^C) * (B^C) <= 0;
}
bool liesOnTriangle(Point3d A, Point3d B, Point3d C, Point3d O) {
    return liesOnAngle(B-A, C-A, O) && liesOnAngle(A-C, B-C, O);
}
int sideRelativePlane(Point3d A, Point3d B, Point3d C, Point3d O) {
    double side = (O-A)*normalized((B-A)^(C-A));
    if (side < -eps) return -1;
    if (side > +eps) return +1;
    return 0;
}
