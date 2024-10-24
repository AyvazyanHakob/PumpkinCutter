#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <map>

const double PI = 3.14159265358979323846;
const double eps = 1e-6;

typedef std::array<int, 3> Triangle;

template<class T>
struct Point {
    T x, y, z;
};
template<class T>
bool operator<(const Point<T>& a, const Point<T>& b) {
    return std::make_tuple(a.x, a.y, a.z) < std::make_tuple(b.x, b.y, b.z);
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
    //return length(b-a) < eps;
}

struct Mesh {
    std::vector<Point<double>> vertices;
    std::vector<Triangle> polygons;
};

bool readMesh(const std::string& filename, Mesh& mesh) {
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
bool writeMesh(const std::string& filename, const Mesh& mesh) {
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

void drawLine(Mesh& a, Point<double> v1, Point<double> v2) {
    int ind = a.vertices.size();
    a.vertices.push_back(v1);
    a.vertices.push_back(v2);
    v1.x += 0.9;
    v1.z += 0.9;
    a.vertices.push_back(v1);
    a.polygons.push_back({ind, ind+1, ind+2});
}
void drawPoint(Mesh& a, Point<double> v) {
    int ind = a.vertices.size();
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

Mesh combine(const Mesh& a, const Mesh& b) {
    Mesh c = a;
    for (Point p : b.vertices) {
        c.vertices.push_back(p);
    }
    for (Triangle p : b.polygons) {
        p[0] += a.vertices.size();
        p[1] += a.vertices.size();
        p[2] += a.vertices.size();
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

std::vector<std::pair<int, int>> __UNUSED__VARIABLE__VECTOR_PAIR_INT_INT_NULLREF;
void breakMeshDown(Mesh& a, Point<double> planeA, Point<double> planeB, Point<double> planeC, std::vector<std::pair<int, int>>& cuttingPoints = __UNUSED__VARIABLE__VECTOR_PAIR_INT_INT_NULLREF) {
    cuttingPoints.clear();
    Mesh nexta;
    for (Triangle p : a.polygons) {
        Point<double> A = a.vertices[p[0]];
        Point<double> B = a.vertices[p[1]];
        Point<double> C = a.vertices[p[2]];
        Point<double> AxB, BxC, AxC;
        bool ABb, BCb, ACb;
        double x;
        ABb = intersect(planeA, planeB, planeC, A, B-A, AxB, x);
        ABb = ABb && (0 <= x && x <= 1);
        BCb = intersect(planeA, planeB, planeC, B, C-B, BxC, x);
        BCb = BCb && (0 <= x && x <= 1);
        ACb = intersect(planeA, planeB, planeC, A, C-A, AxC, x);
        ACb = ACb && (0 <= x && x <= 1);
        int ind = -1;
        std::map<Point<double>, int> indexOfVertex;
        auto addVertex = [&indexOfVertex, &nexta](Point<double> p)->int {
            if (indexOfVertex.count(p)) return indexOfVertex[p];
            nexta.vertices.push_back(p);
            return indexOfVertex[p] = nexta.vertices.size()-1;
        };
        int d1, d2;
        if (ABb && BCb) {
            ind = nexta.vertices.size();
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            d1 = addVertex(AxB);
            d2 = addVertex(BxC);
            // cuttingPoints.push_back({d1, d2});
        } else if (BCb && ACb) {
            ind = nexta.vertices.size();
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            nexta.vertices.push_back(A);
            d1 = addVertex(BxC);
            d2 = addVertex(AxC);
            // cuttingPoints.push_back({d2, d1});
        } else if (ACb && ABb) {
            ind = nexta.vertices.size();
            nexta.vertices.push_back(C);
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            d1 = addVertex(AxC);
            d2 = addVertex(AxB);
            // cuttingPoints.push_back({d1, d2});
        }
        if (ind != -1) {
            nexta.polygons.push_back({d1, ind+1, d2});
            nexta.polygons.push_back({ind+0, d1, ind+2});
            nexta.polygons.push_back({ind+2, d1, d2});
        } else {
            ind = nexta.vertices.size();
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            nexta.polygons.push_back({ind+0, ind+1, ind+2});
        }
    }
    a = nexta;
}

int sideRelativePlane(Point<double> A, Point<double> B, Point<double> C, Point<double> O) {
    return (O-A)*((B-A)^(C-A));
}

void isolateByPlane(Mesh& a, Point<double> planeA, Point<double> planeB, Point<double> planeC) {
    Mesh nexta;
    for (Triangle p : a.polygons) {
        Point<double> A = a.vertices[p[0]];
        Point<double> B = a.vertices[p[1]];
        Point<double> C = a.vertices[p[2]];
        if (sideRelativePlane(planeA, planeB, planeC, A) >= 0 &&
            sideRelativePlane(planeA, planeB, planeC, B) >= 0 &&
            sideRelativePlane(planeA, planeB, planeC, C) >= 0) {
            int ind = nexta.vertices.size();
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            nexta.polygons.push_back({ind+0, ind+1, ind+2});
        }
    }
    a = nexta;
}

void cutByPlane(Mesh& a, Point<double> planeA, Point<double> planeB, Point<double> planeC) {
    std::vector<std::pair<int, int>> ret;
    breakMeshDown(a, planeA, planeB, planeC, ret);
    if (ret.size() >= 2) {
        for (int i = 1; i < ret.size(); i++) {
            a.polygons.push_back({ret[0].first, ret[i].first, ret[i].second});
        }
    }
    isolateByPlane(a, planeA, planeB, planeC);
}

int main() {
    Mesh b;
    readMesh("C:\\Users\\c2zi6\\Desktop\\object.obj", b);
    Mesh visual;


    Point<double> origin = {-15, -10, -110};
    Point<double> direction = {30, 25, 70};
    normalize(direction);
    double cnord = 60;
    double angle = PI/6;
    Point<double> basis1 = perp(direction);
    Point<double> basis2 = basis1^direction;
    normalize(basis1);
    normalize(basis2);
    double scale = sin(angle) * cnord;
    basis1 = scale*basis1;
    basis2 = scale*basis2;
    direction = cos(angle) * cnord * direction;
    int partcnt = 7;
    std::vector<Point<double>> surface;
    for (int part = 0; part < partcnt; part++) {
        double alpha = 2*PI/partcnt * part;
        Point<double> p = sin(alpha) * basis1 + cos(alpha) * basis2;
        surface.push_back(origin + direction + p);
    }
    for (int i = 0; i < partcnt; i++) {
        Point<double> u = surface[i];
        Point<double> v = surface[(i+1)%partcnt];
        cutByPlane(b, u, v, origin);
        // continue;
        break;
        Mesh plane;
        int ind = plane.vertices.size();
        plane.vertices.push_back(origin);
        plane.vertices.push_back(u);
        plane.vertices.push_back(v);
        plane.polygons.push_back({ind+0, ind+1, ind+2});
        visual = combine(visual, plane);
        //break;
    }

    // Point<double> planeA{-80, 30, -90};
    // Point<double> planeB{30, -110, -140};
    // Point<double> planeC{90, 90, -110};
    // cutByPlane(b, planeA, planeB, planeC);
    // Mesh plane;
    // plane.vertices.push_back(planeA);
    // plane.vertices.push_back(planeB);
    // plane.vertices.push_back(planeC);
    // plane.polygons.push_back({0, 1, 2});
    // Mesh visual = combine(b, plane);
    writeMesh("C:\\Users\\c2zi6\\Desktop\\object2.obj", combine(visual, b));
    return 0;
}











