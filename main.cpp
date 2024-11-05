#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

template<typename T>
T NULLREFERENCE;

typedef std::array<int, 3> Triangle;

const double PI = 3.14159265358979323846;
const double eps = 1e-10;

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

std::vector<int> polygonByEdges(std::vector<std::pair<int, int>>& a) {
    int n = (int)a.size();
    int maxind = 0;
    for (auto[u, v] : a) {
        maxind = std::max(maxind, u);
        maxind = std::max(maxind, v);
    }
    // std::cout << maxind << std::endl;
    // return {};
    std::vector<std::vector<int>> gp(maxind+1);
    std::vector<bool> vis(maxind+1);
    for (auto[u, v] : a) {
        gp[u].push_back(v);
        gp[v].push_back(u);
    }
    std::vector<int> ret;
    int u = a[0].first;
    while (true) {
        vis[u] = true;
        ret.push_back(u);
        for (int v : gp[u]) {
            if (vis[v]) continue;
            u = v;
            break;
        }
        if (vis[u]) break;
    }
    return ret;
}



void intersectByPlane(Mesh& a, Point<double> planeA, Point<double> planeB, Point<double> planeC,
                                std::vector<int>& intersectionPoints = NULLREFERENCE<std::vector<int>>) {
    intersectionPoints.clear();
    std::vector<std::pair<int, int>> intersectionEdges;
    std::vector<int> added;
    Mesh nexta;
    for (Triangle p : a.polygons)
    {
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
        std::vector<int> buffer;
        int ind = -1;
        auto addVertex = [&added, &nexta, &buffer](Point<double> p)->int {
            int nearest = -1;
            double nearestLength = std::numeric_limits<double>::max();
            for (int ind : added) {
                double currentLength = length(nexta.vertices[ind] - p);
                if (currentLength < nearestLength) {
                    nearestLength = currentLength;
                    nearest = ind;
                }
            }
            if (nearestLength < 1e-8) {
                return nearest;
            }
            nexta.vertices.push_back(p);
            buffer.push_back((int)nexta.vertices.size()-1);
            return buffer.back();
        };
        auto emptyBuffer = [&added, &buffer]() {
            for (int x : buffer) {
                added.push_back(x);
            }
            buffer.clear();
        };
        int d1, d2;
        if (ABb && BCb) {
            ind = (int)nexta.vertices.size();
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            d1 = addVertex(AxB);
            d2 = addVertex(BxC);
            emptyBuffer();
            if (sideRelativePlane(planeA, planeB, planeC, B) > 0) {
                intersectionEdges.push_back({d1, d2});
            } else {
                intersectionEdges.push_back({d2, d1});
            }
        } else if (BCb && ACb) {
            ind = (int)nexta.vertices.size();
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            nexta.vertices.push_back(A);
            d1 = addVertex(BxC);
            d2 = addVertex(AxC);
            emptyBuffer();
            if (sideRelativePlane(planeA, planeB, planeC, C) > 0) {
                intersectionEdges.push_back({d1, d2});
            } else {
                intersectionEdges.push_back({d2, d1});
            }
        } else if (ACb && ABb) {
            ind = (int)nexta.vertices.size();
            nexta.vertices.push_back(C);
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            d1 = addVertex(AxC);
            d2 = addVertex(AxB);
            emptyBuffer();
            if (sideRelativePlane(planeA, planeB, planeC, A) > 0) {
                intersectionEdges.push_back({d1, d2});
            } else {
                intersectionEdges.push_back({d2, d1});
            }
        }
        if (ind != -1) {
            nexta.polygons.push_back({d1, ind+1, d2});
            nexta.polygons.push_back({ind+0, d1, ind+2});
            nexta.polygons.push_back({ind+2, d1, d2});
        } else {
            ind = (int)nexta.vertices.size();
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            nexta.polygons.push_back({ind+0, ind+1, ind+2});
        }
    }
    a = nexta;
    intersectionPoints = polygonByEdges(intersectionEdges);
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
            int ind = (int)nexta.vertices.size();
            nexta.vertices.push_back(A);
            nexta.vertices.push_back(B);
            nexta.vertices.push_back(C);
            nexta.polygons.push_back({ind+0, ind+1, ind+2});
        }
    }
    a = nexta;
}

Mesh visual;

struct NODE {
    int val;
    NODE* next;
    NODE* prev;
};
std::vector<Triangle> triangulize(std::vector<int> polygon, const std::vector<Point<double>>& points, Point<double> planeNormal) {
    NODE* start = new NODE{polygon[0], nullptr, nullptr};
    NODE* cur = start;
    for (int i = 0; i+1 < polygon.size(); i++) {
        cur->next = new NODE{polygon[(i+1)%(int)polygon.size()], nullptr, cur};
        cur = cur->next;
    }
    cur->next = start;
    start->prev = cur;

    std::vector<Triangle> ret;
    for (int i = 0; i < 10000; i++) {
        int p1 = cur->prev->val;
        int p2 = cur->val;
        int p3 = cur->next->val;
        Point<double> A = points[cur->prev->val];
        Point<double> B = points[cur->val];
        Point<double> C = points[cur->next->val];
        if (sideRelativePlane(A-planeNormal, B-planeNormal, C-planeNormal, planeNormal) < 0) {
            ret.push_back({p1, p2, p3});

            cur->prev->next = cur->next;
            cur->next->prev = cur->prev;

            NODE* tmp = cur->next;
            delete cur;
            cur = tmp;
        } else cur = cur->next;
        // srand(time(0));
        // int c = rand()%20000;
        // for (int i = 0; i < c; i++) cur = cur->next;
    }
    return ret;
}

void cutByPlane(Mesh& a, Point<double> planeA, Point<double> planeB, Point<double> planeC, int tp) {
    std::cout << "AAA" << std::endl;
    std::vector<int> polygon;
    intersectByPlane(a, planeA, planeB, planeC, polygon);

    std::cout << "BBB" << std::endl;
    std::vector<Triangle> triangles = triangulize(polygon, a.vertices, (planeB-planeA)^(planeC-planeA));
    for (auto p : triangles) {
        a.polygons.push_back({p[0], p[1], p[2]});
    }

    std::cout << "CCC" << std::endl;
    isolateByPlane(a, planeA, planeB, planeC);
    std::cout << "Finished" << std::endl;
}

int main() {
    Mesh b;
    readMesh("C:\\Users\\c2zi6\\Desktop\\object.obj", b);



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
    int partcnt = 20;
    std::vector<Point<double>> surface;
    for (int part = 0; part < partcnt; part++) {
        double alpha = 2*PI/partcnt * part;
        Point<double> p = sin(alpha) * basis1 + cos(alpha) * basis2;
        surface.push_back(origin + direction + p);
    }

    for (int i = 0; i < partcnt; i++) {

        std::cout << "cutting part " << i << std::endl;


        Point<double> u = surface[i];
        Point<double> v = surface[(i+1)%partcnt];
        cutByPlane(b, u, v, origin, i);

        //break;
        if (i == 20) break;
        continue;
        Mesh plane;
        int ind = (int)plane.vertices.size();
        plane.vertices.push_back(origin);
        plane.vertices.push_back(u);
        plane.vertices.push_back(v);
        plane.polygons.push_back({ind+0, ind+1, ind+2});
        visual = combine(visual, plane);
        //break;
    }

    // writeMesh("C:\\Users\\c2zi6\\Desktop\\object2.obj", visual);
    // return 0;
    writeMesh("C:\\Users\\c2zi6\\Desktop\\object2.obj", combine(visual, b));
    return 0;
}



/*

Օօօ, զարմանալի է, ինչպես է այդպիսի բան հնարավոր

ՎՕօ՜օ, ինչ զարմանալի դեպք է տեղի ունեցել հենց նոր, ապշում եմ, թե ինչպես է արդյոք այդ տեսակ բանը տեղ գտել
ժամանակակից հասարակության բարքերի ներքո
Ես վերջում տավոտվելու եմ պատվելով

*/










