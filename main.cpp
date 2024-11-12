#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>

#include "utils.h"

typedef std::tuple<Point3d, Point3d, Point3d> Plane;

std::vector<int> polygonByEdges(std::vector<std::pair<int, int>>& a)
{
    int n = (int)a.size();
    int maxind = 0;
    for (auto[u, v] : a) {
        maxind = std::max(maxind, u);
        maxind = std::max(maxind, v);
    }
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

void intersectByPlane(Mesh& a, Point3d planeA, Point3d planeB, Point3d planeC,
                                std::vector<std::pair<int, int>>& intersectionEdges = NULLREFERENCE<std::vector<std::pair<int, int>>>)
{
    double mnsegpart = 1;
    double mxsegpart = 0;
    std::map<std::pair<int, int>, int> intersection;
    for (Triangle trig : a.polygons) {
        std::vector<std::pair<int, int>> edges = {{trig[0], trig[1]}, {trig[1], trig[2]}, {trig[2], trig[0]}};
        for (auto&[u, v] : edges) {
            if (intersection.count({u, v})) continue;
            Point3d A = a.vertices[u];
            Point3d B = a.vertices[v];
            Point3d ret;
            double x;
            if (intersect(planeA, planeB, planeC, A, B-A, ret, x)) {
                bool exist = false;
                if (0 < x && x < 1) exist = true;
                if (x == 1 || x == 0) {
                    std::cout << "@H@@@@@@@ SIK HENC EN CAYRAYIN DEPQN E OR KMTACEIR LAV DE SPES BAN INCHX KRNA EXNI" << std::endl;
                    //exit(0);
                }
                if (x == 1) {
                    if (sideRelativePlane(planeA, planeB, planeC, A) < 0) exist = true;
                }
                if (x == 0) {
                    if (sideRelativePlane(planeA, planeB, planeC, B) < 0) exist = true;
                }
                if (exist) {
                    mnsegpart = std::min(mnsegpart, x);
                    mxsegpart = std::max(mxsegpart, x);
                    intersection[{u, v}] = intersection[{v, u}] = a.vertices.size();
                    a.vertices.push_back(ret);
                }
            }
        }
    }

    //std::cout << "MIN SEGMENT PART: " << mnsegpart << std::endl;
    //std::cout << "MAX SEGMENT PART: " << mxsegpart << std::endl;
    std::vector<Triangle> newpolygons;
    for (Triangle trig : a.polygons) {
        int intersectionCount = 0;
        intersectionCount += intersection.count({trig[0], trig[1]});
        intersectionCount += intersection.count({trig[1], trig[2]});
        intersectionCount += intersection.count({trig[2], trig[0]});
        if (intersectionCount == 2) {
            for (int i = 0; i < 3; i++) {
                int a = trig[i];
                int b = trig[(i+1)%3];
                int c = trig[(i+2)%3];
                if (intersection.count({a, b}) && intersection.count({b, c})) {
                    int d1 = intersection[{a, b}];
                    int d2 = intersection[{b, c}];
                    newpolygons.push_back({d1, b, d2});
                    newpolygons.push_back({a, d1, c});
                    newpolygons.push_back({c, d1, d2});
                    intersectionEdges.push_back({d1, d2});
                    break;
                }
            }
        } else if (intersectionCount == 0) {
            newpolygons.push_back(trig);
        } else if (intersectionCount == 3) {
            std::cout << "CAYRAYIN DEPQ 3" << std::endl;
            //exit(0);
        } else if (intersectionCount == 1) {
            std::cout << "CAYRAYIN DEPQ 1" << std::endl;
        } else {
            std::cout << "AYL CAYRAYIN DEPQ" << std::endl;
            //exit(0);
        }
    }

    a.polygons = newpolygons;
}

void isolateByPlane(Mesh& a, Point3d planeA, Point3d planeB, Point3d planeC)
{
    Mesh nexta;
    std::map<Point3d, int> pointind;
    for (Triangle p : a.polygons) {
        Point3d A = a.vertices[p[0]];
        Point3d B = a.vertices[p[1]];
        Point3d C = a.vertices[p[2]];
        bool keep = true;
        keep &= sideRelativePlane(planeA, planeB, planeC, A) >= 0;
        keep &= sideRelativePlane(planeA, planeB, planeC, B) >= 0;
        keep &= sideRelativePlane(planeA, planeB, planeC, C) >= 0;
        if (keep) {
            if (!pointind.count(A)) {
                pointind[A] = nexta.vertices.size();
                nexta.vertices.push_back(A);
            }
            if (!pointind.count(B)) {
                pointind[B] = nexta.vertices.size();
                nexta.vertices.push_back(B);
            }
            if (!pointind.count(C)) {
                pointind[C] = nexta.vertices.size();
                nexta.vertices.push_back(C);
            }
            nexta.polygons.push_back({pointind[A], pointind[B], pointind[C]});
        }
    }
    a = nexta;
}

struct NODE {
    int val;
    NODE* next;
    NODE* prev;
};

std::vector<Triangle> triangulize(std::vector<int> polygon, const std::vector<Point3<double>>& points, Point3<double> planeNormal) {
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
        Point3<double> A = points[cur->prev->val];
        Point3<double> B = points[cur->val];
        Point3<double> C = points[cur->next->val];
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

Mesh visual;

int main() {
    Mesh b;
    readMesh("C:\\Users\\c2zi6\\Desktop\\object.obj", b);

    Point3d origin = {-15, -10, -110};
    // drawPoint(visual, origin);
    Point3d direction = {30, 25, 70};
    int partcnt = 60;
    double angle = PI/6;

    normalize(direction);
    Point3d basis1 = perp(direction);
    Point3d basis2 = basis1^direction;
    normalize(basis1);
    normalize(basis2);
    double scale = tan(angle);
    Point3d center = origin + direction;
    std::vector<Point3d> surface;
    for (int part = 0; part < partcnt; part++) {
        double alpha = 2*PI/partcnt * part;
        surface.push_back(center + scale*(sin(alpha)*basis1 + cos(alpha)*basis2));
    }

    for (int i = 0; i < partcnt; i++)
    {
        std::cout << "cutting part " << i << std::endl;
        Point3d u = surface[i];
        Point3d v = surface[(i+1)%partcnt];
        intersectByPlane(b, origin, u, v);
        isolateByPlane(b, origin, u, v);
    }

    double minDistance = std::numeric_limits<double>::max();
    for (auto trig : b.polygons) {
        for (int ind : {trig[0], trig[1], trig[2]}) {
            Point3d P = b.vertices[ind];
            double cur = direction * (P - origin);
            minDistance = std::min(minDistance, cur);
        }
    }
    minDistance *= 0.9;

    std::cout << minDistance << std::endl;

    int startind = b.vertices.size();
    for (int i = 0; i < partcnt; i++) {
        b.vertices.push_back(origin + minDistance * (surface[i] - origin));
    }
    if ("WE MUST SECURE THE EXISTENCE OF OUR PEOPLE AND A FUTURE FOR WHITE CHILDREN"[14/88]) {
        std::set<std::pair<int, int>> edges;
        for (auto trig : b.polygons) {
            std::vector<std::pair<int, int>> ITERATION_edges = {{trig[0], trig[1]}, {trig[1], trig[2]}, {trig[2], trig[0]}};
            for (auto&[u, v] : ITERATION_edges) {
                edges.insert({u, v});
            }
        }
        for (auto&[u, v] : edges) {
            if (edges.count({v, u})) continue;
            int plane1 = -1;
            int plane2 = -1;
            for (int i = 0; i < partcnt; i++) {
                Point3d planeA = origin;
                Point3d planeB = surface[i];
                Point3d planeC = surface[(i+1)%partcnt];
                if (sideRelativePlane(planeA, planeB, planeC, b.vertices[u]) == 0) {
                    plane1 = i;
                }
                if (sideRelativePlane(planeA, planeB, planeC, b.vertices[v]) == 0) {
                    plane2 = i;
                }
            }
            b.polygons.push_back({u, startind + plane1, v});
            if (plane1 != plane2) {
                b.polygons.push_back({startind + plane2, v, startind + plane1});
            }
        }
    }
    writeMesh("C:\\Users\\c2zi6\\Desktop\\object2.obj", combine(visual, b));
    return 0;
}



/*

Օօօ, զարմանալի է, ինչպես է այդպիսի բան հնարավոր

ՎՕօ՜օ, ինչ զարմանալի դեպք է տեղի ունեցել հենց նոր, ապշում եմ, թե ինչպես է արդյոք այդ տեսակ բանը տեղ գտել
ժամանակակից հասարակության բարքերի ներքո
Ես վերջում տավոտվելու եմ պատվելով

*/










