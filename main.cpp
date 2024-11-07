#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "utils.h"

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
        ABb = ABb && (0 < x && x <= 1);
        BCb = intersect(planeA, planeB, planeC, B, C-B, BxC, x);
        BCb = BCb && (0 < x && x <= 1);
        ACb = intersect(planeA, planeB, planeC, A, C-A, AxC, x);
        ACb = ACb && (0 <= x && x <= 1);
        std::vector<int> buffer;
        int ind = -1;
        auto addVertex = [&added, &nexta, &buffer](Point<double> p)->int
        {
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
        auto emptyBuffer = [&added, &buffer]()
        {
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

void isolateByPlane(Mesh& a, Point<double> planeA, Point<double> planeB, Point<double> planeC)
{
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

std::vector<Triangle> triangulize(std::vector<int> polygon, const std::vector<Point<double>>& points, Point<double> planeNormal) {
    Point<double> planeOrigin = points[polygon[0]];
    std::vector<Point2<double>> polygonPoints(polygon.size()); // 2d
    normalize(planeNormal);
    Point<double> base1 = normalized(perp(planeNormal));
    Point<double> base2 = normalized(base1^planeNormal);
    for (int i = 0; i < polygon.size(); i++) {
        Point<double> p = points[polygon[i]] - planeOrigin;
        polygonPoints[i] = {base1 * p, base2 *  p};
    }

    auto isInCircumcircle = [](Point<double> A, Point<double> B, Point<double> C, Point<double> D)
    {
        std::array<std::array<double, 3>, 3> matrix;
        double difx, dify;
        difx = A.x - D.x;
        dify = A.y - D.y;
        matrix[0] = {difx, dify, difx*difx + dify*dify};
        difx = B.x - D.x;
        dify = B.y - D.y;
        matrix[1] = {difx, dify, difx*difx + dify*dify};
        difx = C.x - D.x;
        dify = C.y - D.y;
        matrix[2] = {difx, dify, difx*difx + dify*dify};
        double determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
                             - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
                             + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
        return determinant > 0;
    };

    double max_x=-1e18, min_x=1e18, max_y=-1e18, min_y=1e18;
    for (auto&[x, y] : polygonPoints)
    {
        max_x = std::max(max_x, x);
        min_x = std::min(min_x, x);
        max_y = std::max(max_y, y);
        min_y = std::min(min_y, y);
    }
    max_x += 2;
    min_x -= 2;
    max_y += 2;
    min_y -= 2;

    std::vector<Triangle> ret;
    // OPENCV PART
    {
        cv::Subdiv2D subdiv;
        subdiv.initDelaunay(cv::Rect(min_x, min_y, max_x-min_x, max_y-min_y));
        for (auto &[x, y] : polygonPoints) {
            subdiv.insert(cv::Point2d(x, y));
        }
        std::vector<cv::Vec6f> triangles;
        subdiv.getTriangleList(triangles);
        auto getIndex = [&polygonPoints](Point2<double> p) {
            double mindist = std::numeric_limits<double>::max();
            int mindistind = -1;
            for (int i = 0; i < polygonPoints.size(); i++) {
                double len = length(polygonPoints[i] - p);
                if (len < mindist) {
                    mindist = len;
                    mindistind = i;
                }
            }
            return mindistind;
        };
        for (cv::Vec6f trig : triangles) {
            Triangle trigind = {
                        polygon[getIndex({trig[0], trig[1]})],
                        polygon[getIndex({trig[2], trig[3]})],
                        polygon[getIndex({trig[4], trig[5]})]};
            ret.push_back(trigind);
        }

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
        // std::cout << "ha, karuceg@" << std::endl;
        std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;
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
    int partcnt = 3;
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

        break;
        //if (i == 1) break;
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










