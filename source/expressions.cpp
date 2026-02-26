#include "expressions.h"
#include <vector>
#include <string>
#include <cmath>
#include <graphics.h>

std::vector<Node> generateNodes(double h, int n, double L)
{
    std::vector<Node> nodes;

    for(int col = 0; col < 2; ++col)
    {
        double xCoord = (col == 0) ? 0.0 : L;

        for(int i = 0; i <= n; ++i)
        {
            Node node;
            node.id = col*(n+1) + i + 1;
            node.x  = xCoord;
            node.y  = i * h;

            nodes.push_back(node);
        }
    }

    return nodes;
}


std::vector<Element> generateElements(int n, double E, double Aver, double Iver,
                                      double Ahor, double Ihor, double Adiag, double Idiag, double q)
{
    std::vector<Element> elements;
    int id = 1;

    for(int i = 1; i <= n; ++i)
    {
        int leftBottom  = i;
        int leftTop     = i + 1;
        int rightBottom = n + 1 + i;
        int rightTop    = n + 1 + i + 1;

        // ---- Left column ----
        elements.push_back({
            id++, leftBottom, leftTop,
            E, Aver, Iver,
            q, "local"
        });

        // ---- Right column ----
        elements.push_back({
            id++, rightBottom, rightTop,
            E, Aver, Iver,
            q, "local"
        });

        // ---- Beam ----
        elements.push_back({
            id++, leftTop, rightTop,
            E, Ahor, Ihor,
            q, "local"
        });

        // ---- Diagonal 1 ----
        elements.push_back({
            id++, leftBottom, rightTop,
            E, Adiag, Idiag,
            q, "local"
        });

        // ---- Diagonal 2 ----
        elements.push_back({
            id++, leftTop, rightBottom,
            E, Adiag, Idiag,
            q, "local"
        });
    }

    return elements;
}

std::vector<DOF> generateDOFs(const std::vector<Node>& nodes)
{
    std::vector<DOF> dofs;
    int id = 1;

    for(const auto& node : nodes)
    {
        bool isGround = (fabs(node.y) < 1e-9);

        // UX
        dofs.push_back({
            id++,
            node.id,
            "ux",
            isGround   // constrained if ground
        });

        // UY
        dofs.push_back({
            id++,
            node.id,
            "uy",
            isGround   // constrained if ground
        });

        // RZ
        dofs.push_back({
            id++,
            node.id,
            "rotz",
            false      // rotation always free
        });
    }

    return dofs;
}

void plotStructure(const std::vector<Node>& nodes,
                   const std::vector<Element>& elements,
                   int windowWidth,
                   int windowHeight)
{
    initwindow(windowWidth, windowHeight, "2D Frame");

    setbkcolor(BLACK);
    cleardevice();
    setcolor(WHITE);

    int scale  = 60;
    int margin = 100;

    // -------------------------
    // DRAW ELEMENTS
    // -------------------------
    for(const auto& elem : elements)
    {
        const Node& no1 = nodes[elem.n1 - 1];
        const Node& no2 = nodes[elem.n2 - 1];

        int x1 = margin + no1.x * scale;
        int y1 = windowHeight - (margin + no1.y * scale);

        int x2 = margin + no2.x * scale;
        int y2 = windowHeight - (margin + no2.y * scale);

        line(x1, y1, x2, y2);

        int midX = (x1 + x2) / 2;
        int midY = (y1 + y2) / 2;

        std::string label = std::to_string(elem.id);
        outtextxy(midX, midY, const_cast<char*>(label.c_str()));
    }

    // -------------------------
    // DRAW NODES
    // -------------------------
    for(const auto& node : nodes)
    {
        int x = margin + node.x * scale;
        int y = windowHeight - (margin + node.y * scale);

        circle(x, y, 5);

        std::string label = (node.y == 0.0 ? "p" : "n")
                            + std::to_string(node.id);

        outtextxy(x + 8, y - 8, const_cast<char*>(label.c_str()));
    }

    // ================================
    // 🔥 EXPORT IMAGE HERE
    // ================================
    writeimagefile("roof_bracing_truss.bmp", 0, 0, getmaxx(), getmaxy());
    // If PNG supported:
    // writeimagefile("frame.png", 0, 0, getmaxx(), getmaxy());

    getch();
    closegraph();
}


std::vector<int> extractBC(const std::vector<DOF>& dofs)
{
    std::vector<int> bc;

    for(const auto& d : dofs)
    {
        if(d.constrained)
            bc.push_back(d.id);   // already 1-based
    }

    return bc;
}

std::vector<Node> generateroofBracedFrameNodes(double h, int n, double L)
{
    std::vector<Node> nodes;

    int id = 1;   // assuming 1-based node numbering

    // Bottom chord nodes (0 → n)
    for(int i = 0; i <= n; i++)
    {
        Node node;
        node.id = id++;
        node.x  = i * L;
        node.y  = 0.0;

        nodes.push_back(node);
    }

    // Top chord nodes (n+1 → 2n+1)
    for(int i = 0; i <= n; i++)
    {
        Node node;
        node.id = id++;
        node.x  = i * L;
        node.y  = h;

        nodes.push_back(node);
    }

    return nodes;
}
//std::vector<Element> generateElements(int n, double E, double Aver, double Iver,
//                                      double Ahor, double Ihor, double Adiag, double Idiag, double q)

std::vector<Element> generateroofBracedFrameElements(int n,double E,double Aver, double Iver,
double Ahor, double Ihor, double Adiag, double Idiag, double q)
{
    std::vector<Element> elements;

    int id = 1;
    int bottomCount = n + 1;

    for(int i = 0; i < n; i++)
    {
        int b1 = i + 1;
        int b2 = i + 2;

        int t1 = bottomCount + b1;
        int t2 = bottomCount + b2;

        // Bottom chord
        elements.push_back({id++, b1, b2, E, Ahor, Ihor, q, "local","horizontal"});

        // Top chord
        elements.push_back({id++, t1, t2, E, Ahor, Ihor, q, "local","horizontal"});

        // Vertical
        elements.push_back({id++, b1, t1, E, Aver, Iver, q, "local","vertical"});

        // Diagonal 1
        elements.push_back({id++, b1, t2, E, Adiag, Idiag, q, "local","diagonal"});

        // Diagonal 2
        elements.push_back({id++, t1, b2, E, Adiag, Idiag, q, "local","diagonal"});
    }
  // final right vertical
  int b_last = n+1;
  int t_last = bottomCount + b_last;
        elements.push_back({id++, b_last, t_last, E, Aver, Iver, q, "local","vertical"});

    return elements;
}

std::vector<DOF> generateBracedFrameDOFs(int n)
{
    std::vector<DOF> dofs;

    int totalNodes = 2 * (n + 1);

    int leftSupportNode  = 1;
    int rightSupportNode = n + 1;
    int id = 1;

    for(int node = 1; node <= totalNodes; node++)
    {
        bool isSupport = (node == leftSupportNode ||
                          node == rightSupportNode);

        // ux // constrained if support
        dofs.push_back({id++,node,"ux",isSupport});

        // uy // constrained if support
        dofs.push_back({id++,node,"uy",isSupport});

        // rz (always free for pinned)
        dofs.push_back({id++,node,"rz",false});
    }

    return dofs;
}

void plotStructure2(const std::vector<Node>& nodes,
                   const std::vector<Element>& elements,
                   int windowWidth,
                   int windowHeight)
{
    initwindow(windowWidth, windowHeight, "2D Frame");

    setbkcolor(BLACK);
    cleardevice();
    setcolor(WHITE);

    // -----------------------------------
    // 1️⃣ Compute bounding box
    // -----------------------------------
    double minX = nodes[0].x;
    double maxX = nodes[0].x;
    double minY = nodes[0].y;
    double maxY = nodes[0].y;

    for(const auto& node : nodes)
    {
        if(node.x < minX) minX = node.x;
        if(node.x > maxX) maxX = node.x;
        if(node.y < minY) minY = node.y;
        if(node.y > maxY) maxY = node.y;
    }

    double modelWidth  = maxX - minX;
    double modelHeight = maxY - minY;

    int margin = 60;

    // -----------------------------------
    // 2️⃣ Compute automatic scale
    // -----------------------------------
    double scaleX = (windowWidth  - 2.0 * margin) / modelWidth;
    double scaleY = (windowHeight - 2.0 * margin) / modelHeight;

    double scale = std::min(scaleX, scaleY);

    // -----------------------------------
    // 3️⃣ Coordinate transform lambda
    // -----------------------------------
    auto transform = [&](double x, double y)
    {
        int px = margin + (x - minX) * scale;
        int py = windowHeight - (margin + (y - minY) * scale);
        return std::pair<int,int>(px, py);
    };

    // -----------------------------------
    // 4️⃣ Draw elements
    // -----------------------------------
    for(const auto& elem : elements)
    {
        const Node& n1 = nodes[elem.n1 - 1];
        const Node& n2 = nodes[elem.n2 - 1];

        auto p1 = transform(n1.x, n1.y);
        auto p2 = transform(n2.x, n2.y);

        line(p1.first, p1.second, p2.first, p2.second);

        int midX = (p1.first + p2.first) / 2;
        int midY = (p1.second + p2.second) / 2;

        std::string label = std::to_string(elem.id);
        outtextxy(midX, midY, const_cast<char*>(label.c_str()));
    }

    // -----------------------------------
    // 5️⃣ Draw nodes
    // -----------------------------------
    for(const auto& node : nodes)
    {
        auto p = transform(node.x, node.y);

        circle(p.first, p.second, 4);

        std::string label = std::to_string(node.id);
        outtextxy(p.first + 6, p.second - 6,
                  const_cast<char*>(label.c_str()));
    }

    writeimagefile("roof_bracing_truss.bmp",
                   0, 0, getmaxx(), getmaxy());

    getch();
    closegraph();
}

std::vector<Node> generateBuildingNodes(
    int nspan,
    double L,
    int nstories,
    double h)
{
    std::vector<Node> nodes;

    int id = 1;

    for(int j = 0; j <= nstories; j++)        // stories (vertical)
    {
        for(int i = 0; i <= nspan; i++)      // spans (horizontal)
        {
            double x = i * L;
            double y = j * h;

            nodes.push_back({id++, x, y});
        }
    }

    return nodes;
}

std::vector<Element> generateBuildingFrameElements(
    int nspan,
    int nstories,
    double E,
    double Acol, double Icol,
    double Abeam, double Ibeam,
    double q)
{
    std::vector<Element> elements;

    int id = 1;
    int nx = nspan + 1;

    // -----------------------------------
    // HORIZONTAL BEAMS (NO GROUND LEVEL)
    // -----------------------------------
    for(int j = 1; j <= nstories; j++)   // start from 1
    {
        for(int i = 0; i < nspan; i++)
        {
            int n1 = j*nx + i + 1;
            int n2 = n1 + 1;

            elements.push_back({
                id++, n1, n2,
                E, Abeam, Ibeam,
                q,
                "local",
                "beam"
            });
        }
    }

    // -----------------------------------
    // VERTICAL COLUMNS
    // -----------------------------------
    for(int i = 0; i <= nspan; i++)
    {
        for(int j = 0; j < nstories; j++)
        {
            int n1 = j*nx + i + 1;
            int n2 = (j+1)*nx + i + 1;

            elements.push_back({
                id++, n1, n2,
                E, Acol, Icol,
                0,
                "local",
                "column"
            });
        }
    }

    return elements;
}

std::vector<DOF> generatebuildingDOFs(const std::vector<Node>& nodes)
{
    std::vector<DOF> dofs;
    int id = 1;

    for(const auto& node : nodes)
    {
        bool isGround = (fabs(node.y) < 1e-9);

        // UX
        dofs.push_back({
            id++,
            node.id,
            "ux",
            isGround   // constrained if ground
        });

        // UY
        dofs.push_back({
            id++,
            node.id,
            "uy",
            isGround   // constrained if ground
        });

        // RZ
        dofs.push_back({
            id++,
            node.id,
            "rotz",
            isGround      // rotation always free
        });
    }

    return dofs;
}
