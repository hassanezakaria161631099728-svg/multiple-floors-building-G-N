#include "expressions.h"
#include "fem.h"
#include "io.h"
#include <vector>
#include <iostream>
int main()
{
//double h = 4;
//int n = 4;
//double L = 4.5;
double E = 2.1e11;
//double Aver = 10.323e-4;
//double Iver = 15.919e-8;
//double Ahor = 84.5e-4;
//double Ihor = 1317.8e-8;
//double Adiag = 12.3e-4;
//double Idiag = 114.6e-8;
double q = -200;
int nspan = 3;
int L = 5;
int nstories = 3;
int h = 3;
double Acol = 131.4e-4; double Icol = 19270e-8; double Abeam = 84.5e-4; double Ibeam = 23130e-8;
// 1. generate model
//std::vector<Node> nodes = generateNodes(h, n, L);
//std::vector<Node> nodes = generateroofBracedFrameNodes(h, n, L);

//std::vector<Element> elements = generateroofBracedFrameElements(n, E, Aver, Iver,Ahor, Ihor, Adiag, Idiag, q);
//std::vector<DOF> dofs = generateDOFs(nodes);
//std::vector<DOF> dofs = generateBracedFrameDOFs(n);
std::vector<Node> nodes = generateBuildingNodes(nspan,L,nstories,h);
std::vector<Element> elements = generateBuildingFrameElements(nspan,nstories,E,Acol,Icol,Abeam,Ibeam,q);
std::vector<DOF> dofs = generatebuildingDOFs(nodes);

std::vector<int> bc = extractBC(dofs);
int totalDOF = dofs.size();
//2. compute geometry
for (auto& e: elements)
    computeGeometry(e, nodes);
//3. initialize global system
std::vector<std::vector<double>>K(totalDOF,std::vector<double>(totalDOF,0.0));
std::vector<double>F(totalDOF,0.0);

//4.assemble global matrices
for (const auto& e: elements)
{
    double kg[6][6];
    double fe[6];
    globalStiffness(e, kg);
    equivalentLoad(e, fe);
    assembleGlobal(K, kg, e.n1, e.n2);
    assembleLoad(F, fe, e.n1, e.n2);

}
//  nodal loads
//F[1]+= 1281;
//F[4]+= 2812.3;
//F[7]+= 3062.6;
//F[10]+= 2812.3;
//F[13]+= 1281;

//5. partition DOFs
std::vector<int> freeDOF,fixedDOF;
     partitionDOF(totalDOF,bc,freeDOF,fixedDOF);
//6. build reduced system
    std::vector<std::vector<double>> Kff;
    std::vector<double> Ff;

    buildReducedSystem(K,F,freeDOF,Kff,Ff);
//7. solve
    std::vector<double> Uf(freeDOF.size());
    solveSystem(Kff,Ff,Uf);
//8. expand full displacement vector
    std::vector<double> U;
    expandDisplacements(totalDOF,freeDOF,Uf,U);
//9. compute reactions
    std::vector<double> R;
    computeReactions(K,F,U,fixedDOF,R);
//10. outputs
//plotStructure(nodes, elements, 800, 600);
plotStructure2(nodes, elements,800,600);
//    std::string filename = "results3.txt";
//writeresults(filename, nodes, elements, dofs, U, R, fixedDOF);

    return 0;
}
