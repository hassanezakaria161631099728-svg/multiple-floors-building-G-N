#include "functions/frame_generation.h"
#include "functions/fem.h"
#include "io/io_frame_generation.h"
#include <vector>
#include <iostream>
int main()
{
// 1. Read TXT
frame_generation_input d = readInput("txt/frame_generation_input.txt");

// 1. generate model
//std::vector<Node> nodes = wall_bracing_Nodes(h, n, L);
//std::vector<Node> nodes = roof_bracing_Nodes(h, n, L);

//std::vector<Element> elements = roof_bracing_Elements(n, E, Aver, Iver,Ahor, Ihor, Adiag, Idiag, q);
//std::vector<DOF> dofs = wall_bracing_DOFs(nodes);
//std::vector<DOF> dofs = roof_bracing_DOFs(n);
std::vector<Node> nodes = tall_building_Nodes(d,d.nspan,d.Lspan,d.nstories,d.h);
std::vector<Element> elements = tall_building_Elements(d,d.nspan,d.nstories,d.E,d.Acol,d.Icol,d.Abeam,d.Ibeam,d.q);
std::vector<DOF> dofs = tall_building_DOFs(nodes);

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
//plotStructure2(nodes, elements,800,600);
    std::string filename = "txt/elevation_YZ.txt";
writeresults(filename, nodes, elements, dofs, U, R, fixedDOF);

    return 0;
}
