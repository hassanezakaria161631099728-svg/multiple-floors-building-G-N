#pragma once
#include <vector>
#include <string>
#include "../helper_functions/expressions.h"

BuildingData readInput(const char* filename);

void writeresults(const std::string& filename,
                      const std::vector<Node>& nodes,
                      const std::vector<Element>& elements,
                      const std::vector<DOF>& dofs,
                      const std::vector<double>& U,
                      const std::vector<double>& R,
                    const std::vector<int>& fixedDOF
                      );
