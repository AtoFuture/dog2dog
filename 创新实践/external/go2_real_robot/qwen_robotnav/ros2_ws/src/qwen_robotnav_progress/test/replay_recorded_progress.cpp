// Offline counterfactual: check recorded map feedback, without truth or actuation.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "qwen_robotnav_progress/stable_progress.hpp"
int main(int argc, char ** argv) {
  if (argc != 2) {return 2;}
  std::ifstream input(argv[1]);
  if (!input) {return 2;}
  qwen_robotnav_progress::StableProgress progress;
  std::string line;std::getline(input,line);double first_stall = -1.;int samples = 0;
  while (std::getline(input,line)) {
    std::stringstream row(line);std::string token;
    std::getline(row,token,',');const auto time = std::stod(token);
    std::getline(row,token,',');const auto x = std::stod(token);
    std::getline(row,token,',');const auto y = std::stod(token);
    if (!progress.check(time,x,y) && first_stall < 0.) {first_stall = time;}
    ++samples;
  }
  std::cout << "{\"samples\":" << samples << ",\"first_stall_sim_s\":" << first_stall << "}\n";
}
