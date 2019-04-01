#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "astar.hpp"

Eigen::MatrixXd readFile(std::string filename)
{
  std::ifstream inFile(filename);
  std::string line;
  std::vector<std::string> data;
  if(!inFile.is_open()) {
    printf("Cannot open given file.\n"); 
    Eigen::MatrixXd empty;
    return empty;
  }
  if(inFile.is_open()){
    while(getline(inFile, line)){
      data.push_back(line);
    }
  }
  int height = data.size();
  std::vector<int> num;
  for(int i=0; i<=height; ++i){
    std::string temp = data[i];
    while(!temp.empty()){
      std::size_t pos = temp.find(" ");
      if(pos == std::string::npos){
        num.push_back(atoi(temp.c_str()));
        break;
      }
      std::string sub = temp.substr(0, pos);
      num.push_back(atoi(sub.c_str()));
      temp = temp.substr(pos+1, temp.length()-pos);
    }
  }
  int width = (int)num.size()/height;
  Eigen::MatrixXd map(height, width);
  for(int i=0; i<height; ++i){
    for(int j=0; j<width; ++j){
      map(i, j) = num[i*height +j];
    }
  }
  return map;
}

int main(int argc, char** argv)
{
  if(argc!=2){
    printf("Not enough input, ignoring...\n");
    return 0;
  }
  std::string in(argv[1]);
  Eigen::MatrixXd map = readFile(in);
  if(map.size() == 0) return 0;
  AStar::Node start(0, 0, NULL), end(map.rows()-1, map.cols()-1, NULL);
  AStar::NODE_LIST path;
  AStar::NODE_LIST::iterator nit;
  AStar::AStar astar(start, end, map);
  if(astar.plan(path)){
    printf("Length: %d\n", (int)path.size());
    Eigen::MatrixXd draw(map.rows(), map.cols()); draw = map;
    for(nit=path.begin(); nit!=path.end(); ++nit){
      draw(nit->get_x(), nit->get_y()) = -1;
      //std::cout << nit->get_x() << " " << nit->get_y() << "\n";
    } 
    std::cout << draw << std::endl;
  }
  return 0;
}
