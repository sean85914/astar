#include <iostream>
#include <ctime>
#include "astar.hpp"

int main(int argc, char** argv)
{
  if(argc!=2){
    printf("Not enough input, ignoring...\n");
    return 0;
  }
  int size = atoi(argv[1]);
  Eigen::MatrixXd map(size, size);
  AStar::Node start(0, 0, NULL), end(map.rows()-1, map.cols()-1, NULL);
  AStar::NODE_LIST path;
  AStar::NODE_LIST::iterator nit;
  AStar::AStar astar(start, end, map);
  clock_t start_t = clock();
  astar.plan(path);
  printf("Cost %f second.\n", (double)((clock()-start_t))/CLOCKS_PER_SEC);
  return 0;
}
