#include <iostream>
#include <cstdlib>
#include <ctime>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "astar.hpp"

// ./astar_cv height width p
// It will output a video "demo.avi" in your current directory

bool get_in = false;
bool ASTAR_VERBOSE = true;
int height, width; // Image dimension
int start_x, start_y, end_x, end_y;
static int count = 0;
const int FPS = 30;
cv::Mat img;
void mouse_cb(int Event, int x, int y, int flags, void* param)
{
  if(Event == CV_EVENT_LBUTTONUP){
    if(count==0){
      if(img.at<cv::Vec3b>(y, x) == cv::Vec3b(0., 0., 0.)){
        std::cerr << "Target on obstacle, ignore...\n"; return;
      }
      start_x = y; start_y = x; ++count;
      printf("Start point set at (%d, %d)\n", y, x);
    }
    else{
      if(img.at<cv::Vec3b>(y, x) == cv::Vec3b(0., 0., 0.)){
        std::cerr << "Target on obstacle, ignore...\n"; return;
      }
      end_x = y; end_y = x;
      printf("End point set at (%d, %d)\n", y, x);
      get_in = true;
    }
  }
}

int main(int argc, char** argv)
{
  if(argc!=4) {printf("Not enough inputs.\n"); return 0;}
  height = atoi(argv[1]);
  width = atoi(argv[2]);
  double p = atof(argv[3]);
  
  srand(time(NULL));
  img.create(height, width, CV_8UC3);
  cv::VideoWriter video("demo.avi", CV_FOURCC('M', 'J', 'P', 'G'), FPS, cv::Size(height, width));
  for(int i=0; i<img.rows; ++i){
    for(int j=0; j<img.cols; ++j){
      double x = (double) rand() / (RAND_MAX +1.0);
      if(x>=p){
        img.at<cv::Vec3b>(i, j)[0] = 255.;
        img.at<cv::Vec3b>(i, j)[1] = 255.;
        img.at<cv::Vec3b>(i, j)[2] = 255.;
      }
      else{
        img.at<cv::Vec3b>(i, j)[0] = 0.;
        img.at<cv::Vec3b>(i, j)[1] = 0.;
        img.at<cv::Vec3b>(i, j)[2] = 0.;
      }
    }
  }
  // Convert img to Eigen::MatrixXd
  Eigen::MatrixXd map(height, width);
  for(int i=0; i<img.rows; ++i){
    for(int j=0; j<img.cols; ++j){
      if(img.at<cv::Vec3b>(i, j)[0] == 255.) map(i, j)=0.;
      else map(i, j)=100.;
    }
  }
  cv::namedWindow("Test", CV_WINDOW_NORMAL);
  cv::resizeWindow("Test", 2*height, 2*width);
  cv::setMouseCallback("Test", mouse_cb, NULL);
  cv::imshow("Test", img);
  cv::waitKey(0);
  while(!get_in) {} // Wait to get information
  img.at<cv::Vec3b>(start_x, start_y) = cv::Vec3b(0., 255., 0.); // G
  img.at<cv::Vec3b>(end_x, end_y) = cv::Vec3b(0., 255., 0.); // G
  cv::circle(img, cv::Point(start_y, start_x), 8, cv::Vec3b(0., 255., 0.), 1);
  cv::circle(img, cv::Point(end_y, end_x), 8, cv::Vec3b(0., 255., 0.), 1);
  for(int i=0; i<30; ++i) video << img;
  AStar::Node start(start_x, start_y, NULL), end(end_x, end_y, NULL);
  AStar::AStar planner(start, end, map);
  AStar::NODE_LIST res; // Planned path placeholder
  if(!planner.plan(res)) {printf("Cannot find path.\n"); return 0;}
  printf("%d\n", (int)res.size());
  for(AStar::NODE_LIST::iterator it=res.begin(); it!=res.end(); ++it){
    img.at<cv::Vec3b>(it->get_x(), it->get_y())[0] = 0.; // B
    img.at<cv::Vec3b>(it->get_x(), it->get_y())[1] = 0.; // G
    img.at<cv::Vec3b>(it->get_x(), it->get_y())[2] = 255.; // R
    video << img;
    //cv::destroyWindow("Test");cv::namedWindow("Test", CV_WINDOW_NORMAL); cv::resizeWindow("Test", 2*height, 2*width);
    //cv::imshow("Test", img);
    //if(it!=std::prev(res.end())) cv::waitKey(1000/FPS); else cv::waitKey(0);
  }
  
  return 0;
}
