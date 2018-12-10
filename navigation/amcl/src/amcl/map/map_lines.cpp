#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "amcl/map/map.h"
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace cv;
using namespace std;

void help()
{
 cout << "\nThis program demonstrates line finding with the Hough transform.\n"
         "Usage:\n"
         "./houghlines <image_name>, Default is pic1.jpg\n" << endl;
}

// Safely realloc memory 
void* realloc_s(void **ptr, size_t taille)
{
 void *ptr_realloc = realloc(*ptr, taille);

 if (ptr_realloc != NULL)
     *ptr = ptr_realloc;
 
 return ptr_realloc;
}

void map_hough_lines(map_t* map)
{
 Mat src(4000,4000,CV_8UC1,map->gridData);

 // Here get a binary image by thresholding
 uchar intensityThresh = 200;
 Mat srcThresh;
 threshold(src, srcThresh, intensityThresh, 255, THRESH_BINARY_INV);

 // Lines will be plot on cdst
 Mat cdst;
 cvtColor(srcThresh, cdst, CV_GRAY2BGR);
 
 #if 1
  vector<Vec2f> lines;
  HoughLines(srcThresh, lines, 1, CV_PI/180, 40, 0, 0 );
  
  vector<Vec2f> groups; // <rho, theta>
  for( size_t i = 0; i < lines.size(); i++ )
  {
     bool new_group = true;

     float rho = lines[i][0], theta = lines[i][1];
     if(groups.size() != 0)
     {
        for(int j = 0; j < groups.size(); j++)
        {
            double rho_diff = abs(rho-groups[j][0]);
            double theta_diff = abs(theta-groups[j][1]);
            // Here adjust parameters to group lines
            if(rho_diff < 80 && theta_diff < 3*CV_PI/180)
            {
                new_group = false;
            }

        }
     }
     if(new_group == true)
     {
        Vec2f group;
        group[0] = rho;
        group[1] = theta;
        groups.push_back(group);

        map->nb_lines += 1;
        realloc_s((void **) &(map->lines), sizeof(map_line_t)*map->nb_lines);
        map_line_t line;
        line.rho = rho*map->scale;
        line.theta = theta;
        map->lines[map->nb_lines-1] = line;
        // cout << line.rho << ", " << line.theta << "\n" << endl;
        cout << map->nb_lines;
     }
  }
 #else
  vector<Vec4i> lines;
  HoughLinesP(srcThresh, lines, 1, CV_PI/180, 20, 20, 12);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
  }
 #endif

//  waitKey();
}

double compute_incindent_angle(map_t* map, double oa, int ci, int cj)
{
    // Find line on which is lying the cell
    double xCell = MAP_WXGX(map, ci);
    double yCell = MAP_WXGX(map, cj);

    double min_err = 0.3;
    int line_index = -1;

    for(int i=0; i<1; i++)
    {
        // double theta = map->lines[i].theta;
        // double rho = map->lines[i].rho;
        // cout << "HERE 2: " << theta << "\n" << endl;
        // // cout << "HERE 2\n" << endl;

        // double err = min_err;

        // if(abs(theta)>0.3 && abs(theta-M_PI)>0.3 && abs(theta-2*M_PI)>0.3) 
        // {
        //     double err = yCell - (-xCell*cos(theta)+rho)/sin(theta);
        //     cout << "HERE 3\n" << endl;
        // }
        // cout << "HERE 4\n" << endl;
        // if(err < min_err)
        // {
        //     cout << "HERE 4bis\n" << endl;
        //     min_err = err;
        //     line_index = i;
        // }
        // cout << "HERE 4bisbis\n" << endl;
    }

    // If no close line return -1 : failure
    if(line_index == -1)
        return -1;

    double incindent_angle = abs(map->lines[line_index].theta - oa);
    if(oa < M_PI/2.0 && map->lines[line_index].theta > M_PI*1.5)
        incindent_angle = 2*M_PI - incindent_angle;

    return incindent_angle;
}

/*
int main()
{
 const char* filename = "../../../../../tests/maps/Map0.pgm";
//  const char* filename = "../../../../../tests/maps/ex1.pgm";

 Mat src = imread(filename, 0); 
 if(src.empty())
 {
     help();
     cout << "can not open " << filename << endl;
     return -1;
 }
 
 // Here get a binary image by thresholding
 uchar intensityThresh = 200;
 Mat srcThresh;
 threshold(src, srcThresh, intensityThresh, 255, THRESH_BINARY_INV);

 // Lines will be plot on cdst
 Mat cdst;
 cvtColor(srcThresh, cdst, CV_GRAY2BGR);
 imwrite( "../../../../../tests/maps/src.jpg", srcThresh);
 
 #if 1
  vector<Vec2f> lines;
  HoughLines(srcThresh, lines, 1, CV_PI/180, 40, 0, 0 );
  
  vector<Vec2f> groups; // <rho, theta>
  for( size_t i = 0; i < lines.size(); i++ )
  {
     bool new_group = true;

     float rho = lines[i][0], theta = lines[i][1];
     if(groups.size() != 0)
     {
        for(int j = 0; j < groups.size(); j++)
        {
            double rho_diff = abs(rho-groups[j][0]);
            double theta_diff = abs(theta-groups[j][1]);
            // Here adjust parameters to group lines
            if(rho_diff < 80 && theta_diff < 3*CV_PI/180)
            {
                new_group = false;
            }

        }
     }
     if(new_group == true)
     {
        Vec2f group;
        group[0] = rho;
        group[1] = theta;
        groups.push_back(group);

        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 4000*(-b));
        pt1.y = cvRound(y0 + 4000*(a));
        pt2.x = cvRound(x0 - 4000*(-b));
        pt2.y = cvRound(y0 - 4000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 0.5, CV_AA);
     }
  }
  cout << groups.size() << endl;
//   for( size_t i = 0; i < lines.size(); i++ )
//   {
//      float rho = lines[i][0], theta = lines[i][1];
//      Point pt1, pt2;
//      double a = cos(theta), b = sin(theta);
//      double x0 = a*rho, y0 = b*rho;
//      pt1.x = cvRound(x0 + 4000*(-b));
//      pt1.y = cvRound(y0 + 4000*(a));
//      pt2.x = cvRound(x0 - 4000*(-b));
//      pt2.y = cvRound(y0 - 4000*(a));
//      line( cdst, pt1, pt2, Scalar(0,0,255), 0.5, CV_AA);
//   }
 #else
  vector<Vec4i> lines;
  HoughLinesP(srcThresh, lines, 1, CV_PI/180, 20, 20, 12);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
  }
 #endif
 
 imwrite( "../../../../../tests/maps/lines.jpg", cdst);

 namedWindow("source",WINDOW_NORMAL);
 resizeWindow("source", 1500,1500);
 imshow("source", srcThresh);
 namedWindow("detected lines",WINDOW_NORMAL);
 resizeWindow("detected lines", 1500,1500);
 imshow("detected lines", cdst);

 waitKey();

 return 0;
}*/
