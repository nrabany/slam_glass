#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../../../include/amcl/map/map.h"
#include <opencv2/opencv.hpp>

#include <iostream>

// using namespace cv;
using namespace std;

// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}

int main()
{
    /*
    map_t *map = map_alloc();
    map->scale = 1;
    map->size_x = 5;
    map->size_y = 5;
    map->gridData = (int*)malloc(sizeof(int)*10*map->size_x*map->size_y);
    cout  << sizeof(map->gridData) << endl;
    map->nb_lines = 0;
    map->lines = (map_line_t*)malloc(sizeof(map_line_t)*20);

    // int example[] = {1,1,1,1,1, 1,-1,-1,-1,1, 1,-1,-1,-1,1, 1,-1,-1,-1,1, 1,1,1,1,1};
    int example[5][5] = {{255,255,0,0,0}, {0,0,0,0,0},{ 0,0,0,0,0}, {0,0,0,0,0}, {0,0,0,0,0}};

    // cout  << sizeof(example) << endl;

    // for(int i = 0; i < map->size_x; i++)
    // {
    //     for(int j = 0; j < map->size_y; j++)
    //     {
    //         cout << example[i][j] << endl;
    //         if(i==0)// || i%5==0 || (i-4)%5==0 // i<5 || i>=2
    //         {
    //             map->gridData[MAP_INDEX(map, i, j)] = 255;
    //             // example[MAP_INDEX(map, i, j)] = 255;
    //         }
    //         else
    //         {
    //             map->gridData[MAP_INDEX(map, i, j)] = 0;
    //             // example[MAP_INDEX(map, i, j)] = 0;
    //         }
    //     }
    // }

    // for ( int k = 0; k < map->size_x*map->size_y; k++)
    //     cout << map->gridData[k] << endl;

    // cout  << sizeof(map->gridData) << " / " << sizeof(map->gridData[0]) << ". allocated size is " << sizeof(int)*map->size_x*map->size_y << endl;
    // Mat src(Size(map->size_x,map->size_y),CV_8UC1,map->gridData);
    Mat A = Mat(5,5,CV_8UC1,&example);

    cout << A << endl;
    namedWindow("source",WINDOW_NORMAL);
    resizeWindow("source", 1500,1500);
    imshow("source", A);  
    waitKey();

    return 1;*/
    uint8_t uarr[] = {1,2,3,4,5,6,7,8,9,10,11,12};
    int rows = 3;
    int cols = 4;
    cv::Size sz(cols,rows);

    cv::Mat mat1(sz,CV_8UC1, uarr);
    cv::Mat mat2(rows, cols, CV_8UC1, uarr);

    std::cout<< "mat1: \n"<<mat1 << "\n\nmat2:\n" << mat2 << std::endl;
    return 0;
}