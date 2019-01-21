#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../../../include/amcl/map/map.h"
#include <opencv2/opencv.hpp>

#include <iostream>

// using namespace cv;
using namespace std;
using namespace cv;

// Create a new map
map_t *map_alloc(void)
{
    map_t *map;

    map = (map_t *)malloc(sizeof(map_t));

    // Assume we start at (0, 0)
    map->origin_x = 0;
    map->origin_y = 0;

    // Make the size odd
    map->size_x = 0;
    map->size_y = 0;
    map->scale = 0;

    // Allocate storage for main map
    map->cells = (map_cell_t *)NULL;

    return map;
}

// Find two nearest occupied cells from map.
cells_index_t map_find_cells(map_t *map, double ox, double oy, double oa, double max_range)
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;
  // Number of obstacles that are found
  int ncells_found = 0;
  int same_obstacle = 0;

  cells_index_t nearest_cells;

  x0 = MAP_GXWX(map,ox);
  y0 = MAP_GYWY(map,oy);
  
  x1 = MAP_GXWX(map,ox + max_range * cos(oa));
  y1 = MAP_GYWY(map,oy + max_range * sin(oa));

  // Initialize the cells to the ones corresponding to maximum range
  nearest_cells.i_first = x1;
  nearest_cells.j_first = y1;
  nearest_cells.i_second = x1;
  nearest_cells.j_second = y1;

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > 0)
      {
        nearest_cells.i_first = y;
        nearest_cells.j_first = x;
        ncells_found += 1;
        same_obstacle = 1;
        if(map->cells[MAP_INDEX(map,y,x)].p_glass<0.1)
        {
          nearest_cells.i_second = nearest_cells.i_first;
          nearest_cells.j_second = nearest_cells.j_first;
          ncells_found += 1;
        }
      }
  }
  else
  {
    if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > 0)
      {
        nearest_cells.i_first = x;
        nearest_cells.j_first = y;
        ncells_found += 1;
        same_obstacle = 1;
        if(map->cells[MAP_INDEX(map,x,y)].p_glass<0.1)
        {
          nearest_cells.i_second = nearest_cells.i_first;
          nearest_cells.j_second = nearest_cells.j_first;
          ncells_found += 1;
        }
      }
  }    

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state <= 0)
        same_obstacle = 0;
      if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > 0)
      {
        if(ncells_found == 0)
        {
          nearest_cells.i_first = y;
          nearest_cells.j_first = x;
          ncells_found += 1;
          same_obstacle = 1;
          if(map->cells[MAP_INDEX(map,y,x)].p_glass<0.1)
          {
            nearest_cells.i_second = nearest_cells.i_first;
            nearest_cells.j_second = nearest_cells.j_first;
            ncells_found += 1;
          }
        }
        else 
        {
          if(same_obstacle == 0 || map->cells[MAP_INDEX(map,y,x)].p_glass<0.1)
          {
            nearest_cells.i_second = y;
            nearest_cells.j_second = x;
            ncells_found += 1;
          }
        }
      }
    }
    else
    {
      if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state <= 0)
        same_obstacle = 0;
      if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > 0)
      {
        if(ncells_found == 0)
        {
          nearest_cells.i_first = x;
          nearest_cells.j_first = y;
          ncells_found += 1;
          same_obstacle = 1;
          if(map->cells[MAP_INDEX(map,x,y)].p_glass<0.1)
          {
            nearest_cells.i_second = nearest_cells.i_first;
            nearest_cells.j_second = nearest_cells.j_first;
            ncells_found += 1;
          }
        }
        else 
        {
          if(same_obstacle == 0 || map->cells[MAP_INDEX(map,x,y)].p_glass<0.1)
          {
            nearest_cells.i_second = x;
            nearest_cells.j_second = y;
            ncells_found += 1;
          }
        }
      }
    } 

    if(ncells_found == 2)
      return nearest_cells;
  }

  return nearest_cells;
}

// Compute range between a pose and a cell by giving the cell index
double compute_range(map_t *map, double ox, double oy, int ci, int cj, double max_range)
{
  int x0 = MAP_GXWX(map,ox);
  int y0 = MAP_GYWY(map,oy);
  double range = sqrt((ci-x0)*(ci-x0) + (cj-y0)*(cj-y0)) * map->scale;
  return range > max_range? max_range : range; 
}

int main()
{

    map_t *map = map_alloc();
    map->scale = 1;
    map->size_x = 100;
    map->size_y = map->size_x;
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    map->gridData = (uint8_t *)malloc(sizeof(uint8_t) * 10 * map->size_x * map->size_y);
    cout << sizeof(map->gridData) << endl;
    map->nb_lines = 0;
    map->lines = (map_line_t *)malloc(sizeof(map_line_t) * 200);

    // uint8_t example[25] = {255,255,0,0,1, 0,0,0,1,0, 0,0,1,0,0, 0,1,0,0,0, 1,0,0,0,0};

    for (int i = 0; i < map->size_x; i++)
    {
        for (int j = 0; j < map->size_y; j++)
        {
            if (i == 5 || i == map->size_x - 5 || j == 5 || j == map->size_y - 5 || i == j
                || i == 6 || j == 6)
            {
                map->cells[MAP_INDEX(map, i, j)].occ_state = 1;
                map->cells[MAP_INDEX(map, i, j)].p_glass = 0.5;
                map->gridData[MAP_INDEX(map, i, j)] = 0;
            }
            else
            {
                map->cells[MAP_INDEX(map, i, j)].occ_state = 0;
                map->cells[MAP_INDEX(map, i, j)].p_glass = -1;
                map->gridData[MAP_INDEX(map, i, j)] = 255;
            }
        }
    }

    int robot_i = 2;
    int robot_j = static_cast<int>(map->size_y/2);
    double robot_a = -0.785398;
    cout << "robot pose: (" << robot_i << "," << robot_j << ")" << endl;
    map->gridData[MAP_INDEX(map, robot_i, robot_j)] = 0;

    Mat src(Size(map->size_x,map->size_y),CV_8UC1,map->gridData);

    cells_index_t cells_index;
    cells_index = map_find_cells(map, MAP_WXGX(map, robot_i), MAP_WXGX(map, robot_j), robot_a, 100);

    cout << "first: (" << cells_index.i_first << "," << cells_index.j_first << ")\nsecond: (" << cells_index.i_second << "," << cells_index.j_second << ")" << endl; 
    map->gridData[MAP_INDEX(map, cells_index.i_first, cells_index.j_first)] = 200;
    map->gridData[MAP_INDEX(map, cells_index.i_second, cells_index.j_second)] = 200;

    // cout << src << endl;
    namedWindow("source",WINDOW_NORMAL);
    resizeWindow("source", 1500,1500);
    imshow("source", src);
    waitKey();

    return 1;

}
