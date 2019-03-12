/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Range routines
 * Author: Andrew Howard
 * Date: 18 Jan 2003
 * CVS: $Id: map_range.c 1347 2003-05-05 06:24:33Z inspectorg $
 * Modified by Nicolas Rabany 2018.12.05
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "amcl/map/map.h"

// Extract a single range reading from the map.  Unknown cells and/or
// out-of-bound cells are treated as occupied, which makes it easy to
// use Stage bitmap files.
double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range)
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = MAP_GXWX(map,ox);
  y0 = MAP_GYWY(map,oy);
  
  x1 = MAP_GXWX(map,ox + max_range * cos(oa));
  y1 = MAP_GYWY(map,oy + max_range * sin(oa));

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
    if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
  }
  else
  {
    if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
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
      if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
    }
    else
    {
      if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
    }
  }
  return max_range;
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

  // Initialize the cells to the ones corresponding to maximum range
  nearest_cells.i_first = x1;
  nearest_cells.j_first = y1;
  nearest_cells.i_second = x1;
  nearest_cells.j_second = y1;

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
        if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,y,x)].p_glass<0.1)
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
        if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].p_glass<0.1)
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
          if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,y,x)].p_glass<0.1)
          {
            nearest_cells.i_second = nearest_cells.i_first;
            nearest_cells.j_second = nearest_cells.j_first;
            ncells_found += 1;
          }
        }
        else 
        {
          if(same_obstacle == 0)
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
          if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].p_glass<0.1)
          {
            nearest_cells.i_second = nearest_cells.i_first;
            nearest_cells.j_second = nearest_cells.j_first;
            ncells_found += 1;
          }
        }
        else 
        {
          if(same_obstacle == 0)
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
  return range > max_range ? max_range : range; 
}

// Return the probability of being a glass for a cell
double get_glass_prob(map_t *map, int ci, int cj)
{
  if(!MAP_VALID(map,ci,cj))
    return 0.0;

  return map->cells[MAP_INDEX(map,ci,cj)].p_glass;
}
