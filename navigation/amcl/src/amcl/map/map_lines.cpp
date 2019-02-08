#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "amcl/map/map.h"
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace cv;
using namespace std;

/* Find lines in map. Everything is in map coordinates and scale */
void map_hough_lines(map_t *map, uint16_t minPoints)
{
    Mat src(map->size_x, map->size_y, CV_8UC1, map->gridData);

    // Here get a binary image by thresholding (Filter to have only obstacles with p(glass)>0.1)
    uchar intensityThresh = 90;
    Mat srcThresh; // = src;
    threshold(src, srcThresh, intensityThresh, 255, THRESH_BINARY_INV);

    //---------------------------------------------------------------------

    vector<Vec2f> lines;
    HoughLines(srcThresh, lines, 1, CV_PI / 180, minPoints, 0, 0);
    cout << "before: " << lines.size() << "   \n"
         << endl;

    // Lines will be plot on cdst -----------------------------------------
    Mat cdst;
    cvtColor(srcThresh, cdst, CV_GRAY2BGR);
    Mat showMap;
    cvtColor(src, showMap, CV_GRAY2BGR);
    showMap =  Scalar::all(255) - showMap;

    //---------------------------------------------------------------------

    for (size_t i = 0; i < lines.size(); i++)
    {
        bool new_group = true;
        float rho = lines[i][0], theta = lines[i][1];

        // Make sure that rho is positive
        if(rho<0)
        {
            rho = abs(rho);
            theta += M_PI;
        }
        // Make sure that 0 < theta < 2*PI
        if(theta<0)
            theta += 2*M_PI;
        while(theta>2*M_PI)
            theta -= 2*M_PI;

        for (int j = 0; j < map->nb_lines; j++)
        {
            double rho_diff = abs(rho - map->lines[j].rho);
            double theta_diff = abs(theta - map->lines[j].theta);
            double rho_add = abs(rho + map->lines[j].rho);
            // Here adjust parameters to group lines. !!rho_diff is in pixel!!
            if ((rho_diff < 80 && theta_diff < 3 * M_PI / 180) || (rho_add < 80 && theta_diff - M_PI < 3 * M_PI / 180))
            {
                new_group = false;
                break;
            }
        }

        // If line doesn't belong to a group of line, create a new group
        if (new_group == true)
        {
            map->nb_lines += 1;

            if (map->nb_lines > 1999)
            {
                cout << "More than 1999 groups of lines -> break because memory insufficient" << endl;
                break;
            }

            map_line_t mline;
            mline.rho = rho;
            mline.theta = theta;
            map->lines[map->nb_lines - 1] = mline;

            /*-----DRAW LINES--------------------------------------------------------------------*/
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + map->size_y * (-b));
            pt1.y = cvRound(y0 + map->size_x * (a));
            pt2.x = cvRound(x0 - map->size_y * (-b));
            pt2.y = cvRound(y0 - map->size_x * (a));
            line(cdst, pt1, pt2, Scalar(0, 0, 255), 0.5, CV_AA);
            line(showMap, pt1, pt2, Scalar(0, 0, 255), 0.5, CV_AA);
            /*-----END DRAW LINES-----------------------------------------------------------------*/
        }
    }
    cout << "nb_lines " << map->nb_lines << "  \n"
         << endl;
    
    // Print lines
    for(int i=0;i<map->nb_lines;i++)
    {
        cout << "line " << i << "-> theta: " << map->lines[i].theta << " , rho: " << map->lines[i].rho << " " << endl;
    }

    /* ------ Visualize lines on image -------------*/
    // namedWindow("flipped detected lines", WINDOW_NORMAL);
    // resizeWindow("flipped detected lines", 1500, 1500);
    // /*flip before plot because map coordinates and image coordinates have y axis inverted*/
    // Mat flipIm;
    // flip(cdst, flipIm, 0);
    // flip(showMap, flipIm, 0);
    // flip(src, src, 0);
    // imshow("flipped detected lines", flipIm);

    // waitKey();
    /* ----------------------------------------------*/
}

/* Compute incindent angle. Everything is in map coordinates and scale because only angle of robot
 * is used to dtermine incindent angle. !!min_err is in pixel here!! */
double compute_incindent_angle(map_t *map, double oa, int ci, int cj, double min_err)
{
    int line_index = -1;
    // cout << "pos: (" << ci << " , " << cj << ") " << endl;
    for (int i = 0; i < map->nb_lines; i++)
    {
        double theta = map->lines[i].theta;
        double rho = map->lines[i].rho;

        double err = min_err;

        if (abs(theta) <= 0.005 || abs(theta - 2 * M_PI) <= 0.005)
            err = abs(ci - rho);
        else if (abs(theta - M_PI) <= 0.005)
            err = abs(ci + rho);
        else if (abs(theta) <= 0.7 || abs(theta - 2 * M_PI) <= 0.7)
            err = abs((cj*sin(theta) + ci*cos(theta)) - rho);
        else
            err = abs(cj - (-ci * cos(theta) + rho) / sin(theta));

        // double err = abs(cj * sin(theta) - (-ci * cos(theta) + rho));

        // cout << "For line[" << i << "]   err " << err << ", minerr " << min_err << endl;

        if (err < min_err)
        {
            // cout << "here " << err << endl;
            min_err = err;
            line_index = i;
        }
    }
    // If no close line return -1 : failure
    if (line_index == -1)
    {
        // cout << "return" << endl;
        return -1;
    }
    // Here get absolute value and keep incindent angle between 0° and 90°
    double incindent_angle = abs(map->lines[line_index].theta - oa);
    while (incindent_angle > M_PI / 2)
    {
        incindent_angle -= M_PI;
        incindent_angle = abs(incindent_angle);
    }

    return incindent_angle;
}

double compute_std(double angle, double range)
{
    double angle_deg = angle * 180 / M_PI;
    double max_angle = (range <= 3) ? -3.05 * range + 11.6 : 1;
    if (angle_deg != -1 && angle_deg < max_angle)
        return 0.00135 * range + 0.0015 * angle_deg + 0.00105;

    return 0.0014;
}

double compute_p_can_see(double angle, double range)
{
    if (angle != -1 && range == 0)
        return 0.5;

    double max_angle = (range <= 3) ? -4 * range + 15 : -4 * 3 + 15;
    max_angle = max_angle * M_PI / 180.0;
    if (angle < max_angle)
        return 1;
    if (angle > 6 * max_angle)
        return 0;

    double lambda = 7.0 * range;
    double angle0 = max_angle * (1 + 2 / range);
    return 1 - 1 / (1 + exp(-lambda * (angle - angle0)));
}

double compute_p_can_see_thresh(double angle, double range, double angle_max)
{
    if (angle == -1)
        return 0.5;

    // double max_angle = (range <= 5) ? -10 * range + 55 : -10 * 5 + 55;
    // double max_angle = 90;
    angle_max = angle_max * M_PI / 180.0;

    // double beta = 4;
    if (angle < angle_max)
        return 1;
    
    /* // Smoothing
    if (angle < angle_max - 2 * M_PI / 180.0)
        return 1;
    else if (angle < angle_max + 2 * M_PI / 180.0)
        return (angle_max + 2 * M_PI / 180.0 - angle) / (4 * M_PI / 180.0);
    */

    // else if(angle < 3/2*max_angle)
    //     return 3 - 2 / max_angle * angle;
    // else if(angle < beta*max_angle)
    // {   
    //     return - 1 / (max_angle*(beta-1)) * angle + beta / (beta-1);
    // }

    return 0;
}

double compute_p_can_see_wide(double angle, double range)
{
    if (angle == -1)
        return 0.5;

    // double max_angle = (range <= 5) ? -6.5 * range + 35 : -6.6 * 5 + 35;
    double max_angle = (range <= 3) ? -4 * range + 15 : -4 * 3 + 15;
    max_angle = max_angle * M_PI / 180.0;

    double beta = 4;
    if (angle < max_angle)
        return 1;
    // else if(angle < 3/2*max_angle)
    //     return 3 - 2 / max_angle * angle;
    else if(angle < beta*max_angle)
    {   
        // cout << "max_angle: " << max_angle << ", angle: " << angle << ", p: " << angle / (max_angle*(4-1)) - 1 / (4-1) << endl;
        return - 1 / (max_angle*(beta-1)) * angle + beta / (beta-1);
    }
    return 0;
}