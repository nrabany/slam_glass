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

void map_hough_lines(map_t *map, uint16_t minPoints)
{
    const char* filename = "../../../../../tests/maps/Map0.pgm";
    //  const char* filename = "../../../../../tests/maps/ex1.pgm";
    Mat src = imread(filename, 0); 

    // Mat src(map->size_x, map->size_y, CV_8UC1, map->gridData);

    // Here get a binary image by thresholding
    uchar intensityThresh = 200;
    Mat srcThresh, flipIm;
    threshold(src, srcThresh, intensityThresh, 255, THRESH_BINARY_INV);

    // Lines will be plot on cdst -----------------------------------------
    Mat cdst;
    cvtColor(srcThresh, cdst, CV_GRAY2BGR);
    //---------------------------------------------------------------------

    vector<Vec2f> lines;
    HoughLines(srcThresh, lines, 1, CV_PI / 180, minPoints, 0, 0);
    cout << "before: " << lines.size() << "\n"
         << endl;

    for (size_t i = 0; i < lines.size(); i++)
    {
        bool new_group = true;
        float rho = lines[i][0] * map->scale, theta = lines[i][1];

        if (map->nb_lines != 0)
        {
            for (int j = 0; j < map->nb_lines; j++)
            {
                double rho_diff = abs(rho - map->lines[j].rho);
                double theta_diff = abs(theta - map->lines[j].theta);
                double rho_add = abs(rho + map->lines[j].rho);
                // Here adjust parameters to group lines
                if ((rho_diff < 80 * map->scale && theta_diff < 3 * M_PI / 180) || (rho_add < 80 * map->scale && theta_diff - M_PI < 3 * M_PI / 180))
                {
                    new_group = false;
                    break;
                }
            }
        }

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

            cout << "line[" << map->nb_lines - 1 << "]    rho: " << rho << ", theta: " << theta << endl;

            /*-----DRAW LINES--------------------------------------------------------------------*/
            // Point pt1, pt2;
            // double a = cos(theta), b = sin(theta);
            // double x0 = a*rho, y0 = b*rho;
            // pt1.x = cvRound(x0 + map->size_y*(-b));
            // pt1.y = cvRound(y0 + map->size_x*(a));
            // pt2.x = cvRound(x0 - map->size_y*(-b));
            // pt2.y = cvRound(y0 - map->size_x*(a));
            // line( cdst, pt1, pt2, Scalar(0,0,255), 0.5, CV_AA);
            /*-----END DRAW LINES-----------------------------------------------------------------*/
        }
    }
    cout << "nb_lines " << map->nb_lines << "\n"
         << endl;

    // namedWindow("detected lines", WINDOW_NORMAL);
    // resizeWindow("detected lines", 1500, 1500);
    // flip before plot because map coordinates and image coordinates have y axis inverted
    // flip(cdst, flipIm, 0);
    // imshow("detected lines", flipIm);
    // waitKey();
}

double compute_incindent_angle(map_t *map, double oa, int ci, int cj, double min_err)
{
    // Find line on which is lying the cell
    // double xCell = MAP_WXGX(map, ci);
    // double yCell = MAP_WXGX(map, cj);
    double xCell = ci;
    double yCell = cj;
    cout << "(i,j): " << xCell << ", " << yCell << endl;

    // double min_err = 0.3;
    int line_index = -1;

    for (int i = 0; i < map->nb_lines; i++)
    {
        double theta = map->lines[i].theta;
        double rho = map->lines[i].rho;
        // cout << "HERE 2\n" << endl;

        double err = min_err;

        if (abs(theta) <= 0.05 || abs(theta - 2 * M_PI) <= 0.05)
            err = abs(xCell - rho);
        else if (abs(theta - M_PI) <= 0.05)
            err = abs(xCell + rho);
        else
            err = abs(yCell - (-xCell * cos(theta) + rho) / sin(theta));
        cout << "For line[" << i << "]   err " << err << ", minerr " << min_err << endl;

        if (err < min_err)
        {
            cout << "here " << err << endl;
            min_err = err;
            line_index = i;
        }
    }
    // If no close line return -1 : failure
    if (line_index == -1)
    {
        cout << "return" << endl;
        return -1;
    }
    // Here get absolute value and kepp incindent angle between 0° and 90°
    double incindent_angle = abs(map->lines[line_index].theta - oa);
    while (incindent_angle > M_PI / 2)
    {
        incindent_angle -= M_PI;
        incindent_angle = abs(incindent_angle);
    }
    // if(oa < M_PI/2.0 && map->lines[line_index].theta > M_PI*1.5)
    //     incindent_angle = 2*M_PI - incindent_angle;

    return incindent_angle;
}

int main()
{

    map_t *map = map_alloc();
    map->scale = 1;
    map->size_x = 100;
    map->size_y = map->size_x;
    map->gridData = (uint8_t *)malloc(sizeof(uint8_t) * 10 * map->size_x * map->size_y);
    cout << sizeof(map->gridData) << endl;
    map->nb_lines = 0;
    map->lines = (map_line_t *)malloc(sizeof(map_line_t) * 200);

    // uint8_t example[25] = {255,255,0,0,1, 0,0,0,1,0, 0,0,1,0,0, 0,1,0,0,0, 1,0,0,0,0};

    for (int i = 0; i < map->size_x; i++)
    {
        for (int j = 0; j < map->size_y; j++)
        {
            if (i == 5 || i == map->size_x - 5 || j == 5 || j == map->size_y - 5 || i == j) // || i==2)
            {
                map->gridData[MAP_INDEX(map, i, j)] = 0;
            }
            else
            {
                map->gridData[MAP_INDEX(map, i, j)] = 255;
            }
        }
    }

    // Mat src(Size(5,5),CV_8UC1,example);

    // cout << src << endl;
    // namedWindow("source",WINDOW_NORMAL);
    // resizeWindow("source", 1500,1500);
    // imshow("source", src);
    // waitKey();

    map_hough_lines(map, 50);
    double angle = compute_incindent_angle(map, M_PI * 3 / 4 * 0, 50, 5, 3);
    if (angle == -1)
        cout << "angle: " << angle << endl;
    else
        cout << "angle: " << angle / M_PI * 180 << endl;
    return 1;
    /*uint8_t data[] = {255,255,255,255,255, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 255,255,255,255,255};
    uint8_t uarr[] = {1,2,3,4,5,6,7,8,9,10,11,12};
    int rows = 5;



    
    int cols = 5;
    cv::Size sz(cols,rows);

    cv::Mat mat1(sz,CV_8UC1, data);
    cv::Mat mat2(rows, cols, CV_8UC1, data);

    std::cout<< "mat1: \n"<<mat1 << "\n\nmat2:\n" << mat2 << std::endl;
    return 0;*/
}
