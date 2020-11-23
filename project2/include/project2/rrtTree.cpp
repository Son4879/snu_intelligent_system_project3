#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846
#include <iostream>
using namespace std;
double temp = 0; // this line is added
double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree()
{
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal)
{
    this->x_init = x_init;
    this->x_goal = x_goal;
    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
    root->backstep_count =0;

}

rrtTree::~rrtTree()
{
    for (int i = 1; i <= count; i++)
    {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin)
{
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin)
{
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++)
    {
        for (int j = 0; j < xSize; j++)
        {
            if (map.at<uchar>(i, j) < 125)
            {
                for (int k = i - margin; k <= i + margin; k++)
                {
                    for (int l = j - margin; l <= j + margin; l++)
                    {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize)
                        {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree()
{
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    for (int i = 1; i < this->count; i++)
    {
        idx_parent = this->ptrTable[i]->idx_parent;
        for (int j = 0; j < 10; j++)
        {
            double alpha = this->ptrTable[i]->alpha;
            double d = this->ptrTable[i]->d;
            double p1_th = this->ptrTable[idx_parent]->location.th + d * j / 10 * tan(alpha) / L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d * (j + 1) / 10 * tan(alpha) / L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L / tan(alpha) * (sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
            double p1_y = this->ptrTable[idx_parent]->location.y + L / tan(alpha) * (cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L / tan(alpha) * (sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
            double p2_y = this->ptrTable[idx_parent]->location.y + L / tan(alpha) * (cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res * (p1_y / res + map_origin_y)), (int)(Res * (p1_x / res + map_origin_x)));
            x2 = cv::Point((int)(Res * (p2_y / res + map_origin_y)), (int)(Res * (p2_x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res * 200, (int)Res * 200, (int)Res * 400, (int)Res * 400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path)
{
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res * (path[0].y / res + map_origin_y)), (int)(Res * (path[0].x / res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res * (path[path.size() - 1].y / res + map_origin_y)), (int)(Res * (path[path.size() - 1].x / res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for (int i = 1; i < this->count; i++)
    {
        idx_parent = this->ptrTable[i]->idx_parent;
        for (int j = 0; j < 10; j++)
        {
            double alpha = this->ptrTable[i]->alpha;
            double d = this->ptrTable[i]->d;
            double p1_th = this->ptrTable[idx_parent]->location.th + d * j / 10 * tan(alpha) / L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d * (j + 1) / 10 * tan(alpha) / L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L / tan(alpha) * (sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
            double p1_y = this->ptrTable[idx_parent]->location.y + L / tan(alpha) * (cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L / tan(alpha) * (sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
            double p2_y = this->ptrTable[idx_parent]->location.y + L / tan(alpha) * (cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res * (p1_y / res + map_origin_y)), (int)(Res * (p1_x / res + map_origin_x)));
            x2 = cv::Point((int)(Res * (p2_y / res + map_origin_y)), (int)(Res * (p2_x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }

    thickness = 3;
    for (int i = 1; i < path.size(); i++)
    {
        for (int j = 0; j < 10; j++)
        {
            double alpha = path[i].alpha;
            double d = path[i].d;
            double p1_th = path[i - 1].th + d * j / 10 * tan(alpha) / L; // R = L/tan(alpha)
            double p2_th = path[i - 1].th + d * (j + 1) / 10 * tan(alpha) / L;
            double p1_x = path[i - 1].x + L / tan(alpha) * (sin(p1_th) - sin(path[i - 1].th));
            double p1_y = path[i - 1].y + L / tan(alpha) * (cos(path[i - 1].th) - cos(p1_th));
            double p2_x = path[i - 1].x + L / tan(alpha) * (sin(p2_th) - sin(path[i - 1].th));
            double p2_y = path[i - 1].y + L / tan(alpha) * (cos(path[i - 1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res * (p1_y / res + map_origin_y)), (int)(Res * (p1_x / res + map_origin_x)));
            x2 = cv::Point((int)(Res * (p2_y / res + map_origin_y)), (int)(Res * (p2_x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res * 200, (int)Res * 200, (int)Res * 400, (int)Res * 400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d)
{
    //TODO
    ptrTable[count] = new node;
    ptrTable[count]->idx = count;
    ptrTable[count]->rand = x_rand;
    ptrTable[count]->location = x_new;
    ptrTable[count]->idx_parent = idx_near;
    ptrTable[count]->alpha = alpha;
    ptrTable[count]->d = d;
    ptrTable[count]->backstep_count = 0;
    count++;
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep)
{
    //TODO
    std::cout <<"count is : "<<count<<std::endl;
    int succeed = 0;
    int i =0;
    std::cout <<"gen started "<<i<<std::endl;
    for (i = 0; i < K; i++)
    {
        //goal
        if (pow(x_goal.x - ptrTable[count - 1]->location.x, 2) + pow(x_goal.y - ptrTable[count - 1]->location.y, 2) < 0.04)
        {
            std::cout <<"path found"<<std::endl;
            succeed = 1;
            break;
        }
        int x_near_index = -1;
        point x_rand;
        x_rand = randomState(x_max, x_min, y_max, y_min); //random point generate
        x_near_index = nearestNeighbor(x_rand, MaxStep);  // nearest neighbor search
        if (x_near_index == -1)
        {
//            i--;
            continue;
        }
//cout << "near found" << endl;
//if this point can't make randompath more than 20, do not use
while(ptrTable[x_near_index]->backstep_count > 20)
{
    if(x_near_index == 0)
        break;
    x_near_index = ptrTable[x_near_index]->idx_parent;
}

        double out[5];
        int valid = 0;
        valid = randompath(out, ptrTable[x_near_index]->location, x_rand, MaxStep); //random path generation
        if (valid == 0)
        {
//            i--;
//count backstep_count when it cannot make randompath
        ptrTable[x_near_index]->backstep_count ++;
            continue;
        }
//cout << "find RP" << endl;
        point x_new;
        x_new.x = out[0];
        x_new.y = out[1];
        x_new.th = out[2];
//cout << "is it here? " << count << endl;
        addVertex(x_new, x_rand, x_near_index, out[3], out[4]);
//cout << (i+1) << "th add success" << endl;
    }
    std::cout <<"loop is over "<< i<<std::endl;
    std::cout <<"goal: "<<x_goal.x<<" "<<x_goal.y<<" "<<x_goal.th<<std::endl;
    int ccc =nearestNeighbor(x_goal, 999);
std::cout <<"nearestNeighbor: "<<ccc<<std::endl;
    
    std::cout << ptrTable[ccc]->location.x <<" "<<ptrTable[ccc]->location.y<<std::endl;
//        cout << "loop is finished might be failed" << endl; //- for debug
    return succeed;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min)
{
    //TODO
    point random;
//    if (count % 20 == 0) // goal biased
//        return x_goal;

    random.x = (double)rand() / (RAND_MAX) * (x_max - x_min) + x_min;
    random.y = (double)rand() / (RAND_MAX) * (y_max - y_min) + y_min;
//    random.th = atan2(random.y, random.x);
    return random;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep)
{
    //TODO
    double maxR = L / tan(max_alpha); //positive
    int nN = -1;                      //nearest Neighbor
    double nd = 99999999.0;           //nearest distance
    double beta = MaxStep / maxR;
    for (int i = 0; i < count; i++)
    {
        double temp = sqrt(pow(ptrTable[i]->location.x - x_rand.x, 2) + pow(ptrTable[i]->location.y - x_rand.y, 2)); //distance
        if (temp < nd)                                                                                         //to reduce calculation
        {
/*            //checking 2 direction (left & right)
            double x_left_center = ptrTable[i]->location.x - maxR * sin(ptrTable[i]->location.th);
            double y_left_center = ptrTable[i]->location.y + maxR * cos(ptrTable[i]->location.th);
            double x_right_center = ptrTable[i]->location.x + maxR * sin(ptrTable[i]->location.th);
            double y_right_center = ptrTable[i]->location.y - maxR * cos(ptrTable[i]->location.th);
            double temp_left = pow(x_left_center - x_rand.x, 2) + pow(y_left_center - x_rand.y, 2);
            double temp_right = pow(x_right_center - x_rand.x, 2) + pow(y_right_center - x_rand.y, 2);
            if (temp_left > pow(maxR, 2) && temp_right > pow(maxR, 2)) //if it's accessible
            {
                nd = temp;
                nN = i;
                //cout<<"nearest neighbor reachable! "<< nN<<endl; - for debug
            }*/

  	    double theta;
	    if (x_rand.y - ptrTable[i]->location.y > 0)
	        theta = acos((x_rand.x-ptrTable[i]->location.x)/(sqrt(pow(x_rand.x - ptrTable[i]->location.x,2.0) + pow(x_rand.y - ptrTable[i]->location.y,2.0))));
	    else if (x_rand.y - ptrTable[i]->location.y < 0)
		theta = 2*PI - acos((x_rand.x-ptrTable[i]->location.x)/(sqrt(pow(x_rand.x - ptrTable[i]->location.x,2.0) + pow(x_rand.y - ptrTable[i]->location.y,2.0))));
	    else
	    {
		if(x_rand.x - ptrTable[i]->location.x > 0)
	    	    theta = 0.0;
		else
	            theta = PI;
    	    }
	    double error = theta - ptrTable[i]->location.th;
	    if (error > PI) error = error - 2.0 * PI;
    	    else if (error < (-1.0) * PI) error = error + 2.0 * PI;
	    if (error <= beta && error >= -beta)
	    {
		nd = temp;
		nN = i;
	    }


        }
    }
    //cout<<"nN "<<nN<<" "; - for debug
    return nN;
}
int rrtTree::nearestNeighbor(point x_rand)
{
    int nN = 0;                                                                                          //nearest Neighbor
    double nd = pow(x_rand.x - ptrTable[0]->location.x, 2) + pow(x_rand.y - ptrTable[0]->location.y, 2); //nearest distance
    for (int i = 1; i < count; i++)
    {
        double temp = pow(x_rand.x - ptrTable[i]->location.x, 2) + pow(x_rand.y - ptrTable[i]->location.y, 2);
        if (temp < nd)
        {
            nd = temp;
            nN = i;
        }
    }
    return nN;
}

int rrtTree::randompath(double *out, point x_near, point x_rand, double MaxStep)
{
    //TODO
    // create some random paths starting from x_near
    int howmany = 30;
    double *alpha = new double[howmany];
    double *d = new double[howmany];
    double *R = new double[howmany];
    double *x_where = new double[howmany];
    double *y_where = new double[howmany];
    double x_center;
    double y_center;
    double *theta = new double[howmany];
    point *p = new point[howmany];
    int best = -1;
    double best_val = 99999999;
    //random pathpoint generate
    for (int i = 0; i < howmany; i++)
    {
        alpha[i] = (((double)rand() / RAND_MAX) * 2 - 1) * max_alpha;
        R[i] = L / tan(alpha[i]);
        d[i] = (double)rand() / RAND_MAX * MaxStep;
        double beta = d[i] / R[i];
        x_center = x_near.x - R[i] * sin(x_near.th);
        y_center = x_near.y + R[i] * cos(x_near.th);
        x_where[i] = x_center + R[i] * sin(x_near.th + beta);
        y_where[i] = y_center - R[i] * cos(x_near.th + beta);
        theta[i] = x_near.th + d[i] / R[i];
        p[i].x = x_where[i];
        p[i].y = y_where[i];
        p[i].th = theta[i];
    }
    for (int i = 0; i < howmany; i++)
    {
        if (!isCollision(x_near, p[i], d[i], R[i]))
        {
            if (pow(x_rand.x - p[i].x, 2) + pow(x_rand.y - p[i].y, 2) < best_val)
            {
                best = i;
                best_val = pow(x_rand.x - p[i].x, 2) + pow(x_rand.y - p[i].y, 2);
            }
        }
    }
    if (best == -1)
        return 0;
    out[0] = x_where[best]; //x
    out[1] = y_where[best]; //y
    out[2] = theta[best];   //theta
    out[3] = alpha[best];   //alpha
    out[4] = d[best];       //d

    delete[] alpha;
    delete[] d;
    delete[] R;
    delete[] x_where;
    delete[] y_where;
    delete[] theta;
    delete[] p;

    return 1;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R)
{
    //TODO
    double alpha = atan(L / R);
    double x_center = x1.x - R * sin(x1.th);
    double y_center = x1.y + R * cos(x1.th);
    double beta = d / R;
    int map_origin_x_int = int(map_origin_x);
    int map_origin_y_int = int(map_origin_y);
    int howmany = 10;
    for (int i = 0; i < howmany; i++)
    {
        double x_where = x_center + R * sin(x1.th + (i + 1) * beta / (howmany));
        double y_where = y_center - R * cos(x1.th + (i + 1) * beta / (howmany));
        if (!map.at<uchar>(map_origin_x_int + x_where / res + L * cos(x1.th + (i + 1) * beta / (howmany)), map_origin_y_int + y_where / res + L * sin(x1.th + (i + 1) * beta / (howmany))))
            return true;
    }
    return false;
}

std::vector<traj> rrtTree::backtracking_traj()
{
    //TODO
    std::vector<traj> path;
    int id = 0;
    for (int i = 0; i < count; i++)
    {
        double dist = pow(ptrTable[i]->location.x - x_goal.x, 2) + pow(ptrTable[i]->location.y - x_goal.y, 2);
        if (dist <= 0.04)
        {
            id = i;
            break;
        }
    }
    traj T;
    do
    {
        T.x = ptrTable[id]->location.x;
        T.y = ptrTable[id]->location.y;
        T.th = atan2(T.y, T.x);
        T.alpha = ptrTable[id]->alpha;
        T.d = ptrTable[id]->d;
        path.push_back(T);
        id = ptrTable[id]->idx_parent;
    } while (id != 0);

    T.x = ptrTable[0]->location.x;
    T.y = ptrTable[0]->location.y;
    T.th = atan2(T.y, T.x);
    T.alpha = ptrTable[0]->alpha;
    T.d = ptrTable[0]->d;
    path.push_back(T);

    return path;
}
