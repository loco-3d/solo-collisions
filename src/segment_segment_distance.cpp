// Computes the distance between 2 segments, 
// given each as pairs of 3D points
#include <array>
#include <iostream>
#include <eigen3/Eigen/Core>

class Segment {       // Segment class
  public:             // Access specifier
    Eigen::Vector3d p0;  
    Eigen::Vector3d p1;  // Extremities as 3D eigen vectors

    Segment(Eigen::Vector3d p0, Eigen::Vector3d p1){
        this->p0 = p0;
        this->p1 = p1;
    }
};

/*
The segments are passed as 3D vectors, one for each endpoint
*/
void computeDistance(Segment seg1, Segment seg2, double& distanceResult)
{   
    /*------------- Setting up the variables -------------*/   
    //double distanceResult = INFINITY;

    // Direction vector of seg1
    Eigen::Matrix<double,3,1> u = seg1.p1 - seg1.p0;
    // Direction vector of seg2 
    Eigen::Matrix<double,3,1> v = seg2.p1 - seg2.p0;
    // Direction between segments origins
    Eigen::Matrix<double,3,1> w = seg1.p0 - seg2.p0;

    // Vector products
    double uTu = u.transpose()*u;
    double uTv = u.transpose()*v;
    double vTv = v.transpose()*v;
    double uTw = u.transpose()*w;
    double vTw = v.transpose()*w;

    // Solving denominators (den) and numerators (num)
    double den = uTu*vTv - uTv*uTv;
    double s_den = den;
    double t_den = den;
    double s_num = 0;
    double t_num = 0;

    double s_closest = 0;
    double t_closest = 0;
    
    // Setting threshold on denominator for parallel case
    double DEN_THRESHOLD = 1e-9;

    /*------------- Computing the closest points and shortest distance -------------*/
    // Parallel case
    if (den < DEN_THRESHOLD) {
        s_num =  0 ;
        s_den = 1 ;
        t_num = vTw ;
        t_den = vTv ;
    }
    else {
        s_num = uTv*vTw - vTv*uTw ;
        t_num = uTu*vTw - uTv*uTw ;
    }

    // Check the constraint s in [0,1]
    if (s_num < 0) {
        s_num = 0 ;     // constrain s to 0 
        t_num = vTw ;
        t_den = vTv ;
    }
    else if (s_num > s_den) {
        s_num = s_den ;  // constrain s to 1
        t_num = vTw + uTv ;
        t_den = vTv ;
    }

    // Check the constraint t in [0,1]
    if (t_num < 0){
        t_num = 0 ;  // constrain t to 0 
        // Re check constrain on s
        if (-uTw < 0){
            s_num = 0 ;
        }
        else if (-uTw > uTu) {
            s_num = s_den ;
        }
        else {
            s_num = -uTw ;
            s_den = uTu ;
        }
    }
    else if (t_num > t_den) {
        t_num = t_den ;  // constrain t to 1
        if (-uTw + uTv < 0) {
            s_num = 0 ;
        }
        else if ((-uTw + uTv) > uTu){
            s_num = s_den ;
        }
        else {
            s_num = -uTw + uTv ;
            s_den = uTu ;
        }
    }

    s_closest = (abs(s_num) < DEN_THRESHOLD ? 0.0 : s_num / s_den);
    t_closest = (abs(t_num) < DEN_THRESHOLD ? 0.0 : t_num / t_den);

    Eigen::Matrix<double,3,1> diff_at_closest = w + s_closest*u - t_closest*v;
    distanceResult = diff_at_closest.norm();
    //return distanceResult;
}

void computeDistanceFromPoints(double x10,
                               double y10, 
                               double z10,
                               double x11,
                               double y11, 
                               double z11,
                               double x20,
                               double y20,
                               double z20,
                               double x21,
                               double y21,
                               double z21,
                               double& distanceResult){
    Eigen::Vector3d p10(x10,y10,z10);
    Eigen::Vector3d p11(x11,y11,z11);
    Eigen::Vector3d p20(x20,y20,z20);
    Eigen::Vector3d p21(x21,y21,z21);

    Segment s0 = Segment(p10,p11);
    Segment s1 = Segment(p20,p21);

    computeDistance(s0,s1,distanceResult);
    
    }

int main(void) {
    Eigen::Vector3d p10(0,0,0);
    Eigen::Vector3d p11(1,1,1);
    Eigen::Vector3d p20(0,0,2);
    Eigen::Vector3d p21(0,0,3);

    Segment s1(p10,p11);
    Segment s2(p20,p21);

    double distResult = INFINITY;
    double distResultP = INFINITY;
    //double cDist = computeDistance(s1, s2);
    computeDistance(s1,s2, distResult);
    computeDistanceFromPoints(0,0,0,1,1,1,0,0,2,0,0,3, distResultP);

    std::cout << "Shortest distance (vectors) :\n" << distResult << std::endl;
    std::cout << "Shortest distance (doubles) :\n" << distResultP << std::endl;
}