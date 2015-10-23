#include "functies.hpp"



void triangulate_stereo(const cv::Mat & K1, const cv::Mat & kc1, const cv::Mat & K2, const cv::Mat & kc2,
                                  const cv::Mat & Rt, const cv::Mat & T, const cv::Point2d & p1, const cv::Point2d & p2,
                                  cv::Point3d & p3d, double * distance)
{
    //to image camera coordinates
    cv::Mat inp1(1, 1, CV_64FC2), inp2(1, 1, CV_64FC2);
    inp1.at<cv::Vec2d>(0, 0) = cv::Vec2d(p1.x, p1.y);
    inp2.at<cv::Vec2d>(0, 0) = cv::Vec2d(p2.x, p2.y);
    cv::Mat outp1, outp2;
    cv::undistortPoints(inp1, outp1, K1, kc1);
    cv::undistortPoints(inp2, outp2, K2, kc2);
    assert(outp1.type()==CV_64FC2 && outp1.rows==1 && outp1.cols==1);
    assert(outp2.type()==CV_64FC2 && outp2.rows==1 && outp2.cols==1);
    const cv::Vec2d & outvec1 = outp1.at<cv::Vec2d>(0,0);
    const cv::Vec2d & outvec2 = outp2.at<cv::Vec2d>(0,0);
    cv::Point3d u1(outvec1[0], outvec1[1], 1.0);
    cv::Point3d u2(outvec2[0], outvec2[1], 1.0);

    //to world coordinates
    cv::Point3d w1 = u1;
    cv::Point3d w2 = cv::Point3d(cv::Mat(Rt*(cv::Mat(u2) - T)));

    //world rays
    cv::Point3d v1 = w1;
    cv::Point3d v2 = cv::Point3d(cv::Mat(Rt*cv::Mat(u2)));

    //compute ray-ray approximate intersection
    p3d = approximate_ray_intersection(v1, w1, v2, w2, distance);
}

cv::Point3d approximate_ray_intersection(const cv::Point3d & v1, const cv::Point3d & q1,
                                                    const cv::Point3d & v2, const cv::Point3d & q2,
                                                    double * distance/*, double * out_lambda1, double * out_lambda2*/)
{
    cv::Mat v1mat = cv::Mat(v1);
    cv::Mat v2mat = cv::Mat(v2);

    double v1tv1 = cv::Mat(v1mat.t()*v1mat).at<double>(0,0);
    double v2tv2 = cv::Mat(v2mat.t()*v2mat).at<double>(0,0);
    double v1tv2 = cv::Mat(v1mat.t()*v2mat).at<double>(0,0);
    double v2tv1 = cv::Mat(v2mat.t()*v1mat).at<double>(0,0);

    //cv::Mat V(2, 2, CV_64FC1);
    //V.at<double>(0,0) = v1tv1;  V.at<double>(0,1) = -v1tv2;
    //V.at<double>(1,0) = -v2tv1; V.at<double>(1,1) = v2tv2;
    //std::cout << " V: "<< V << std::endl;

    cv::Mat Vinv(2, 2, CV_64FC1);
    double detV = v1tv1*v2tv2 - v1tv2*v2tv1;
    Vinv.at<double>(0,0) = v2tv2/detV;  Vinv.at<double>(0,1) = v1tv2/detV;
    Vinv.at<double>(1,0) = v2tv1/detV; Vinv.at<double>(1,1) = v1tv1/detV;
    //std::cout << " V.inv(): "<< V.inv() << std::endl << " Vinv: " << Vinv << std::endl;

    //cv::Mat Q(2, 1, CV_64FC1);
    //Q.at<double>(0,0) = cv::Mat(v1mat.t()*(cv::Mat(q2-q1))).at<double>(0,0);
    //Q.at<double>(1,0) = cv::Mat(v2mat.t()*(cv::Mat(q1-q2))).at<double>(0,0);
    //std::cout << " Q: "<< Q << std::endl;

    cv::Point3d q2_q1 = q2 - q1;
    double Q1 = v1.x*q2_q1.x + v1.y*q2_q1.y + v1.z*q2_q1.z;
    double Q2 = -(v2.x*q2_q1.x + v2.y*q2_q1.y + v2.z*q2_q1.z);

    //cv::Mat L = V.inv()*Q;
    //cv::Mat L = Vinv*Q;
    //std::cout << " L: "<< L << std::endl;

    double lambda1 = (v2tv2 * Q1 + v1tv2 * Q2) /detV;
    double lambda2 = (v2tv1 * Q1 + v1tv1 * Q2) /detV;
    //std::cout << "lambda1: " << lambda1 << " lambda2: " << lambda2 << std::endl;

    //cv::Mat p1 = L.at<double>(0,0)*v1mat + cv::Mat(q1); //ray1
    //cv::Mat p2 = L.at<double>(1,0)*v2mat + cv::Mat(q2); //ray2
    //cv::Point3d p1 = L.at<double>(0,0)*v1 + q1; //ray1
    //cv::Point3d p2 = L.at<double>(1,0)*v2 + q2; //ray2
    cv::Point3d p1 = lambda1*v1 + q1; //ray1
    cv::Point3d p2 = lambda2*v2 + q2; //ray2

    //cv::Point3d p = cv::Point3d(cv::Mat((p1+p2)/2.0));
    cv::Point3d p = 0.5*(p1+p2);

    if (distance!=NULL)
    {
        *distance = cv::norm(p2-p1);
    }
    /*if (out_lambda1)
    {
        *out_lambda1 = lambda1;
    }
    if (out_lambda2)
    {
        *out_lambda2 = lambda2;
    }*/

    return p;
}
