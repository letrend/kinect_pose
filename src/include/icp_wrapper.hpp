#pragma once

#include "ICPOdometry.h"

class ICPCUDA{
public:
    ICPCUDA(int pWidth, int pHeight, Eigen::Matrix4d pose_init, float cx, float cy, float fx, float fy){
        icpOdom = new ICPOdometry(pWidth, pHeight, cx, cy, fx, fy);
        pose = pose_init;
        p = Sophus::SE3d(pose_init);
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        width = pWidth;
        height = pHeight;
    };
    
    ICPCUDA(size_t pWidth, size_t pHeight, float cx, float cy, float fx, float fy){
        icpOdom = new ICPOdometry(pWidth, pHeight, cx, cy, fx, fy);
        pose = Eigen::Matrix4d::Identity();
        p = Sophus::SE3d(pose);
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        width = pWidth;
        height = pHeight;
    };
    
    void setInitialPose(Eigen::Matrix4d pose_init){
        pose = pose_init;
    }
    
    void getPoseFromDepth(cv::Mat &depth0, cv::Mat &depth1){
         // depth maps need to be in milimeters
        depth0.convertTo(depth0,CV_16U);
        depth1.convertTo(depth1,CV_16U);
        
        // ICP
        icpOdom->initICPModel((unsigned short *)depth0.data, 20.0f);
        
        icpOdom->initICP((unsigned short *)depth1.data, 20.0f);

//        Eigen::Matrix< double, 3, 1 >  trans = pose.topRightCorner(3, 1);
//        Eigen::Matrix< double, 3, 3 > rot = pose.topLeftCorner(3, 3);
//
//        Sophus::SE3d p(rot,trans);

        icpOdom->getIncrementalTransformation(p,128,96);

        pose.topLeftCorner(3, 3) = p.rotationMatrix();
        pose.topRightCorner(3, 1) = p.translation();
    };
public:
    Eigen::Matrix4d getPose(){
        return pose;
    }
    Eigen::Matrix4d getPose_inv(){
        return pose.inverse();
    };
    
private:
    ICPOdometry *icpOdom;
    Eigen::Matrix4d pose;
    Sophus::SE3d p;
    cv::Mat depth0;
    cv::Mat depth1;
    std::vector<Eigen::Matrix< double, 3, 1 >> translations;
    size_t width,height;
};

