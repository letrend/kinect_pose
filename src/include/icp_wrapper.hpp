#pragma once

#include "ICPOdometry.h"

class ICPCUDA{
public:
    ICPCUDA(int pWidth, int pHeight, Eigen::Matrix4d pose_init){
        icpOdom = new ICPOdometry(640, 480, 320, 240, 528, 528);
        pose = pose_init;
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        width = pWidth;
        height = pHeight;
    };
    
    ICPCUDA(size_t pWidth, size_t pHeight){
        icpOdom = new ICPOdometry(640, 480, 320, 240, 528, 528);
        pose = Eigen::Matrix4d::Identity();
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        width = pWidth;
        height = pHeight;
    };
    
    void setInitialPose(Eigen::Matrix4d pose_init){
        pose = pose_init;
    }
    
    void getPoseFromDepth(cv::Mat &depth0f, cv::Mat &depth1f){
         // depth maps need to be in milimeters
        depth0f.convertTo(depth0,CV_16U);
        depth1f.convertTo(depth1,CV_16U);
        
        // ICP
        icpOdom->initICPModel((unsigned short *)depth0.data, 20.0f);
        
        icpOdom->initICP((unsigned short *)depth1.data, 20.0f);

        Eigen::Matrix< double, 3, 1 >  trans = pose.topRightCorner(3, 1);
        Eigen::Matrix< double, 3, 3 > rot = pose.topLeftCorner(3, 3);

        Sophus::SE3d p(rot,trans);

        icpOdom->getIncrementalTransformation(p,128,96);

        translations.push_back(trans);
        
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
    cv::Mat depth0;
    cv::Mat depth1;
    std::vector<Eigen::Matrix< double, 3, 1 >> translations;
    size_t width,height;
};

