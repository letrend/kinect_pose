#include "kinect.hpp"
#include "icp_wrapper.hpp"
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char *argv[]) {
    Mat rgb, depth0, depth1;

    MyFreenectDevice device;
    device.updateFrames();
    device.getDepthMM(depth0);
    ICPCUDA icpcuda(depth0.cols,depth0.rows, device.irCameraParams.cx, device.irCameraParams.cy,
                    device.irCameraParams.fx, device.irCameraParams.fy);


    char k;
    while(k!=27){
        device.updateFrames();
        device.getDepthMM(depth1);
        device.getRgbMapped2Depth(rgb);
        imshow("rgb", rgb);
        imshow("depth0", depth0*1000.0f);
        imshow("depth1", depth1*1000.0f);
        k = waitKey(1);
        icpcuda.getPoseFromDepth(depth0,depth1);
        cout << icpcuda.getPose() << endl;
        std::swap(depth0,depth1);
    }
}
