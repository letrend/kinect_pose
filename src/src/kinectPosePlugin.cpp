#include <mutex>
#include "kinectPosePlugin.hpp"

KinectPosePlugin::KinectPosePlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QPushButton *resetpose = new QPushButton(tr("reset pose"));
    resetpose->setObjectName("resetPose");
    connect(resetpose, SIGNAL(clicked()), this, SLOT(resetPose()));
    frameLayout->addWidget(resetpose);

    QLabel *speed = new QLabel(tr("speed: "));
    speed->setObjectName("speed");
    frameLayout->addWidget(speed);

    QCheckBox *renderimages = new QCheckBox(tr("render images"));
    renderimages->setObjectName("renderimages");
    connect(renderimages, SIGNAL(clicked()), this, SLOT(renderImages()));
    frameLayout->addWidget(renderimages);

    QCheckBox *showliveview = new QCheckBox(tr("show live view"));
    showliveview->setObjectName("showliveview");
    connect(showliveview, SIGNAL(clicked()), this, SLOT(showLiveView()));
    frameLayout->addWidget(showliveview);

    QLabel *rgbimage = new QLabel(tr("rgbimage"));
    rgbimage->setObjectName("rgbimage");
    frameLayout->addWidget(rgbimage);

    QLabel *depthimage = new QLabel(tr("depthimage"));
    depthimage->setObjectName("depthimage");
    frameLayout->addWidget(depthimage);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "kinect pose icpcuda",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

    device = boost::shared_ptr<MyFreenectDevice>(new MyFreenectDevice);
    device->updateFrames();
    device->getDepthMM(depth0);
    icpcuda = boost::shared_ptr<ICPCUDA>(new ICPCUDA(depth0.cols,depth0.rows, device->irCameraParams.cx,
                                                     device->irCameraParams.cy, device->irCameraParams.fx,
                                                     device->irCameraParams.fy));

    odometry_thread = boost::shared_ptr<std::thread>(new std::thread(&KinectPosePlugin::poseEstimation, this));

    QObject::connect(this, SIGNAL(dataReady(float)), this, SLOT(updateData(float)));
}

KinectPosePlugin::~KinectPosePlugin(){
    getPose = false;
    odometry_thread->join();
}

void KinectPosePlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void KinectPosePlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

void KinectPosePlugin::resetPose(){
    Matrix4d newPose;
    newPose = Matrix4d::Identity();
    icpcuda->setInitialPose(newPose);
}

void KinectPosePlugin::renderImages(){
    QCheckBox* w = this->findChild<QCheckBox*>("renderimages");
    renderImages_flag = w->isChecked();
}

void KinectPosePlugin::showLiveView(){
    QCheckBox* w = this->findChild<QCheckBox*>("showliveview");
    showLiveView_flag = w->isChecked();
}

void KinectPosePlugin::poseEstimation(){
    ROS_INFO("start pose estimation");
    while(getPose){
        device->updateFrames();
        device->getDepthMM(depth1);
        device->getRgbMapped2Depth(rgb);
        icpcuda->getPoseFromDepth(depth0,depth1);
        emit dataReady(icpcuda->mean_time);
        pose = icpcuda->getPose();
//        ROS_INFO_STREAM_THROTTLE(1.0,"speed: " << icpcuda->mean_time << "\npose\n" << pose);
        std::swap(depth0, depth1);
    }
    ROS_INFO("stop pose estimation");
}

void KinectPosePlugin::publishModel(){
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "kinect";
    mesh.ns = "kinect_v2";
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 1.0;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration();
    mesh.header.stamp = ros::Time::now();
    mesh.id = 0;
    Quaterniond q(rot);
    mesh.pose.orientation.x = 0;
    mesh.pose.orientation.y = 0;
    mesh.pose.orientation.z = 0;
    mesh.pose.orientation.w = 1;
    mesh.mesh_resource = "package://kinect_pose/models/kinect_v2.STL";
    marker_visualization_pub.publish(mesh);
}

void KinectPosePlugin::updateData(float mean_time){
    QLabel *speed = this->findChild<QLabel *>("speed");
    speed->setText("speed: " + QString::number(mean_time) + " ms");
    speed->repaint();

    trans = pose.topRightCorner(3,1);
    rot = pose.topLeftCorner(3,3);
    Quaterniond q(rot);

    ros::Time time = ros::Time::now();
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(trans(0), trans(1), trans(2)));
    tf.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_broadcaster.sendTransform(tf::StampedTransform(tf, time, "world", "kinect"));

    if(renderImages_flag) {
        QLabel *rgbmage = this->findChild<QLabel *>("rgbimage");
        QLabel *depthimage = this->findChild<QLabel *>("depthimage");
        int w = rgb.cols;
        int h = rgb.rows;
        QImage qim_rgb(w, h, QImage::Format_RGB32), qim_depth(w, h, QImage::Format_RGB32);;
        unsigned short *rgb_ = (unsigned short *) rgb.data;
        unsigned short *depth_ = (unsigned short *) depth1.data;
        QRgb pixel;
        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                // to meter divided by maximum range times max pixel value
                int distance = (int) (depth_[i + w * j] / 1000.0f / 4.5f * 255.0f);
                pixel = qRgb(distance, distance, distance);
                qim_depth.setPixel(i, j, pixel);
                int blue = (int) rgb_[0 + 3 * (i + w * j)];
                int green = (int) rgb_[1 + 3 * (i + w * j)];
                int red = (int) rgb_[2 + 3 * (i + w * j)];
                pixel = qRgb(red, green, blue);
                qim_rgb.setPixel(i, j, pixel);
            }
        }
        QPixmap pixmap0 = QPixmap::fromImage(qim_rgb);
        rgbmage->setPixmap(pixmap0);
        rgbmage->repaint();
        QPixmap pixmap1 = QPixmap::fromImage(qim_depth);
        depthimage->setPixmap(pixmap1);
        depthimage->repaint();
    }

    if(showLiveView_flag){
        visualization_msgs::Marker points;
        points.header.frame_id = "kinect";
        points.ns = "RGBD";
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.01;
        points.scale.y = 0.01;
        points.lifetime = ros::Duration();
        points.header.stamp = ros::Time::now();
        points.id = 1;
        points.pose.position.x = trans(0);
        points.pose.position.y = trans(1);
        points.pose.position.z = trans(2);
        points.pose.orientation.x = q.x();
        points.pose.orientation.y = q.y();
        points.pose.orientation.z = q.z();
        points.pose.orientation.w = q.w();
        int w = rgb.cols;
        int h = rgb.rows;
        unsigned short *rgb_ = (unsigned short *) rgb.data;
        unsigned short *depth_ = (unsigned short *) depth1.data;

        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                Vector3d pos3D;
                pos3D = device->projectTo3D(i, j);
                pos3D *= depth_[i + w * j] / 1000.0f;
                geometry_msgs::Point p;
                p.x = pos3D(0);
                p.y = pos3D(1);
                p.z = pos3D(2);
                points.points.push_back(p);
                std_msgs::ColorRGBA c;
                c.b = rgb_[0 + 3 * (i + w * j)] / 255.0f;
                c.g = rgb_[1 + 3 * (i + w * j)] / 255.0f;
                c.r = rgb_[2 + 3 * (i + w * j)] / 255.0f;
                c.a = 1.0;
                points.colors.push_back(c);
            }
        }
        marker_visualization_pub.publish(points);
    }

//    ROS_INFO_STREAM_THROTTLE(1.0,pose);
    publishModel();
}

PLUGINLIB_EXPORT_CLASS(KinectPosePlugin, rviz::Panel)