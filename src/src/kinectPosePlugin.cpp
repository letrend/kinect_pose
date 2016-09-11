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

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

    device = boost::shared_ptr<MyFreenectDevice>(new MyFreenectDevice);
    device->updateFrames();
    device->getDepthMM(depth0);
    icpcuda = boost::shared_ptr<ICPCUDA>(new ICPCUDA(depth0.cols,depth0.rows));

    odometry_thread = boost::shared_ptr<std::thread>(new std::thread(&KinectPosePlugin::poseEstimation, this));
}

KinectPosePlugin::~KinectPosePlugin(){
    odometry_thread->join();
}

void KinectPosePlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void KinectPosePlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

void KinectPosePlugin::resetPose(){
    QPushButton* w = this->findChild<QPushButton*>("initwalkcontroller");
    w->setText("fuck me");
}

void KinectPosePlugin::poseEstimation(){
    ROS_INFO("start pose estimation");
    while(getPose){
        device->getDepthMM(depth1);
        device->getRgbMapped2Depth(rgb);
        icpcuda->getPoseFromDepth(depth0,depth1);
        pose = icpcuda->getPose();
        depth0 = depth1;
        publishModel();
    }
    ROS_INFO("stop pose estimation");
}

void KinectPosePlugin::publishModel(){
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "world";
    mesh.ns = "kinect_v2";
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration();
    mesh.header.stamp = ros::Time::now();
    mesh.id = 0;
    Vector3d trans = pose.topRightCorner(3,1);
    Matrix3d rot = pose.topLeftCorner(3,3);
    Quaterniond q(rot);
    mesh.pose.position.x = trans(0);
    mesh.pose.position.y = trans(1);
    mesh.pose.position.z = trans(2);
    mesh.pose.orientation.x = q.x();
    mesh.pose.orientation.y = q.y();
    mesh.pose.orientation.z = q.z();
    mesh.pose.orientation.w = q.w();
    mesh.mesh_resource = "package://kinect_pose/models/kinect_v2.STL";
    marker_visualization_pub.publish(mesh);
}

PLUGINLIB_EXPORT_CLASS(KinectPosePlugin, rviz::Panel)