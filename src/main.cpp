#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include "EtQ.hpp"
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/image_encodings.h>
void onCameraOpenError();
cv::Mat undistortImage(cv::Mat frame);
cv::Mat detectMarkers(cv::Mat frame);
cv::Mat detectPose(cv::Mat frame);
std::vector<std::vector<double>> EstimateSingleMarkerCameraPosition(cv::Vec3d &, cv::Vec3d &);

void estimateCameraPosition(cv::Mat, std::vector<std::vector<double>>&);
void onCameraUpdate(cv::Mat frame, image_transport::Publisher *);
void ondetectMarkersUpdate(cv::Mat frame, image_transport::Publisher *);
void onDetectPoseUpdate(cv::Mat frame, image_transport::Publisher *, ros::Publisher *);

float mtx[] = {1.76079567e+03, 0.00000000e+00, 5.62624364e+02, 0.00000000e+00, 2.36397397e+03, 3.71636030e+02, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};
float dst[] = {7.63044801e+00, -9.01413724e+01, -2.28321157e-01, -1.31753316e+00, 2.63183087e+03};

std::map<int, arucoPose> MarkerReferencePosition;
void onArucoMarkerUpdate(const geometry_msgs::PoseStampedConstPtr pose);
cv::Mat map1, map2;
cv::Mat cvMtx = cv::Mat(3, 3, CV_32FC1, mtx);
cv::Mat cvDst = cv::Mat(1, 5, CV_32FC1, dst);
cv::Mat RoI = cv::getOptimalNewCameraMatrix(cvMtx, cvDst, cv::Size(1280, 720), 1, cv::Size(1280, 720));
int main(int argv, char **argc)
{
    ros::init(argv, argc, "RAWImageNode");
    cv::VideoCapture vc;
    if (!vc.open(0))
        onCameraOpenError();
    vc.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    vc.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    vc.set(cv::CAP_PROP_FPS, 30);
    ros::NodeHandle nh;
    ros::Publisher position = nh.advertise<geometry_msgs::Pose>("camera/Pose", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image", 1);
    image_transport::Publisher aruco = it.advertise("/ArucoCodes", 1);
    image_transport::Publisher pose = it.advertise("/Estimatedpose", 1);
    ros::Subscriber arucoUpdateMarker = nh.subscribe("/MarkerUpdate", 1, onArucoMarkerUpdate);
    cv::initUndistortRectifyMap(cvMtx, cvDst, cv::Mat(), RoI, cv::Size(1280, 720), CV_32FC1, map1, map2);
    for (int i = 4; i >= 0; i--)
    {
        ROS_INFO("Waiting [%s] seconds for camera to initialise", std::to_string(i + 1).c_str());
        sleep(1);
    }
    ROS_INFO("Starting camera node");
    ros::Rate rate(30);
    cv::Mat frame;
    while (ros::ok())
    {
        vc.read(frame);
        if (pub.getNumSubscribers() > 0)
        {
            onCameraUpdate(frame, &pub);
        }
        if (aruco.getNumSubscribers() > 0)
        {
            ondetectMarkersUpdate(frame, &aruco);
        }
        onDetectPoseUpdate(frame,&pose,&position) ;
        ros::spinOnce();
        rate.sleep();
    }
}
cv::Mat undistortImage(cv::Mat frame)
{
    cv::Mat result;
    cv::remap(frame, result, map1, map2, cv::INTER_LINEAR);
    return result;
}
cv::Mat detectMarkers(cv::Mat frame)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejected);
    ROS_INFO("Found %i markers", markerIds.size());
    if (markerIds.size() > 0)
        ROS_INFO("First Marker ID: %i", markerIds[0]);
    for (int i = 0; i < markerCorners.size(); i++)
        for (int j = 0; j <= markerCorners[i].size(); j++)
            ROS_INFO("%i,%i,X:%i,Y%i", i, j, markerCorners[i][j].x, markerCorners[i][j].y);
    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    return frame;
}
cv::Mat detectPose(cv::Mat frame)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejected);
    std::vector<cv::Vec3d> rvecs, tvecs;
    ROS_INFO("LOOKING FOR ARUCO poses");
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cvMtx, cvDst, rvecs, tvecs);
    for (int i = 0; i < rvecs.size(); ++i)
    {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        ROS_INFO("drawing axis");
        cv::aruco::drawAxis(frame, cvMtx, cvDst, rvec, tvec, 0.1);
    }
    return frame;
}
void onCameraUpdate(cv::Mat frame, image_transport::Publisher *pub)
{
    cv::Mat undistortedImage;
    undistortedImage = undistortImage(frame);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    cv_ptr->header.stamp = time;
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.frame_id = "/image";
    cv_ptr->image = frame;
    pub->publish(cv_ptr->toImageMsg());
}
void ondetectMarkersUpdate(cv::Mat frame, image_transport::Publisher *pub)
{
    cv::Mat marker = detectMarkers(frame);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    cv_ptr->header.stamp = time;
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.frame_id = "/image";
    cv_ptr->image = marker;
    pub->publish(cv_ptr->toImageMsg());
}
void estimateCameraPosition(cv::Mat frame, std::vector<std::vector<double>> &Result)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejected);
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cvMtx, cvDst, rvecs, tvecs);
    ROS_INFO("Found %i amount of aruco codes",markerIds.size());
    if (markerIds.size() <= 0)
        return;
    Result.push_back(std::vector<double>());
    Result[0].push_back(0);
    Result[0].push_back(0);
    Result[0].push_back(0);
    Result.push_back(std::vector<double>());
    Result[1].push_back(0);
    Result[1].push_back(0);
    Result[1].push_back(0);
    for (int i = 0; i < markerIds.size(); i++)
    {   
        if(MarkerReferencePosition.find(markerIds[i]) != MarkerReferencePosition.end()){
            tvecs[i][0] += MarkerReferencePosition[markerIds[i]][0][0];
            tvecs[i][1] += MarkerReferencePosition[markerIds[i]][0][1];
            tvecs[i][2] += MarkerReferencePosition[markerIds[i]][0][2];
            
            rvecs[i][0] += MarkerReferencePosition[markerIds[i]][1][0];
            rvecs[i][1] += MarkerReferencePosition[markerIds[i]][1][1];
            rvecs[i][2] += MarkerReferencePosition[markerIds[i]][1][2];
        }
        std::vector<std::vector<double>> singleMarkerCameraPosition = EstimateSingleMarkerCameraPosition(tvecs[i], rvecs[i]);
        for (int y = 0; y < 2; y++)
            for (int j = 0; j < 3; j++)
                Result[y][j] += singleMarkerCameraPosition[y][j];
    }
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            Result[i][j] /= markerIds.size();
}
void onDetectPoseUpdate(cv::Mat frame, image_transport::Publisher *pub, ros::Publisher *pose)
{
    if (pub->getNumSubscribers() > 0)
    {
        cv::Mat pose = detectPose(frame);
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        ros::Time time = ros::Time::now();
        cv_ptr->header.stamp = time;
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.frame_id = "/image";
        cv_ptr->image = pose;
        pub->publish(cv_ptr->toImageMsg());
    }
    if (pose->getNumSubscribers() > 0)
    {
        std::vector<std::vector<double>> realCameraPosition;
        estimateCameraPosition(frame,realCameraPosition);
        geometry_msgs::Pose camPose;
        if(realCameraPosition.size() > 0){
        Quaternion rot = ToQuaternion(realCameraPosition[1][0], realCameraPosition[1][1], realCameraPosition[1][2]);
        camPose.position.x = realCameraPosition[0][0];
        camPose.position.y = realCameraPosition[0][1];
        camPose.position.z = realCameraPosition[0][2];
        camPose.orientation.w = rot.w;
        camPose.orientation.x = rot.x;
        camPose.orientation.z = rot.z;
        camPose.orientation.y = rot.y;
        }else { 
        camPose.position.x = std::numeric_limits<double>::min();
        camPose.position.y = std::numeric_limits<double>::min();
        camPose.position.z = std::numeric_limits<double>::min();
        camPose.orientation.w = std::numeric_limits<double>::min();
        camPose.orientation.x = std::numeric_limits<double>::min();
        camPose.orientation.z = std::numeric_limits<double>::min();
        camPose.orientation.y = std::numeric_limits<double>::min();
        }
        pose->publish(camPose);
    }
}

std::vector<std::vector<double>> EstimateSingleMarkerCameraPosition(cv::Vec3d &tvec, cv::Vec3d &rvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat H = cv::Mat(4,4,CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            H.at<float>(i, j) = R.at<float>(i, j);
    for (int i = 0; i < 3; i++)
        H.at<float>(i, 3) = tvec[i];
    H.at<float>(3,0)= 0;
    H.at<float>(3,1)= 0;
    H.at<float>(3,2)= 0;
    H.at<float>(3,3)= 1;
    H.inv();
    cv::Mat Rcam = cv::Mat(3,3,CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Rcam.at<float>(i, j) = H.at<float>(i, j);
    cv::Vec3d camRotEuler, camPos;
    cv::Rodrigues(Rcam, camRotEuler);
    for (int i = 0; i < 3; i++)
        camPos[i] = H.at<float>(i, 3);
    return {
        {camPos[2], camPos[0], camPos[1]},
        {camRotEuler[0], camRotEuler[1], camRotEuler[2]}};
}
void onCameraOpenError()
{
    ROS_ERROR("Camera is not working!");
    exit(-1);
}
void onArucoMarkerUpdate(const geometry_msgs::PoseStampedConstPtr pose)
{
    int ArucoId = stoi(pose->header.frame_id);
    arucoPose Aruco;
    Quaternion q;
    q.x = pose->pose.orientation.x;
    q.y = pose->pose.orientation.y;
    q.z = pose->pose.orientation.z;
    q.w = pose->pose.orientation.w;
    EulerAngles e = ToEulerAngles(q);
    MarkerReferencePosition[ArucoId] = {
        {
            pose->pose.position.x,
            pose->pose.position.y, 
            pose->pose.position.z
        },
        {
            e.roll,
            e.pitch, 
            e.yaw
            }
        };
}