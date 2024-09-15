/**
 * @file orb_slam3_interface.cpp
 * @brief Implementation of the ORBSLAM3Interface class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"

namespace ORB_SLAM3_Wrapper
{
    ORBSLAM3Interface::ORBSLAM3Interface(const std::string &strVocFile,
                                         const std::string &strSettingsFile,
                                         ORB_SLAM3::System::eSensor sensor,
                                         bool bUseViewer,
                                         bool rosViz,
                                         double robotX,
                                         double robotY,
                                         std::string globalFrame,
                                         std::string odomFrame,
                                         std::string robotFrame)
        : strVocFile_(strVocFile),
          strSettingsFile_(strSettingsFile),
          sensor_(sensor),
          bUseViewer_(bUseViewer),
          rosViz_(rosViz),
          robotX_(robotX),
          robotY_(robotY),
          globalFrame_(globalFrame),
          odomFrame_(odomFrame),
          robotFrame_(robotFrame)
    {
        std::cout << "Interface constructor started" << endl;

        // Main ORB SLAM3 object
        mSLAM_ = std::make_shared<ORB_SLAM3::System>(strVocFile_, strSettingsFile_, sensor_, bUseViewer_);

        // handle the conversion between ORB-SLAM3 data types and ROS2 message types
        typeConversions_ = std::make_shared<WrapperTypeConversions>();

        std::cout << "Interface constructor complete" << endl;
    }

    ORBSLAM3Interface::~ORBSLAM3Interface()
    {
        std::cout << "Interface destructor" << endl;
        mSLAM_->Shutdown();
        mSLAM_.reset();
        typeConversions_.reset();
        mapReferencePoses_.clear();
        allKFs_.clear();
    }

    // Creates a mapping between keyframe IDs and their corresponding keyframe objects -> allows for quick lookup of keyframes by their ID. Used in calculateReferencePoses().
    std::unordered_map<long unsigned int, ORB_SLAM3::KeyFrame *> ORBSLAM3Interface::makeKFIdPair(std::vector<ORB_SLAM3::Map *> mapsList)
    {
        std::unordered_map<long unsigned int, ORB_SLAM3::KeyFrame *> mpIdKFs;
        for (ORB_SLAM3::Map *pMap_i : mapsList)
        {
            std::vector<ORB_SLAM3::KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

            for (ORB_SLAM3::KeyFrame *pKF_j_Mi : vpKFs_Mi)
            {
                mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
            }
        }

        return mpIdKFs;
    }

    /*
        Computes the reference poses for all maps managed by ORB-SLAM3 (In ROS coord frame). 
        This involves transforming the poses of keyframes to align with the robot's coordinate system, accounting for any offsets. 
        The poses are stored in the global variable mapReferencePoses_ map, which associates each map with its reference pose.
            This is used in ...

        Is called in trackRGBD() and trackRGBDi() to update the current reference poses.
    */
    void ORBSLAM3Interface::calculateReferencePoses()
    {
        struct compareInitKFid
        {
            inline bool operator()(ORB_SLAM3::Map *elem1, ORB_SLAM3::Map *elem2)
            {
                return elem1->GetInitKFid() < elem2->GetInitKFid();
            }
        };

        mapReferencesMutex_.lock();
        mapReferencePoses_.clear();
        std::vector<ORB_SLAM3::Map *> mapsList = orbAtlas_->GetAllMaps();
        // sort the map array in init kf id order.
        std::sort(mapsList.begin(), mapsList.end(), compareInitKFid());
        allKFs_ = makeKFIdPair(mapsList);
        std::vector<ORB_SLAM3::Map *> mapsList2 = orbAtlas_->GetAllMaps();
        // std::cout << "Current map id: " << orbAtlas_->GetCurrentMap()->GetId() << std::endl;
        // for (auto mp : mapsList2)
        // {
        //     if (mp != nullptr && mp->GetOriginKF() != nullptr)
        //     {
        //         std::cout << "Map id: " << mp->GetId() << std::endl;
        //         std::cout << "Map init id: " << mp->GetInitKFid() << std::endl;
        //         std::cout << "Map origin kf id: " << mp->GetOriginKF()->mnId << std::endl;
        //         std::cout << "Map max kf id: " << mp->GetMaxKFid() << std::endl;
        //         std::cout << "================" << std::endl;
        //     }
        // }
        // std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
        for (size_t c = 0; c < mapsList.size(); c++)
        {
            // std::cout << "Map ID: " << mapsList[c]->GetId() << " origin kf ID: " << mapsList[c]->GetOriginKF()->mnId << " init kf id: " << mapsList[c]->GetInitKFid() << " max kf id: " << mapsList[c]->GetMaxKFid() << std::endl;
            if (c == 0)
            {
                auto poseWithoutOffset = typeConversions_->se3ToAffine(mapsList[c]->GetOriginKF()->GetPose());
                auto poseOffset = Eigen::Affine3d(
                    Eigen::Translation3d(robotX_, robotY_, 0.0) *
                    Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
                mapReferencePoses_[mapsList[c]] = poseOffset * poseWithoutOffset;
            }
            else
            {
                bool parentMapFound = false;
                int parentMapID = mapsList[c]->GetInitKFid() - 1;
                while (!parentMapFound)
                {
                    if (parentMapID < 0)
                    {
                        throw std::runtime_error("The init KF id - 1 is lesser than 0. This should not happen");
                    }
                    if (allKFs_.count(parentMapID) > 0)
                    {
                        parentMapFound = true;
                        break;
                    }
                    parentMapID -= 1;
                }
                if (mapReferencePoses_.count(allKFs_[parentMapID]->GetMap()) == 0)
                {
                    throw std::runtime_error("The parent map pose for this map ID does not exist. This should not happen.");
                }
                auto parentMapORBPose = allKFs_[parentMapID]->GetPose();
                mapReferencePoses_[mapsList[c]] = typeConversions_->transformPoseWithReference<Eigen::Affine3d>(mapReferencePoses_[allKFs_[parentMapID]->GetMap()], parentMapORBPose);
            }
        }
        // std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
        mapReferencesMutex_.unlock();
    }

    /*
        Retrieves the current map points (i.e. all points of the current map) tracked by ORB-SLAM3 and transforms them into the ROS2 coordinate frame. 
        It then converts these points into a ROS2 PointCloud2 message, which can be used for visualization or further processing in ROS2.
    */
    void ORBSLAM3Interface::getCurrentMapPoints(sensor_msgs::msg::PointCloud2 &mapPointCloud)
    {
        std::lock_guard<std::mutex> lock(currentMapPointsMutex_);
        // this flag serves to support
        std::vector<Eigen::Vector3f> trackedMapPoints;
        auto atlasAllKFs_ = orbAtlas_->GetAllKeyFrames();
        for (auto &KF : atlasAllKFs_)
        {
            for (auto &mapPoint : KF->GetMapPoints())
            {
                if (!mapPoint->isBad())
                {
                    auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                    mapReferencesMutex_.lock();

                    // check if KF (referenced by its unique ID KF->mnId) exists in the allKFs_. If KF is not present, count will return and we skip the rest of the code. 
                    if (allKFs_.count(KF->mnId) == 0)
                    {
                        mapReferencesMutex_.unlock();
                        continue;
                    }
                    auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[allKFs_[KF->mnId]->GetMap()], worldPos);
                    mapReferencesMutex_.unlock();
                    trackedMapPoints.push_back(mapPointWorld);
                }
            }
        }
        mapPointCloud = typeConversions_->MapPointsToPCL(trackedMapPoints);
    }

    /*
        Retrieves the points tracked by ORB-SLAM3 in the current key frame and transforms them into the ROS2 coordinate frame. 
        It then converts these points into a ROS2 PointCloud2 message, which can be used for visualization or further processing in ROS2.

        localmapping.cc has GetCurrKF() ...
    */
    void ORBSLAM3Interface::getCurrentKFPoints(sensor_msgs::msg::PointCloud2 &kFPointCloud)
    {
        std::lock_guard<std::mutex> lock(currentKFPointsMutex_);
        std::vector<Eigen::Vector3f> trackedKFPoints;
        auto atlasAllKFs_ = orbAtlas_->GetAllKeyFrames();


        // Check if there are any keyframes available
        if (atlasAllKFs_.empty())
        {
            std::cerr << "Publishing last keyframe: No keyframes available." << endl;
            return;
        }
        
        // Sort the keyframes by their unique ID (mnId) to get the most recent one
        auto latestKF = *std::max_element(atlasAllKFs_.begin(), atlasAllKFs_.end(),
            [](const ORB_SLAM3::KeyFrame* a, const ORB_SLAM3::KeyFrame* b)
            {
                return a->mnId < b->mnId;  // Compare based on mnId
            });

        // Iterate over the map points in the most recent keyframe
        for (auto &mapPoint : latestKF->GetMapPoints())
        {
            if (!mapPoint->isBad())
            {
                auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                mapReferencesMutex_.lock();

                // Check if the keyframe exists in the allKFs_ container
                if (allKFs_.count(latestKF->mnId) == 0)
                {
                    mapReferencesMutex_.unlock();
                    continue;
                }

                // Transform the map point based on the reference pose
                auto mapPointWorld = typeConversions_->transformPointWithReference<Eigen::Vector3f>(mapReferencePoses_[allKFs_[latestKF->mnId]->GetMap()], worldPos);
                mapReferencesMutex_.unlock();

                // Store the transformed map point
                trackedKFPoints.push_back(mapPointWorld);
            }
        }

        // Convert tracked map points to a PointCloud2 message
        kFPointCloud = typeConversions_->MapPointsToPCL(trackedKFPoints);
    }

    // Populates a MapData message with the current state of the SLAM system, including the pose graph and optionally the map points.
    void ORBSLAM3Interface::mapDataToMsg(slam_msgs::msg::MapData &mapDataMsg, bool currentMapKFOnly, bool includeMapPoints, std::vector<int> kFIDforMapPoints)
    {
        std::lock_guard<std::mutex> lock(mapDataMutex_);
        slam_msgs::msg::MapGraph poseGraph_;
        getOptimizedPoseGraph(poseGraph_, currentMapKFOnly);
        // publish the map data
        mapDataMsg.graph = poseGraph_;
        mapDataMsg.header.frame_id = globalFrame_;
        if (includeMapPoints)
        {
            for (auto kFId : kFIDforMapPoints)
            {
                slam_msgs::msg::KeyFrame pushedKf;
                pushedKf.id = kFId;
                if (allKFs_.find(kFId) != allKFs_.end())
                {
                    for (auto mapPoint : allKFs_[kFId]->GetMapPoints())
                    {
                        if (!mapPoint->isBad())
                        {
                            auto worldPos = typeConversions_->vector3fORBToROS(mapPoint->GetWorldPos());
                            mapReferencesMutex_.lock();
                            auto mapPointWorld = typeConversions_->transformPointWithReference<geometry_msgs::msg::Point>(mapReferencePoses_[allKFs_[kFId]->GetMap()], worldPos);
                            mapReferencesMutex_.unlock();
                            pushedKf.word_pts.push_back(mapPointWorld);
                        }
                    }
                    mapDataMsg.nodes.push_back(pushedKf);
                }
                else
                {
                    std::cerr << "Requested ID not available." << endl;
                }
            }
        }
        mapDataMutex_.unlock();
    }

    //  Updates the latest tracked pose by applying the reference poses calculated earlier. This ensures that the tracked pose is consistent with the global frame of reference.
    void ORBSLAM3Interface::correctTrackedPose(Sophus::SE3f &s)
    {
        mapReferencesMutex_.lock();
        latestTrackedPose_ = typeConversions_->transformPoseWithReference<Eigen::Affine3d>(
            mapReferencePoses_[orbAtlas_->GetCurrentMap()], s);
        mapReferencesMutex_.unlock();
    }

    // Compute the transforms from the map frame to the robot frame
    void ORBSLAM3Interface::getDirectMapToRobotTF(std_msgs::msg::Header headerToUse, geometry_msgs::msg::TransformStamped &tf)
    {
        tf.header.frame_id = globalFrame_;
        tf.child_frame_id = robotFrame_;
        if (hasTracked_)
        {
            // get transform between map and odom and send the transform.
            auto tfMapOdom = latestTrackedPose_;
            geometry_msgs::msg::Pose poseMapOdom = tf2::toMsg(tfMapOdom);
            rclcpp::Duration transformTimeout_ = rclcpp::Duration::from_seconds(0.5);
            rclcpp::Time odomTimestamp = headerToUse.stamp;
            tf.header.stamp = odomTimestamp + transformTimeout_;
            tf.header.frame_id = globalFrame_;
            tf.child_frame_id = robotFrame_;
            tf.transform.translation.x = poseMapOdom.position.x;
            tf.transform.translation.y = poseMapOdom.position.y;
            tf.transform.translation.z = poseMapOdom.position.z;
            tf.transform.rotation = poseMapOdom.orientation;
        }
    }

    // Compute the transforms from the map frame to the odom frame
    void ORBSLAM3Interface::getMapToOdomTF(const nav_msgs::msg::Odometry::SharedPtr msgOdom, geometry_msgs::msg::TransformStamped &tf)
    {
        // tf.header.stamp;
        tf.header.frame_id = globalFrame_;
        tf.child_frame_id = odomFrame_;
        if (hasTracked_)
        {
            // convert odom value to Eigen::Affine3d
            auto latestOdomTransform_ = Eigen::Affine3d(
                Eigen::Translation3d(msgOdom->pose.pose.position.x,
                                     msgOdom->pose.pose.position.y,
                                     msgOdom->pose.pose.position.z) *
                Eigen::Quaterniond(msgOdom->pose.pose.orientation.w,
                                   msgOdom->pose.pose.orientation.x,
                                   msgOdom->pose.pose.orientation.y,
                                   msgOdom->pose.pose.orientation.z));
            // get transform between map and odom and send the transform.
            auto tfMapOdom = latestTrackedPose_ * latestOdomTransform_.inverse();
            geometry_msgs::msg::Pose poseMapOdom = tf2::toMsg(tfMapOdom);
            rclcpp::Duration transformTimeout_ = rclcpp::Duration::from_seconds(0.5);
            rclcpp::Time odomTimestamp = msgOdom->header.stamp;
            tf.header.stamp = odomTimestamp + transformTimeout_;
            tf.header.frame_id = globalFrame_;
            tf.child_frame_id = odomFrame_;
            tf.transform.translation.x = poseMapOdom.position.x;
            tf.transform.translation.y = poseMapOdom.position.y;
            tf.transform.translation.z = poseMapOdom.position.z;
            tf.transform.rotation = poseMapOdom.orientation;
        }
    }

    // Retrieves the pose graph of the SLAM system, either for the entire map or just the current keyframes.
    // Used in mapDataToMsg().
    void ORBSLAM3Interface::getOptimizedPoseGraph(slam_msgs::msg::MapGraph &graph, bool currentMapKFOnly)
    {
        if (!currentMapKFOnly)
        {
            for (const auto &cKf : allKFs_)
            {
                ORB_SLAM3::KeyFrame *kf = cKf.second;
                Sophus::SE3f kfPose = kf->GetPose();
                geometry_msgs::msg::PoseStamped kfPoseStamped;
                mapReferencesMutex_.lock();
                kfPoseStamped.pose = typeConversions_->transformPoseWithReference<geometry_msgs::msg::Pose>(mapReferencePoses_[kf->GetMap()], kfPose);
                mapReferencesMutex_.unlock();
                kfPoseStamped.header.frame_id = globalFrame_;
                kfPoseStamped.header.stamp = typeConversions_->secToStamp(kf->mTimeStamp);
                graph.poses.push_back(kfPoseStamped);
                graph.poses_id.push_back(kf->mnId);
            }
        }
        else
        {
            vector<ORB_SLAM3::KeyFrame *> vKeyFrames = orbAtlas_->GetAllKeyFrames();
            // TODO: add isBad() check for keyframes. Evaluate mapping if you do this.
            // iterate over current keyframes.
            for (auto pKFcurr : vKeyFrames)
            {
                mapReferencesMutex_.lock();
                auto currReferencePose_ = mapReferencePoses_[pKFcurr->GetMap()];
                mapReferencesMutex_.unlock();
                Sophus::SE3f kFPose = pKFcurr->GetPose();
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.pose = typeConversions_->transformPoseWithReference<geometry_msgs::msg::Pose>(currReferencePose_, kFPose);
                poseStamped.header.frame_id = globalFrame_;
                poseStamped.header.stamp = typeConversions_->secToStamp(pKFcurr->mTimeStamp);
                // push to pose graph.
                graph.poses.push_back(poseStamped);
                graph.poses_id.push_back(pKFcurr->mnId);
            }
        }
    }

    // Store IMU data in buffer, which is used in trackRGBDi().
    void ORBSLAM3Interface::handleIMU(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        bufMutex_.lock();
        imuBuf_.push(msgIMU);
        bufMutex_.unlock();
    }

    /*
        Handle the tracking process for RGB-D images and IMU data. 
        Convert the incoming ROS2 images into OpenCV matrices and pass them to ORB-SLAM3 for tracking.

        Update the tracked pose based on ORB-SLAM3's output. 
        Check the tracking state returned by ORB-SLAM3 and handle cases where tracking fails (e.g., tracking lost, not initialized). 
        If tracking is successful, the reference poses are recalculated, and the tracked pose is corrected.
    */
    bool ORBSLAM3Interface::trackRGBDi(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD, Sophus::SE3f &Tcw)
    {
        orbAtlas_ = mSLAM_->GetAtlas();
        cv_bridge::CvImageConstPtr cvRGB;
        cv_bridge::CvImageConstPtr cvD;

        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cvRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception RGB!" << endl;
            return false;
        }

        // Copy the ros depth image message to cv::Mat.
        try
        {
            cvD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception D!" << endl;
            return false;
        }

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        bufMutex_.lock();
        if (!imuBuf_.empty())
        {
            // Load imu measurements from buffer
            vImuMeas.clear();
            while (!imuBuf_.empty() && typeConversions_->stampToSec(imuBuf_.front()->header.stamp) <= std::min(typeConversions_->stampToSec(msgRGB->header.stamp), typeConversions_->stampToSec(msgD->header.stamp)))
            {
                double t = typeConversions_->stampToSec(imuBuf_.front()->header.stamp);
                cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                imuBuf_.pop();
            }
        }
        bufMutex_.unlock();
        if (imuBuf_.size() > 0)
        {
            // Track the frame.
            Tcw = mSLAM_->TrackRGBD(cvRGB->image, cvD->image, typeConversions_->stampToSec(msgRGB->header.stamp), vImuMeas);
            auto currentTrackingState = mSLAM_->GetTrackingState();
            auto orbLoopClosing = mSLAM_->GetLoopClosing();
            if (orbLoopClosing->mergeDetected())
            {
                // do not publish any values during map merging. This is because the reference poses change.
                std::cout << "Waiting for merge to finish." << endl;
                return false;
            }
            if (currentTrackingState == 2)
            {
                calculateReferencePoses();
                correctTrackedPose(Tcw);
                hasTracked_ = true;
                return true;
            }
            else
            {
                switch (currentTrackingState)
                {
                case 0:
                    std::cerr << "ORB-SLAM failed: No images yet." << endl;
                    break;
                case 1:
                    std::cerr << "ORB-SLAM failed: Not initialized." << endl;
                    break;
                case 3:
                    std::cerr << "ORB-SLAM failed: Tracking LOST." << endl;
                    break;
                }
                return false;
            }
        }
        return false;
    }

    /*
        Handle the tracking process for RGB-D images only.
        Convert the incoming ROS2 images into OpenCV matrices and pass them to ORB-SLAM3 for tracking.
        Update the tracked pose based on ORB-SLAM3's output.
    */
    bool ORBSLAM3Interface::trackRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD, Sophus::SE3f &Tcw)
    {
        orbAtlas_ = mSLAM_->GetAtlas();
        cv_bridge::CvImageConstPtr cvRGB;
        cv_bridge::CvImageConstPtr cvD;
        // Copy the ros rgb image message to cv::Mat.
        try
        {
            cvRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception RGB!" << endl;
            return false;
        }

        // Copy the ros depth image message to cv::Mat.
        try
        {
            cvD = cv_bridge::toCvShare(msgD);
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception D!" << endl;
            return false;
        }
        // track the frame.
        Tcw = mSLAM_->TrackRGBD(cvRGB->image, cvD->image, typeConversions_->stampToSec(msgRGB->header.stamp));
        auto currentTrackingState = mSLAM_->GetTrackingState();
        auto orbLoopClosing = mSLAM_->GetLoopClosing();
        if (orbLoopClosing->mergeDetected())
        {
            // do not publish any values during map merging. This is because the reference poses change.
            std::cout << "Waiting for merge to finish." << endl;
            return false;
        }
        if (currentTrackingState == 2)
        {
            calculateReferencePoses();
            correctTrackedPose(Tcw);
            hasTracked_ = true;
            return true;
        }
        else
        {
            switch (currentTrackingState)
            {
            case 0:
                std::cerr << "ORB-SLAM failed: No images yet." << endl;
                break;
            case 1:
                std::cerr << "ORB-SLAM failed: Not initialized." << endl;
                break;
            case 3:
                std::cerr << "ORB-SLAM failed: Tracking LOST." << endl;
                break;
            }
            return false;
        }
    }
}