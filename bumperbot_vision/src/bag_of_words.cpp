#include <filesystem>

#include "bumperbot_vision/bag_of_words.hpp"

#include "cv_bridge/cv_bridge.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
BagOfWords::BagOfWords(const std::string & name)
: Node(name), is_training_(true), is_trained_(false)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), image_sub_, info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&BagOfWords::imageCallback, this, _1, _2));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  training_frequency_ = declare_parameter("training_frequency", 10.0);
  last_train_time_ = now();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  image_sub_.subscribe(
    this, "left_camera/image_raw", hints.getTransport(), sensor_data_qos);
  info_sub_.subscribe(this, "left_camera/camera_info", sensor_data_qos);

  detector_ = cv::ORB::create();

  train_mode_srv_ = create_service<std_srvs::srv::Trigger>(
    name + "/train_mode", [&](const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      (void)request;
      is_training_ = true;
      response->success = true;
      response->message = "Switched to train mode";
    });

  test_mode_srv_ = create_service<std_srvs::srv::Trigger>(
    name + "/test_mode", [&](const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      (void)request;
      is_training_ = false;
      response->success = true;
      response->message = "Switched to test mode";
    });

  save_srv_ = create_service<bumperbot_msgs::srv::SetString>(
    name + "/save", [&](const bumperbot_msgs::srv::SetString::Request::SharedPtr request,
      bumperbot_msgs::srv::SetString::Response::SharedPtr response) {
      std::string path = request->data;
      if (path.empty() || !std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
        response->success = false;
        response->message = "Path cannot be empty or invalid";
        return;
      }

      if (!is_trained_) {
        if(!buildVocabulary()){
          response->success = false;
          response->message = "Failed to build vocabulary. Cannot save database.";
          RCLCPP_ERROR(get_logger(), response->message.c_str());
          return;
        }
      }
      voc_.save(path + "/bow_voc.bin");
      
      // Save the database in binary format
      std::ofstream feat_file(path + "/bow_features.bin", std::ios::binary);
      if (!feat_file.is_open()) throw std::runtime_error("Could not open bow_features.bin");

      size_t num_images = training_features_.size();
      feat_file.write(reinterpret_cast<const char*>(&num_images), sizeof(num_images));

      for(const auto& mat : training_features_) {
        int rows = mat.rows;
        int cols = mat.cols;
        int type = mat.type();
        feat_file.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
        feat_file.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
        feat_file.write(reinterpret_cast<const char*>(&type), sizeof(type));
          
        if (mat.isContinuous()) {
          feat_file.write(reinterpret_cast<const char*>(mat.ptr()), mat.total() * mat.elemSize());
        } else {
          cv::Mat cont = mat.clone();
          feat_file.write(reinterpret_cast<const char*>(cont.ptr()), cont.total() * cont.elemSize());
        }
      }
      feat_file.close();

      // Save the corresponding poses
      std::ofstream pose_file(path + "/bow_poses.txt");
      for (const auto& pose : stored_poses_) {
        pose_file << pose.transform.translation.x << " "
                  << pose.transform.translation.y << " "
                  << pose.transform.translation.z << " "
                  << pose.transform.rotation.x << " "
                  << pose.transform.rotation.y << " "
                  << pose.transform.rotation.z << " "
                  << pose.transform.rotation.w << "\n";
      }
      pose_file.close();

      response->success = true;
      response->message = "Database saved to " + path;
    });

  load_srv_ = create_service<bumperbot_msgs::srv::SetString>(
    name + "/load", [&](const bumperbot_msgs::srv::SetString::Request::SharedPtr request,
      bumperbot_msgs::srv::SetString::Response::SharedPtr response) {
      std::string path = request->data;
      if (path.empty() || !std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
        response->success = false;
        response->message = "Path cannot be empty or invalid";
        return;
      }
      try {
        voc_.load(path + "/bow_voc.bin");
        
        // Load the binary features 
        std::ifstream feat_file(path + "/bow_features.bin", std::ios::binary);
        size_t num_images;
        feat_file.read(reinterpret_cast<char*>(&num_images), sizeof(num_images));
        training_features_.clear();
        
        for(size_t i = 0; i < num_images; ++i) {
          int rows, cols, type;
          feat_file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
          feat_file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
          feat_file.read(reinterpret_cast<char*>(&type), sizeof(type));
          
          cv::Mat mat(rows, cols, type);
          feat_file.read(reinterpret_cast<char*>(mat.data), mat.total() * mat.elemSize());
          training_features_.push_back(mat);
        }

        feat_file.close();
        db_.setVocabulary(voc_, false, 0);
        for(const auto& descriptors : training_features_) {
          db_.add(descriptors);
        }

        std::ifstream pose_file(path + "/bow_poses.txt");
        if (!pose_file.is_open()) {
          throw std::runtime_error("Poses file not found!");
        }

        // Restore poses
        stored_poses_.clear();
        double tx, ty, tz, rx, ry, rz, rw;
        while (pose_file >> tx >> ty >> tz >> rx >> ry >> rz >> rw) {
          geometry_msgs::msg::TransformStamped pose;
          pose.header.frame_id = "map";
          pose.child_frame_id = "base_footprint";
          pose.transform.translation.x = tx;
          pose.transform.translation.y = ty;
          pose.transform.translation.z = tz;
          pose.transform.rotation.x = rx;
          pose.transform.rotation.y = ry;
          pose.transform.rotation.z = rz;
          pose.transform.rotation.w = rw;
          stored_poses_.push_back(pose);
        }
        pose_file.close();

        is_trained_ = true;
        is_training_ = false;
        response->success = true;
        response->message = "Database loaded from " + path;
      } catch (const std::exception & e) {
        response->success = false;
        response->message = "Failed to load database: " + std::string(e.what());
        RCLCPP_ERROR(get_logger(), response->message.c_str());
      }
    });
}

void BagOfWords::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Detect and compute ORB features
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detector_->detectAndCompute(img, cv::noArray(), keypoints, descriptors);

  if (descriptors.empty()) {
    return;
  }

  if (is_training_) {
    // Control training frequency
    if ((now() - last_train_time_).seconds() < (1.0 / training_frequency_)) {
      return;
    }
    last_train_time_ = now();

    // Get the pose of the robot at the time of the image
    geometry_msgs::msg::TransformStamped current_pose;
    try {
      current_pose = tf_buffer_->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                           "Could not transform: %s", ex.what());
      return; 
    }

    // Store features and pose for database creation
    training_features_.push_back(descriptors);
    stored_poses_.push_back(current_pose);
    RCLCPP_INFO(get_logger(), "Added image to training set. Total images: %zu", training_features_.size());
  } else {
    if (!is_trained_) {
      if (!buildVocabulary()){
        RCLCPP_WARN(get_logger(), "Not trained yet! Cannot perform localization.");
        return;
      }
    }

    // Query the database to find the most similar image
    DBoW3::QueryResults results;
    db_.query(descriptors, results, 1);

    if (!results.empty()) {
      auto best_match = results[0];
      int best_id = best_match.Id;

      if (best_id >= 0 && best_id < static_cast<int>(stored_poses_.size())) {
        auto pose = stored_poses_[best_id];
        pose.child_frame_id = "base_footprint_bow";
        pose.header.stamp = now();
        transform_broadcaster_->sendTransform(pose);
        
        RCLCPP_INFO(get_logger(), 
          "Relocalized! Match Score: %.3f | Pose [x: %.2f, y: %.2f]",
          best_match.Score, 
          pose.transform.translation.x, 
          pose.transform.translation.y);
      }
    }
  }
}

bool BagOfWords::buildVocabulary()
{
  if (training_features_.empty()) {
    RCLCPP_WARN(get_logger(), "No features gathered! Cannot build vocabulary.");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Clustering visual words... This may take a moment.");
  voc_.create(training_features_);
  db_.setVocabulary(voc_, false, 0);
  for (const auto& descriptors : training_features_) {
    db_.add(descriptors);
  }
  is_trained_ = true;
  return true;
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::BagOfWords>("bag_of_words"));
  rclcpp::shutdown();
  return 0;
}