#include "frontier_detection/map.hpp"

OctoMap::OctoMap(rclcpp::Logger logger) : mpLogger(logger) {
    RCLCPP_INFO(mpLogger, "Creating OctoMap");
    mpGlobalMap = std::make_shared<octomap::OcTree>(0.1);
}

void OctoMap::UpdateMap(octomap::Pointcloud& cloud, octomap::point3d& pos) {
    RCLCPP_DEBUG(mpLogger,
                 "Size of point cloud to be added: %d in an octomap of size %d",
                 cloud.size(), mpGlobalMap->size());
    RCLCPP_DEBUG(mpLogger,
                 "Point cloud reading from position- x: %f, y: %f, z: %f",
                 pos.x(), pos.y(), pos.z());
    std::unique_lock<std::mutex> lock(mpMtxPointCloudUpdate);
    // mpGlobalMap->insertPointCloud(cloud, pos);
    for (auto it = cloud.begin(); it != cloud.end(); ++it) {
        octomap::point3d endpoint = *it;

        // Compute the ray keys from the sensor origin (pos) to the endpoint
        octomap::KeyRay ray_keys;
        if (mpGlobalMap->computeRayKeys(pos, endpoint, ray_keys)) {
            // Mark nodes along the ray as free
            for (auto ray_it = ray_keys.begin(); ray_it != ray_keys.end();
                 ++ray_it) {
                mpGlobalMap->updateNode(*ray_it, false);  // Free space
            }
        }

        // Mark the endpoint node as occupied
        octomap::OcTreeKey key;
        if (mpGlobalMap->coordToKeyChecked(endpoint + pos, key)) {
            mpGlobalMap->updateNode(key, true);  // Occupied space
        }
    }
    RCLCPP_DEBUG(mpLogger, "Updated size of octomap: %d", mpGlobalMap->size());

    // Optionally prune the tree to optimize memory usage
    // RCLCPP_DEBUG(mpLogger, "Octree size for occupancy update: %d",
    // mpGlobalMap->size()); RCLCPP_DEBUG(mpLogger, "Number of leaf nodes in the
    // octree: %d", mpGlobalMap->calcNumNodes());
    if (ValidateOctree()) {
        RCLCPP_DEBUG(
            mpLogger,
            "Octree has some nan nodes which were removed, new size: %d",
            mpGlobalMap->size());
    }
    RCLCPP_DEBUG(mpLogger,
                 "Successfully validated and processed octomap, now updating "
                 "inner occupancy");
    // mpGlobalMap->updateInnerOccupancy();
    // mpGlobalMap->prune();
}

std::shared_ptr<octomap::OcTree> OctoMap::GetMap() {
    std::unique_lock<std::mutex> lock(mpMtxPointCloudUpdate);
    return std::shared_ptr<octomap::OcTree>(mpGlobalMap);
}

void OctoMap::GetMap(std::shared_ptr<octomap::OcTree> globalMap) {
    std::unique_lock<std::mutex> lock(mpMtxPointCloudUpdate);
    RCLCPP_DEBUG(mpLogger, "Handing over OcTree of size %d to ROS2 node",
                 mpGlobalMap->size());
    globalMap = mpGlobalMap;
}

bool OctoMap::ValidateOctree() {
    if (!mpGlobalMap) {
        throw std::invalid_argument("Octree is null.");
    }
    RCLCPP_DEBUG(mpLogger, "Validating octree, which is not null");

    // Vector of futures to store validation tasks
    std::vector<std::future<bool>> tasks;

    // Divide the workload into chunks
    size_t num_threads = std::thread::hardware_concurrency();
    size_t chunk_size = mpGlobalMap->size() / num_threads;
    size_t index = 0;

    // Mutex for thread-safe logging
    std::mutex log_mutex;

    // Launch tasks for parallel validation
    for (size_t i = 0; i < num_threads; ++i) {
        std::shared_ptr<octomap::OcTree> globalMap = mpGlobalMap;
        rclcpp::Logger logger = mpLogger;
        tasks.push_back(std::async(std::launch::async, [globalMap, logger,
                                                        index, chunk_size,
                                                        &log_mutex]() {
            auto it = globalMap->begin_leafs();
            std::advance(it, index);

            size_t processed = 0;
            for (; it != globalMap->end_leafs() && processed < chunk_size;
                 ++it, ++processed) {
                // Validate occupancy value
                if (std::isnan(it->getOccupancy())) {
                    // Log the invalid node details
                    std::lock_guard<std::mutex> lock(log_mutex);
                    RCLCPP_WARN(logger,
                                "Invalid node detected. Coordinates: [%f, %f, "
                                "%f], Occupancy: NaN",
                                it.getCoordinate().x(), it.getCoordinate().y(),
                                it.getCoordinate().z());

                    // Remove the invalid node or reset its value
                    globalMap->deleteNode(it.getKey());  // Remove the node
                }
            }
            return true;
        }));
        index += chunk_size;
    }

    // Wait for all tasks and aggregate results
    for (auto& task : tasks) {
        if (!task.get()) {
            return false;  // If any thread fails validation
        }
    }

    return true;  // All nodes are valid
}

OctoMapNode::OctoMapNode(std::string nodeName) : Node(nodeName) {
    RCLCPP_INFO(this->get_logger(), "Creating OctoMap Node");
    mpGlobalMap = std::make_shared<OctoMap>(this->get_logger());
    const int num_channels = 3;  // x y z
    mpRawPointCloud.header.frame_id = "map";
    mpRawPointCloud.height = 1;
    mpRawPointCloud.width = 0;
    mpRawPointCloud.is_bigendian = false;
    mpRawPointCloud.is_dense = true;
    mpRawPointCloud.point_step = num_channels * sizeof(float);
    mpRawPointCloud.row_step =
        mpRawPointCloud.point_step * mpRawPointCloud.width;
    mpRawPointCloud.fields.resize(num_channels);
    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        mpRawPointCloud.fields[i].name = channel_id[i];
        mpRawPointCloud.fields[i].offset = i * sizeof(float);
        mpRawPointCloud.fields[i].count = 1;
        mpRawPointCloud.fields[i].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
    }

    mpRawPointCloud.data.resize(mpRawPointCloud.row_step *
                                mpRawPointCloud.height);
    mpPointCloudSubscriber = this->create_subscription<MapMsg>(
        "/orbslam3_mono_node/map_points", 10,
        std::bind(&OctoMapNode::AddNewPointCloudCallback, this,
                  std::placeholders::_1));
    mpMapPublisher =
        this->create_publisher<octomap_msgs::msg::Octomap>("~/octomap", 10);
    mpPointCloudPublisher =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud",
                                                              10);
    mpMapPublisherTimer =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&OctoMapNode::PublishMap, this));
    mpThrMsgQueueProcess = new std::thread(&OctoMapNode::UpdateMap, this);
}

OctoMapNode::~OctoMapNode() {
    if (mpThrMsgQueueProcess->joinable()) {
        RCLCPP_INFO(this->get_logger(), "Joining mpMtxMsgQueue thread...");
        mpThrMsgQueueProcess->join();
    }
}

void OctoMapNode::UpdateMap() {
    MapMsg::SharedPtr msg;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mpMtxMsgQueue);
            if (mpMsgQueue.empty()) {
                continue;
            } else {
                msg = mpMsgQueue.front();
                mpMsgQueue.pop_front();
            }
        }
        octomap::Pointcloud cloud;
        octomap::point3d point;
        octomap::pointCloud2ToOctomap(msg->cloud, cloud);
        // ConvertPointCloud2ToOctomap(msg->cloud, cloud);
        CreatePoint3dFromTransform(msg->pose, point);
        mpGlobalMap->UpdateMap(cloud, point);
    }
}

void OctoMapNode::UpdateRawPointCloud(sensor_msgs::msg::PointCloud2& cloud) {
    // Verify that both point clouds have the same fields
    if (cloud.fields != mpRawPointCloud.fields) {
        RCLCPP_ERROR(this->get_logger(),
                     "PointCloud2 messages have different fields and cannot be "
                     "combined.");
        return;
    }

    // Copy header from the first cloud (modify as needed if frame alignment is
    // required)
    mpRawPointCloud.header = cloud.header;

    // Set the output point cloud properties
    mpRawPointCloud.height = 1;  // Unordered point cloud
    mpRawPointCloud.is_bigendian = cloud.is_bigendian;
    mpRawPointCloud.is_dense = cloud.is_dense;
    mpRawPointCloud.fields = cloud.fields;
    mpRawPointCloud.point_step = cloud.point_step;
    mpRawPointCloud.row_step = 0;  // Will be updated later

    // Combine point data
    mpRawPointCloud.data.reserve(cloud.data.size() +
                                 mpRawPointCloud.data.size());
    mpRawPointCloud.data.insert(mpRawPointCloud.data.end(), cloud.data.begin(),
                                cloud.data.end());

    // Update the total number of points
    mpRawPointCloud.width = (cloud.width * cloud.height) +
                            (mpRawPointCloud.width * mpRawPointCloud.height);
    mpRawPointCloud.row_step =
        mpRawPointCloud.width * mpRawPointCloud.point_step;
}

void OctoMapNode::AddNewPointCloudCallback(const MapMsg::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Adding point cloud to queue for processing");
    std::unique_lock<std::mutex> lock(mpMtxMsgQueue);
    mpMsgQueue.push_back(msg);
    UpdateRawPointCloud(msg->cloud);
    sensor_msgs::msg::PointCloud2 cloud = mpRawPointCloud;
    mpPointCloudPublisher->publish(cloud);
}

void OctoMapNode::ConvertPointCloud2ToOctomap(
    const sensor_msgs::msg::PointCloud2& cloud_msg,
    octomap::Pointcloud& octomap_cloud) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Converting sensor_msgs::msg::PointCloud2 to octomap::Pointcloud");
    if (sizeof(cloud_msg.data) == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Empty PointCloud2 received, skipping conversion.");
        return;
    }

    // Find offsets for x, y, z fields
    unsigned int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : cloud_msg.fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
        if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
        RCLCPP_WARN(this->get_logger(),
                    "PointCloud2 is missing required fields: x, y, or z.");
        return;
    }

    // Parse the point cloud data
    for (size_t point_idx = 0; point_idx < cloud_msg.data.size();
         point_idx += cloud_msg.point_step) {
        // Extract x, y, z values
        float x = *reinterpret_cast<const float*>(
            &cloud_msg.data[point_idx + x_offset]);
        float y = *reinterpret_cast<const float*>(
            &cloud_msg.data[point_idx + y_offset]);
        float z = *reinterpret_cast<const float*>(
            &cloud_msg.data[point_idx + z_offset]);
        RCLCPP_DEBUG(
            this->get_logger(),
            "Got map point coordinates as follows- x: %f, y: %f, z: %f", x, y,
            z);
        octomap_cloud.push_back(x, y, z);
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "Populated octomap::Pointcloud for further processing.");
}

void OctoMapNode::CreatePoint3dFromTransform(
    const geometry_msgs::msg::Transform& transform, octomap::point3d& point) {
    // Extract the translation components from the Transform message
    RCLCPP_DEBUG(
        this->get_logger(),
        "Converting geometry_msgs::msg::Transform into octomap::point3d");
    point = octomap::point3d(transform.translation.x, transform.translation.y,
                             transform.translation.z);
    RCLCPP_DEBUG(this->get_logger(),
                 "Current position of camera- x: %f, y: %f, z: %f", point.x(),
                 point.y(), point.z());
    RCLCPP_DEBUG(this->get_logger(),
                 "Successfully converted geometry_msgs::msg::Transform into "
                 "ocotmap::point3d");
}

void OctoMapNode::PublishMap() {
    if (mpGlobalMap) {
        RCLCPP_DEBUG(this->get_logger(), "Map exists, publishing map");
        octomap_msgs::msg::Octomap::UniquePtr msg =
            std::make_unique<octomap_msgs::msg::Octomap>();
        std::shared_ptr<octomap::OcTree> globalMap;
        globalMap = mpGlobalMap->GetMap();
        RCLCPP_DEBUG(this->get_logger(), "Got new map of size %d",
                     globalMap->size());
        if (globalMap && globalMap->size() > 0) {
            if (octomap_msgs::fullMapToMsg(*globalMap, *msg)) {
                msg->header.frame_id = "map";
                msg->header.stamp = this->get_clock()->now();
                mpMapPublisher->publish(std::move(msg));
                RCLCPP_DEBUG(this->get_logger(), "published octomap message");
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "failed to convert OctoMap to message");
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Got empty octomap to publish");
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Global Map object null, restart needed");
    }
}
