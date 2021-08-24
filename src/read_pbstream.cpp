/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-08-04 14:17:28
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-08-04 14:28:39
 * @Description: 
 */
 class PbStreamInterface
 {
     public:
     PbStreamInterface();
     ~PbStreamInterface();
 };

bool ModelDetector::readFromPbstream(std::string filename,
                                     std::vector<cv::Point2f>& point_cloud2,
                                     Rect2d& point_cloud2_range)
{
  vector<cartographer::mapping::TrajectoryNode::Data> node_datas;
  std::unique_ptr<cartographer::mapping::Submap3D> merge_submap;

  io::ProtoStreamReader stream(filename);
  io::ProtoStreamDeserializer deserializer(&stream);
  cartographer::mapping::proto::PoseGraph pose_graph_proto =
    deserializer.pose_graph();
  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const cartographer::mapping::proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory())
  {
    node_datas.resize(trajectory_proto.node().size());
    for (const cartographer::mapping::proto::Trajectory::Node& node_proto :
         trajectory_proto.node())
    {
      transform::Rigid3d global_pose = transform::ToRigid3(node_proto.pose());
      node_poses.Insert(
        NodeId{ trajectory_proto.trajectory_id(), node_proto.node_index() },
        global_pose);
    }
  }
  cartographer::mapping::proto::SerializedData proto_tmp;
  sensor::PointCloud all_point_cloud;

  while (deserializer.ReadNextSerializedData(&proto_tmp))
  {
    switch (proto_tmp.data_case())
    {
    case cartographer::mapping::proto::SerializedData::kNode:
    {
      cartographer::mapping::proto::Node* proto_node = proto_tmp.mutable_node();
      const cartographer::mapping::proto::TrajectoryNodeData proto_node_data =
        *proto_node->mutable_node_data();
      const cartographer::mapping::TrajectoryNode::Data node_data =
        cartographer::mapping::FromProto(proto_node_data);

      const sensor::PointCloud temp_point_cloud =
        node_data.high_resolution_point_cloud;

      NodeId id(proto_node->node_id().trajectory_id(),
                proto_node->node_id().node_index());
      const transform::Rigid3d gravity_alignment =
        transform::Rigid3d::Rotation(node_data.gravity_alignment);
      transform::Rigid3f node_global_pose =
        node_poses.at(id).cast<float>()
        * gravity_alignment.inverse().cast<float>();
      sensor::PointCloud point_cloud_in_map =
        TransformPointCloud(temp_point_cloud, node_global_pose);
      all_point_cloud.insert(all_point_cloud.end(), point_cloud_in_map.begin(),
                             point_cloud_in_map.end());
    }
    }
  }

  {
    sensor_msgs::PointCloud2Modifier modifier(origin_pointcloud2_);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                  1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(all_point_cloud.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(origin_pointcloud2_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(origin_pointcloud2_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(origin_pointcloud2_, "z");

    origin_pointcloud2_.header.stamp = ros::Time::now();
    origin_pointcloud2_.header.frame_id = "map";
    origin_pointcloud2_.height = 1;
    origin_pointcloud2_.width = all_point_cloud.size();

    for (auto& point : all_point_cloud)
    {
      *iter_x = point.position[0];
      *iter_y = point.position[1];
      *iter_z = 0.;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  float min_x = 9999999., min_y = 9999999, max_x = -9999999, max_y = -9999999;
  for (sensor::RangefinderPoint& point : all_point_cloud)
  {
    cv::Point2f p;
    p.x = point.position[0];
    p.y = point.position[1];
    point_cloud2.push_back(p);

    min_x = p.x < min_x ? p.x : min_x;
    min_y = p.y < min_y ? p.y : min_y;
    max_x = p.x > max_x ? p.x : max_x;
    max_y = p.y > max_y ? p.y : max_y;
  }
  point_cloud2_range.x = min_x;
  point_cloud2_range.y = min_y;
  point_cloud2_range.width = max_x - min_x;
  point_cloud2_range.height = max_y - min_y;
  return true;
}

