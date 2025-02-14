class DriverNode {
public:
    // ... other members ...
    bool HandleLidarControl(
        livox_ros_driver2::LidarControlRequest_<std::allocator<void>>& req,
        livox_ros_driver2::LidarControlResponse_<std::allocator<void>>& res);
    
    // ... existing code ...
}; 