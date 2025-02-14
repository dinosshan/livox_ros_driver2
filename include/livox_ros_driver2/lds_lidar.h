class LdsLidar {
public:
    // ... existing methods ...
    static LdsLidar* GetInstance(double publish_freq);
    uint32_t GetCurrentLidarDevice() const { return lidar_devices_[0].handle; }  // Returns first LiDAR handle
    uint32_t GetLidarHandle() const { return lidar_handle_; }
private:
    uint32_t lidar_handle_;
    std::vector<DeviceItem> lidar_devices_;
    // ... existing members ...
}; 