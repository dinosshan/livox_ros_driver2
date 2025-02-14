class LdsLidar {
public:
    // ... existing methods ...
    uint32_t GetLidarHandle() const { return lidar_handle_; }
private:
    uint32_t lidar_handle_;
    // ... existing members ...
}; 