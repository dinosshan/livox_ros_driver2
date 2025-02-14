class LdsLidar {
public:
    // ... existing methods ...
    static LdsLidar* GetInstance(double publish_freq);
    uint32_t GetLidarHandle() const { return lidar_handle_; }
private:
    uint32_t lidar_handle_;
    // ... existing members ...
}; 