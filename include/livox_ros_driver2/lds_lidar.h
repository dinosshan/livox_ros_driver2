class LdsLidar {
public:
    // ... existing methods ...
    static LdsLidar* GetInstance(double publish_freq);
    // Using existing methods instead of adding new ones
private:
    uint32_t lidar_handle_;
    // ... existing members ...
}; 