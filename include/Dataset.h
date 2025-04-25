#pragma once
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace RansacNamespace {
struct LaneGT { 
    std::vector<cv::Point> pts; 
    bool isSolid; 
};
struct FrameSample { 
    cv::Mat image; 
    std::vector<LaneGT> lanes; 
};
class Dataset {
public:
    explicit Dataset(const std::string& root, bool shuffle = true);
    bool next(FrameSample& out);        // пост‑инкремент, false – конец
    void reset();                       // начать сначала
    size_t size() const { return files_.size(); }
private:
    struct Rec { std::filesystem::path img, json; };
    std::vector<Rec> files_;
    size_t idx_ = 0;
};
}