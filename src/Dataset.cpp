#include "../include/Dataset.h"
#include <random>
#include <fstream>
using json = nlohmann::json;

RansacNamespace::Dataset::Dataset(const std::string& root, bool shuffle)
{
    namespace fs = std::filesystem;
    for (auto& p : fs::directory_iterator(root + "/frames"))
        files_.push_back({p.path(), root + "/labels/" + p.path().stem().string() + ".json"});
    if(shuffle){ std::mt19937 rng(42); std::shuffle(files_.begin(), files_.end(), rng);}    
}

bool RansacNamespace::Dataset::next(FrameSample& out)
{
    if(idx_ >= files_.size()) return false;
    const auto& rec = files_[idx_++];
    out.image = cv::imread(rec.img.string(), cv::IMREAD_COLOR);
    if (out.image.empty()) {
        std::cerr << "bad image: " << rec.img << '\n';
        return next(out);               // пропускаем кадр
    }
    std::ifstream ifs(rec.json);
    json j; ifs >> j;
    out.lanes.clear();
    for(auto& jl : j["lanes"]){
        LaneGT gt; gt.isSolid = jl["isSolid"].get<bool>();
        for(auto& p : jl["points"]) gt.pts.emplace_back(p[0], p[1]);
        out.lanes.push_back(gt);
    }
    return true;
}

void RansacNamespace::Dataset::reset(){ idx_ = 0; }