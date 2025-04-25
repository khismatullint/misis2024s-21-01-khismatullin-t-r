#pragma once
#include "Dataset.h"
#include "Ransac.h" 
namespace RansacNamespace {
struct Metrics { 
    double iou=0, ale=0, fType=0; 
};
Metrics evalFrame(const TL& pred, const std::vector<bool>& type,
                  const FrameSample& gt, int imgH, int imgW);
}