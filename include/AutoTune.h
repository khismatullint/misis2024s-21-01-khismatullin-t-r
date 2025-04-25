#pragma once
#include "Dataset.h"
#include "Metrics.h"
#include "Ransac.h"
namespace RansacNamespace { 
    settings autoTune(settings base, Dataset& ds, size_t iter=1000); 
}