#include "../include/Metrics.h"
#include <set>

using namespace RansacNamespace;

static double lateralErr(const mrpt::math::TLine2D& ln,const std::vector<cv::Point>& pts)
{
    double acc=0; for(auto&p:pts) acc += std::abs(ln.distance({p.y,p.x}));
    return acc/pts.size();
}

Metrics RansacNamespace::evalFrame(const TL& pred,const std::vector<bool>& type,
                                   const FrameSample& gt,int H,int W)
{
    Metrics m; if(gt.lanes.empty()) return m;
    // --- IoU ---
    cv::Mat maskGT(H, W, CV_8UC3, cv::Scalar::all(0));
    cv::Mat maskPR = maskGT.clone();
    
    for(auto& l:gt.lanes)
        cv::polylines(maskGT, l.pts, false, cv::Scalar(255,255,255), 5);

    draw_lines(maskPR,
               const_cast<TL&>(pred),
               true,
               const_cast<std::vector<bool>&>(type));
    
    // переводим в серый
    cv::Mat gtGray, prGray;
    cv::cvtColor(maskGT, gtGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(maskPR, prGray, cv::COLOR_BGR2GRAY);
    
    int tp = cv::countNonZero(gtGray & prGray);
    int fp = cv::countNonZero(~gtGray & prGray);
    int fn = cv::countNonZero(gtGray & ~prGray);
    m.iou = tp ? double(tp)/(tp+fp+fn) : 0;

    // --- ALE --- (берём только ближайшую GT к центру)
    double accAle=0; size_t cnt=0;
    for(size_t i=0;i<std::min(pred.size(),gt.lanes.size());++i){
        if(!gt.lanes[i].pts.empty() && pred[i].coefs[2]!=0){
            accAle += lateralErr(pred[i],gt.lanes[i].pts); cnt++; }
    }
    m.ale = cnt? accAle/cnt : 999;

    // --- F‑type --- (solid / dashed)
    size_t tpType=0, pType=type.size(), tType=gt.lanes.size();
    for(size_t i=0;i<std::min(pType,tType);++i) if(gt.lanes[i].isSolid==type[i]) tpType++;
    double prec = pType? double(tpType)/pType : 0;
    double rec  = tType? double(tpType)/tType : 0;
    m.fType = (prec+rec)? 2*prec*rec/(prec+rec):0;
    return m;
}