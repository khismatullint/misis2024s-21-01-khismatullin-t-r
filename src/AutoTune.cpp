#include "../include/AutoTune.h"
#include <random>
#include <numeric>   // std::accumulate


namespace {

// — Весовая формула: чем больше score → тем лучше
//   0.5 * IoU + 0.3 * (1 – tanh(ALE / 50)) + 0.2 * F‑measure type
inline double score(const RansacNamespace::Metrics& m)
{
    return 0.5 * m.iou +
           0.3 * (1.0 - std::tanh(m.ale / 50.0)) +
           0.2 * m.fType;
}

// — Генерируем случайный вектор размера 6: [h1,s1,v1,h2,s2,v2]
std::vector<int> randomHSV(std::mt19937& rng)
{
    std::uniform_int_distribution<> dHue(0, 179);
    std::uniform_int_distribution<> dSat(0, 255);
    std::uniform_int_distribution<> dVal(0, 255);
    return { dHue(rng), dSat(rng), dVal(rng),
             dHue(rng), dSat(rng), dVal(rng) };
}

} // anonymous NS

using namespace RansacNamespace;

settings RansacNamespace::autoTune(settings base,
                                   Dataset&       ds,
                                   size_t         iter)
{
    std::mt19937 rng(42);
    std::uniform_int_distribution<> dThr (1, 10);   // Dist_threshold
    std::uniform_int_distribution<> dInl (10, 40);  // min_inliers
    std::uniform_int_distribution<> dNorm(3, 15);   // sense_to_normolize_data

    // Буферизуем выборку, чтобы много раз не гонять disk I/O
    std::vector<FrameSample> frames;  frames.reserve(ds.size());
    FrameSample smp;
    while (ds.next(smp)) frames.push_back(std::move(smp));
    ds.reset();

    settings bestS = base;
    double   best  = -1.0;

    for (size_t it = 0; it < iter; ++it)
    {
        settings cur = base;
        cur.parametersHSV             = randomHSV(rng);
        cur.Dist_threshold            = dThr (rng);
        cur.min_inliers               = dInl (rng);
        cur.sense_to_normolize_data   = dNorm(rng);

        // --- аккумулируем метрики по всем кадрам ---
        Metrics acc;  // всё =0 по умолч.
        for (const auto& f : frames)
        {
            // Заглушка: пустое предсказание (ни одной линии) и «сплошные» типы
            //    → draw_lines внутри evalFrame ничего не рисует и не крашится
            TL emptyPred;
            std::vector<bool> fakeType(cur.cout_stripes, true);

            Metrics m = evalFrame(emptyPred, fakeType,
                                   f, f.image.rows, f.image.cols);
            acc.iou   += m.iou;
            acc.ale   += m.ale;
            acc.fType += m.fType;
        }
        const double N = static_cast<double>(frames.size());
        acc.iou   /= N;  acc.ale   /= N;  acc.fType /= N;

        double sc = score(acc);
        if (sc > best) { best = sc; bestS = cur; }
    }
    return bestS;
}
