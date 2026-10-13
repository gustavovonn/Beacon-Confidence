/**
 * CostEvaluator -- standalone cost measurement for the confidence pipeline.
 *
 * This answers one question: can a vehicle-class processor run this pipeline inside the interval
 * between two V2X messages? It is deliberately free of OMNeT++, Artery, Vanetza and the dataset:
 * it generates its own synthetic load, so it builds and runs on the target board with nothing but
 * a C++17 compiler. That is the point -- the VisionFive 2 runs a bare Linux with no simulator on it.
 *
 *   make cost          (the Makefile knows the include paths and the vendored sources)
 *
 * The whole pipeline and its per-stage breakdown come from ONE instrumented run of the real
 * onMotionVectors(), via the engine's optional StageTimer. An earlier version drove the six stages
 * by hand to get the breakdown; that version silently measured the wrong thing, because hand-driving
 * skips the track bookkeeping the engine does, so every track aged out and the stages were timed
 * against an empty store. Measuring the real path is the only way the breakdown and the verdict
 * are about the same system.
 *
 * Synthetic load is parameterised by the three terms that actually set the cost:
 *   N  trajectories currently believed      -> the predict stage is O(N)
 *   m  motion vectors in one message        -> gating is O(N*m)
 *   k  candidates inside the gate per vector-> the assignment is worst-case cubic in min(N, m)
 * The sweep shows how the cost moves with N and m so the growth, not just one point, is visible.
 */

#include "../include/ObjectConfidenceEngine.h"
#include "../include/StageTimer.h"
#include "../include/Hungarian.h"   // Hungarian::kNone: the evaluator reads the assignment directly
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <random>
#include <fstream>
#include <string>

using namespace ot;

namespace {

/// Default budget: a 10 Hz CAM/CPM cadence gives 100 ms between messages. The pipeline must leave
/// room for everything else the stack does, so the headroom factor, not the bare fit, is the number.
constexpr double kDefaultBudgetUs = 100000.0;

struct Options {
    int tracks = 50;        ///< N to establish before measuring
    int vectors = 10;       ///< m per message
    int messages = 2000;    ///< measured messages
    int warmup = 200;
    double budgetUs = kDefaultBudgetUs;
    double jitter = 0.4;    ///< [m] report noise; sets how many candidates fall inside the gate
    bool sweep = false;
    bool coarseGate = true;   ///< the pre-filter is an ablation here, not a setting
    unsigned seed = 20261009;
};

/**
 * A synthetic road: `n` objects spread over lanes, all moving. Lane spacing is wide enough that the
 * objects are genuinely distinct but close enough that gating has real work to do -- if every object
 * were kilometres apart, k would be 1 and the assignment would never be exercised.
 */
class SyntheticWorld {
public:
    SyntheticWorld(int n, unsigned seed) : m_rng(seed)
    {
        m_x.reserve(n);
        m_y.reserve(n);
        m_v.reserve(n);
        std::uniform_real_distribution<double> speed(12.0, 28.0);
        std::uniform_real_distribution<double> along(0.0, 400.0);
        for (int i = 0; i < n; ++i) {
            m_x.push_back(3.5 * (i % 6));          // six lanes
            m_y.push_back(along(m_rng) + 30.0 * (i / 6));
            m_v.push_back(speed(m_rng));
        }
    }

    void advance(double dt)
    {
        for (std::size_t i = 0; i < m_x.size(); ++i) { m_y[i] += m_v[i] * dt; }
    }

    /// One message: `m` objects reported, with noise. Round-robin so every object is fed and the
    /// track store neither starves nor grows without bound.
    std::vector<MotionVector> message(double t, int m, double jitter)
    {
        std::normal_distribution<double> noise(0.0, jitter);
        std::vector<MotionVector> out;
        out.reserve(static_cast<std::size_t>(m));
        for (int j = 0; j < m; ++j) {
            const std::size_t i = m_cursor % m_x.size();
            ++m_cursor;
            MotionVector v;
            v.t = t;
            v.x = m_x[i] + noise(m_rng);
            v.y = m_y[i] + noise(m_rng);
            v.speed = m_v[i] + 0.1 * noise(m_rng);
            v.heading = 0.0 + 0.01 * noise(m_rng);
            v.objectClass = 1;
            v.origin = Origin::CpmObject;
            v.author = 1000 + (i % 8);           // eight distinct senders
            v.trueStation = static_cast<int64_t>(i);
            out.push_back(v);
        }
        return out;
    }

    std::size_t size() const { return m_x.size(); }

private:
    std::mt19937 m_rng;
    std::vector<double> m_x, m_y, m_v;
    std::size_t m_cursor = 0;
};

struct Result {
    std::size_t tracks = 0;
    double meanUs = 0.0;
    double p99Us = 0.0;
    double maxUs = 0.0;
};

/// Measure the whole pipeline, as the engine really runs it, with per-stage instrumentation on.
Result measureWhole(const Options& o, StageTimer& timer)
{
    EngineParams params;
    params.coarseGate = o.coarseGate;
    ObjectConfidenceEngine engine(params);
    SyntheticWorld world(o.tracks, o.seed);
    const double dt = 0.1;
    double t = 0.0;

    for (int i = 0; i < o.warmup; ++i) {
        world.advance(dt);
        t += dt;
        engine.onMotionVectors(world.message(t, o.vectors, o.jitter), 1, t);
    }

    // instrumentation on only now, so the warmup messages are not in the statistics
    engine.setStageTimer(&timer);
    for (int i = 0; i < o.messages; ++i) {
        world.advance(dt);
        t += dt;
        // build the message OUTSIDE the timed region: generating synthetic input is not a cost the
        // real system pays, and allocating inside the region would be measuring the generator
        const std::vector<MotionVector> msg = world.message(t, o.vectors, o.jitter);
        engine.onMotionVectors(msg, 1, t);
    }
    engine.setStageTimer(nullptr);

    const StageStats& whole = timer.stage("onMotionVectors");
    Result r;
    r.tracks = engine.tracks().size();
    r.meanUs = whole.mean();
    r.p99Us = whole.percentile(0.99);
    r.maxUs = whole.max();
    return r;
}

void runOnce(const Options& o)
{
    StageTimer timer;
    const Result r = measureWhole(o, timer);
    std::printf("\nN = %zu trajectories believed, m = %d vectors/message, %d messages measured\n",
                r.tracks, o.vectors, o.messages);
    timer.report("one instrumented run: onMotionVectors is the verdict, the rest is the breakdown",
                 o.budgetUs);
}

/// The sweep. One point proves nothing about growth; the shape of the curve is the real answer.
void runSweep(const Options& base)
{
    const int trackCounts[] = {10, 25, 50, 100, 200, 400};
    const int vectorCounts[] = {1, 5, 10, 20};

    std::printf("\nSWEEP -- whole-pipeline p99 in microseconds, budget %.0f us\n", base.budgetUs);
    std::printf("%8s", "N \\ m");
    for (int m : vectorCounts) { std::printf("%12d", m); }
    std::printf("\n%s\n", std::string(8 + 12 * 4, '-').c_str());

    for (int n : trackCounts) {
        std::printf("%8d", n);
        for (int m : vectorCounts) {
            Options o = base;
            o.tracks = n;
            o.vectors = m;
            o.messages = 500;
            o.warmup = 100;
            StageTimer timer;
            const Result r = measureWhole(o, timer);
            std::printf("%12.1f", r.p99Us);
        }
        std::printf("\n");
    }
    std::printf("\nRead the columns for the cost of a fuller message and the rows for the cost of a\n"
                "busier neighbourhood. A row that grows linearly is the predict stage; a column that\n"
                "grows faster than linearly is the assignment.\n");
}

/**
 * Identify the machine. Without this a result is meaningless: the same table printed from a desktop
 * and from the VisionFive 2 looks identical and means something completely different. Read from
 * /proc so it works on both x86-64 and the board's RISC-V without any extra dependency.
 */
void printHost()
{
    std::printf("host: ");
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line, model, isa;
    int cores = 0;
    while (std::getline(cpuinfo, line)) {
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) { continue; }
        const std::string key = line.substr(0, colon);
        const std::string val = line.substr(colon + 1);
        const std::size_t first = val.find_first_not_of(" \t");
        const std::string v = (first == std::string::npos) ? "" : val.substr(first);
        if (key.rfind("model name", 0) == 0 && model.empty()) { model = v; }      // x86, arm
        if (key.rfind("uarch", 0) == 0 && model.empty()) { model = v; }           // riscv
        if (key.rfind("isa", 0) == 0 && isa.empty()) { isa = v; }                 // riscv
        if (key.rfind("processor", 0) == 0) { ++cores; }
    }
    std::printf("%s", model.empty() ? "unknown cpu" : model.c_str());
    if (!isa.empty()) { std::printf(" [%s]", isa.c_str()); }
    std::printf(", %d logical core%s\n", cores, cores == 1 ? "" : "s");
}

int toInt(const char* s, int fallback)
{
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    return (end != nullptr && *end == '\0') ? static_cast<int>(v) : fallback;
}

}  // namespace

int main(int argc, char** argv)
{
    Options o;
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool hasNext = (i + 1 < argc);
        if (std::strcmp(a, "--sweep") == 0) { o.sweep = true; }
        else if (std::strcmp(a, "--no-coarse-gate") == 0) { o.coarseGate = false; }
        else if (std::strcmp(a, "--tracks") == 0 && hasNext) { o.tracks = toInt(argv[++i], o.tracks); }
        else if (std::strcmp(a, "--vectors") == 0 && hasNext) { o.vectors = toInt(argv[++i], o.vectors); }
        else if (std::strcmp(a, "--messages") == 0 && hasNext) { o.messages = toInt(argv[++i], o.messages); }
        else if (std::strcmp(a, "--budget-us") == 0 && hasNext) { o.budgetUs = std::atof(argv[++i]); }
        else if (std::strcmp(a, "--seed") == 0 && hasNext) { o.seed = static_cast<unsigned>(toInt(argv[++i], 1)); }
        else if (std::strcmp(a, "--help") == 0) {
            std::printf("usage: cost_evaluator [--sweep] [--tracks N] [--vectors m] "
                        "[--messages M] [--budget-us US] [--seed S]\n");
            return 0;
        } else {
            std::printf("unknown argument: %s (try --help)\n", a);
            return 2;
        }
    }

    std::printf("CostEvaluator -- confidence pipeline, standalone. C++%ld, %zu-bit double.\n",
                __cplusplus / 100, sizeof(double) * 8);
    printHost();
    if (o.sweep) { runSweep(o); } else { runOnce(o); }
    return 0;
}
