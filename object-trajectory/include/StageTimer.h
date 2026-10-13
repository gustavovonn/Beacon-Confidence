#ifndef OT_STAGE_TIMER_H_
#define OT_STAGE_TIMER_H_

/**
 * Per-stage timing for the cost evaluator.
 *
 * Why percentiles and not a mean: the question is not "what does this cost on average", it is "can
 * this keep up". A mean of 40 us hides a tail of 5 ms, and the tail is what misses a 100 ms deadline.
 * The budget is set by the worst message the engine must still finish, so the p99 and the maximum
 * are the numbers that answer the advisor's question; the mean is reported only for context.
 *
 * Why steady_clock: it never jumps backwards and is not adjusted by NTP, unlike system_clock. On a
 * run long enough to be meaningful a clock adjustment would otherwise show up as a negative
 * duration or an impossible spike.
 *
 * Header only, no allocation inside a timed region, and the timer itself is never timed.
 */

#include <algorithm>
#include <chrono>
#include <deque>
#include <cstdio>
#include <string>
#include <vector>

namespace ot {

using Clock = std::chrono::steady_clock;

/// Microseconds elapsed, as a double so sub-microsecond stages do not quantise to 0.
inline double usSince(const Clock::time_point& t0)
{
    return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

/// One stage's samples and the statistics that matter for a deadline.
struct StageStats {
    std::string name;
    std::vector<double> samples;   ///< [us]

    void add(double us) { samples.push_back(us); }

    std::size_t count() const { return samples.size(); }

    double mean() const
    {
        if (samples.empty()) { return 0.0; }
        double s = 0.0;
        for (double v : samples) { s += v; }
        return s / static_cast<double>(samples.size());
    }

    double total() const
    {
        double s = 0.0;
        for (double v : samples) { s += v; }
        return s;
    }

    /// Nearest-rank percentile on a sorted copy. q in [0, 1].
    double percentile(double q) const
    {
        if (samples.empty()) { return 0.0; }
        std::vector<double> v = samples;
        std::sort(v.begin(), v.end());
        const double pos = q * static_cast<double>(v.size() - 1);
        const std::size_t i = static_cast<std::size_t>(pos + 0.5);
        return v[std::min(i, v.size() - 1)];
    }

    double max() const
    {
        if (samples.empty()) { return 0.0; }
        return *std::max_element(samples.begin(), samples.end());
    }
};

/// Scoped timer: starts on construction, records on destruction. One stage, one line of code.
class ScopedStage {
public:
    explicit ScopedStage(StageStats& s) : m_stats(s), m_t0(Clock::now()) {}
    ~ScopedStage() { m_stats.add(usSince(m_t0)); }
    ScopedStage(const ScopedStage&) = delete;
    ScopedStage& operator=(const ScopedStage&) = delete;

private:
    StageStats& m_stats;
    Clock::time_point m_t0;
};

class StageTimer;

/**
 * A scoped timer that is a no-op when the timer pointer is null. This is what lets the engine carry
 * stage instrumentation permanently: with no timer attached the whole cost is one predictable
 * branch, and with one attached the breakdown comes from the real code path rather than from a
 * hand-driven copy of it that can drift.
 */
class MaybeStage {
public:
    MaybeStage(StageTimer* timer, const char* name);
    ~MaybeStage();
    MaybeStage(const MaybeStage&) = delete;
    MaybeStage& operator=(const MaybeStage&) = delete;

private:
    StageStats* m_stats;
    Clock::time_point m_t0;
};

/// A set of named stages, printed as one table. The ordering is the pipeline's, not alphabetical.
class StageTimer {
public:
    /// Returns a reference the caller is expected to hold for the whole run. A std::deque, not a
    /// vector, precisely for that: registering a later stage must not invalidate the references
    /// already handed out, and a vector reallocates. (Found by AddressSanitizer, not by reasoning.)
    StageStats& stage(const std::string& name)
    {
        for (StageStats& s : m_stages) {
            if (s.name == name) { return s; }
        }
        m_stages.push_back(StageStats{name, {}});
        m_stages.back().samples.reserve(4096);
        return m_stages.back();
    }

    const std::deque<StageStats>& stages() const { return m_stages; }

    /// Human-readable table. `budgetUs` is the per-message deadline to judge against.
    void report(const char* title, double budgetUs) const
    {
        std::printf("\n%s\n", title);
        std::printf("%-22s %8s %10s %10s %10s %10s %9s\n", "stage", "n", "mean us", "p50 us",
                    "p99 us", "max us", "% of whole");
        std::printf("%s\n", std::string(84, '-').c_str());

        // The share is taken against the whole pipeline, not against the sum of every row: the
        // whole is itself one of the rows, so summing them all would double-count it and every
        // share would come out at half its real value.
        double whole100 = 0.0;
        for (const StageStats& s : m_stages) {
            if (s.name == "onMotionVectors") { whole100 = s.total(); }
        }
        if (whole100 <= 0.0) {
            for (const StageStats& s : m_stages) { whole100 += s.total(); }
        }

        for (const StageStats& s : m_stages) {
            std::printf("%-22s %8zu %10.2f %10.2f %10.2f %10.2f %8.1f%%\n", s.name.c_str(),
                        s.count(), s.mean(), s.percentile(0.50), s.percentile(0.99), s.max(),
                        whole100 > 0.0 ? 100.0 * s.total() / whole100 : 0.0);
        }

        // The verdict line. This is the sentence the advisor actually asked for.
        const StageStats* whole = nullptr;
        for (const StageStats& s : m_stages) {
            if (s.name == "onMotionVectors") { whole = &s; }
        }
        if (whole != nullptr && whole->count() > 0 && budgetUs > 0.0) {
            const double p99 = whole->percentile(0.99);
            std::printf("\n  budget %.0f us per message; whole pipeline p99 = %.2f us, max = %.2f us"
                        "\n  headroom at p99: %.1fx   -> %s\n",
                        budgetUs, p99, whole->max(), p99 > 0.0 ? budgetUs / p99 : 0.0,
                        whole->max() < budgetUs ? "fits, worst case included"
                                                : (p99 < budgetUs ? "fits at p99, tail exceeds"
                                                                  : "DOES NOT FIT"));
        }
    }

private:
    std::deque<StageStats> m_stages;
};

/// Defined after StageTimer because it needs the complete type. The stage lookup happens before
/// the clock starts, so the name search is not counted against the stage.
inline MaybeStage::MaybeStage(StageTimer* timer, const char* name)
    : m_stats(timer != nullptr ? &timer->stage(name) : nullptr), m_t0(Clock::now())
{
}

inline MaybeStage::~MaybeStage()
{
    if (m_stats != nullptr) { m_stats->add(usSince(m_t0)); }
}

}  // namespace ot

#endif  // OT_STAGE_TIMER_H_
