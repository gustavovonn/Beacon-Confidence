/**
 * experiment_runner -- replays a dataset through the engine and scores it against the oracle.
 *
 * One engine per receiver, because that is what a deployment has: a vehicle knows only what reached
 * it. Pooling every reception into one engine would hand it a god's-eye view and make every
 * association number optimistic.
 *
 * The oracle (`true_station`) is read HERE and never passed to the engine. It is the only route to
 * association ground truth: without it, a track that quietly swapped identity looks exactly like one
 * that did not.
 *
 *   ./build/experiment_runner --dataset dataset/groundtruth --out results/run.csv
 */

#include "../include/DatasetParser.h"
#include "../include/ObjectConfidenceEngine.h"
#include "../include/Hungarian.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <string>

using namespace ot;

namespace {

struct Options {
    std::string dataset = "dataset/groundtruth";
    std::string out = "results/run.csv";
    std::string label;          ///< echoed into the summary so a batch run is identifiable
    EngineParams params;        ///< every knob a batch might sweep
    bool noCsv = false;         ///< summary only; a sweep does not need 300 k rows per point
};

/// What the oracle lets us say about one track, which the engine itself cannot know.
struct TrackTruth {
    std::map<int64_t, int> stationHits;   ///< how often each true station fed it
    int64_t dominant = -1;                ///< the station it is mostly about
    int switches = 0;                     ///< times the dominant station changed
    int hits = 0;
};

struct Totals {
    long vectors = 0;
    long dropped = 0;        ///< failed plausibility
    long started = 0;        ///< began a new trajectory
    long fused = 0;
    long rejected = 0;

    long fusedWithOracle = 0;    ///< fused and the oracle knows what it really was
    long correct = 0;            ///< fused into a track that is about the same real station
    long wrong = 0;              ///< fused into a track about a DIFFERENT real station
    long idSwitches = 0;

    long gatedSum = 0;           ///< for the mean candidate count k
    long contested = 0;          ///< more than one candidate inside the gate
    double confidenceCorrect = 0.0;
    double confidenceWrong = 0.0;

    /// Fragmentation: how many separate trajectories one real object was split across, at one
    /// receiver. The complement of an ID switch -- a swap merges two objects into one track,
    /// fragmentation splits one object across several -- and a pipeline can look good on one while
    /// failing badly on the other, so both are reported.
    long trueObjectsTracked = 0;   ///< (receiver, true station) pairs that got at least one track
    long tracksForThose = 0;       ///< distinct tracks those pairs were spread over
    long fragmented = 0;           ///< pairs that needed more than one track
};

const char* outcomeName(Outcome o)
{
    switch (o) {
        case Outcome::Fused: return "fused";
        case Outcome::Rejected: return "rejected";
        case Outcome::StartedTrack: return "started";
        case Outcome::Dropped: return "dropped";
    }
    return "?";
}

}  // namespace

int main(int argc, char** argv)
{
    Options o;
    for (int i = 1; i < argc; ++i) {
        const bool hasNext = (i + 1 < argc);
        if (std::strcmp(argv[i], "--dataset") == 0 && hasNext) { o.dataset = argv[++i]; }
        else if (std::strcmp(argv[i], "--out") == 0 && hasNext) { o.out = argv[++i]; }
        else if (std::strcmp(argv[i], "--label") == 0 && hasNext) { o.label = argv[++i]; }
        else if (std::strcmp(argv[i], "--no-csv") == 0) { o.noCsv = true; }
        else if (std::strcmp(argv[i], "--gate") == 0 && hasNext) {
            o.params.gate = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--pos-var") == 0 && hasNext) {
            // R for px and py together: the knob that decides how wide the gate really is, since
            // the gate is a threshold on the gap divided by this
            const double v = std::atof(argv[++i]);
            o.params.noise.px = v;
            o.params.noise.py = v;
        } else if (std::strcmp(argv[i], "--speed-var") == 0 && hasNext) {
            o.params.noise.speed = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--heading-var") == 0 && hasNext) {
            o.params.noise.heading = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--retire-after") == 0 && hasNext) {
            o.params.retireAfter = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--confirm-after") == 0 && hasNext) {
            o.params.confirmAfter = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--initial-pos-var") == 0 && hasNext) {
            o.params.initialPosVar = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--no-coarse-gate") == 0) {
            o.params.coarseGate = false;
        } else if (std::strcmp(argv[i], "--ignore-class") == 0) {
            o.params.classMustMatch = false;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("usage: experiment_runner [--dataset DIR] [--out CSV] [--label NAME]\n"
                        "                        [--gate X] [--pos-var X] [--speed-var X]\n"
                        "                        [--heading-var X] [--retire-after S]\n"
                        "                        [--confirm-after N] [--initial-pos-var X]\n"
                        "                        [--ignore-class] [--no-coarse-gate] [--no-csv]\n");
            return 0;
        } else {
            std::printf("unknown argument: %s (try --help)\n", argv[i]);
            return 2;
        }
    }

    DatasetParser parser;
    std::string error;
    if (!parser.load(o.dataset, error)) {
        std::printf("ERROR: %s\n", error.c_str());
        return 1;
    }
    const DatasetInfo& info = parser.info();
    std::printf("dataset %s\n", info.path.c_str());
    std::printf("  origin        %.9f, %.9f\n", info.originLat, info.originLon);
    std::printf("  cams          %zu\n", info.cams);
    std::printf("  cpms          %zu carrying %zu objects (%zu with oracle)\n", info.cpms,
                info.objects, info.objectsWithOracle);
    std::printf("  receptions    %zu -> %zu deliveries with content\n", info.receptions,
                info.deliveries);
    std::printf("  time          %.2f s to %.2f s\n", info.firstTime, info.lastTime);

    std::ofstream out;
    if (!o.noCsv) {
        out.open(o.out);
        if (!out) {
            std::printf("ERROR: cannot write %s\n", o.out.c_str());
            return 1;
        }
        out << "t_rx,receiver,author,origin,track_id,true_station,track_dominant,outcome,"
               "gap,normalised_gap,runner_up_gap,gated,contributors,confidence,correct\n";
    }

    Totals tot;
    const std::vector<uint64_t> receivers = parser.receivers();

    for (uint64_t rx : receivers) {
        ObjectConfidenceEngine engine(o.params);
        std::map<uint64_t, TrackTruth> truth;        // engine track id -> what it is really about
        std::map<int64_t, std::set<uint64_t>> tracksOf;  // true station -> tracks it ever fed

        for (const Delivery* d : parser.forReceiver(rx)) {
            const std::vector<VectorRecord> records = engine.onMotionVectors(d->vectors, rx, d->tRx);

            for (std::size_t i = 0; i < records.size(); ++i) {
                const VectorRecord& r = records[i];
                ++tot.vectors;
                tot.gatedSum += r.gatedCandidates;
                if (r.gatedCandidates > 1) { ++tot.contested; }

                int correct = -1;   // -1 = not applicable
                int64_t dominantBefore = -1;

                switch (r.outcome) {
                    case Outcome::Dropped: ++tot.dropped; break;
                    case Outcome::StartedTrack: ++tot.started; break;
                    case Outcome::Rejected: ++tot.rejected; break;
                    case Outcome::Fused: ++tot.fused; break;
                }

                if (r.outcome == Outcome::Fused || r.outcome == Outcome::StartedTrack) {
                    TrackTruth& t = truth[r.trackId];
                    dominantBefore = t.dominant;

                    if (r.outcome == Outcome::Fused && r.trueStation >= 0 && t.dominant >= 0) {
                        ++tot.fusedWithOracle;
                        correct = (r.trueStation == t.dominant) ? 1 : 0;
                        if (correct == 1) {
                            ++tot.correct;
                            tot.confidenceCorrect += r.confidence;
                        } else {
                            ++tot.wrong;
                            tot.confidenceWrong += r.confidence;
                        }
                    }

                    // update what the track is really about, AFTER scoring this vector against
                    // what it was about before -- otherwise every vector scores correct by
                    // definition, because it just voted for its own answer
                    if (r.trueStation >= 0) {
                        tracksOf[r.trueStation].insert(r.trackId);
                        ++t.stationHits[r.trueStation];
                        ++t.hits;
                        int64_t best = -1;
                        int bestCount = 0;
                        for (const auto& kv : t.stationHits) {
                            if (kv.second > bestCount) { bestCount = kv.second; best = kv.first; }
                        }
                        if (t.dominant >= 0 && best != t.dominant) {
                            ++t.switches;
                            ++tot.idSwitches;
                        }
                        t.dominant = best;
                    }
                }

                if (o.noCsv) {
                    continue;
                }
                out << std::fixed << std::setprecision(4) << r.t << ',' << r.receiver << ','
                    << r.author << ',' << (r.origin == Origin::Cam ? "cam" : "cpm") << ','
                    << r.trackId << ',' << r.trueStation << ',' << dominantBefore << ','
                    << outcomeName(r.outcome) << ',' << r.gap << ',' << r.normalisedGap << ','
                    << r.runnerUpGap << ',' << r.gatedCandidates << ',' << r.contributors << ','
                    << std::setprecision(6) << r.confidence << ',' << correct << '\n';
            }
        }

        for (const auto& kv : tracksOf) {
            ++tot.trueObjectsTracked;
            tot.tracksForThose += static_cast<long>(kv.second.size());
            if (kv.second.size() > 1) { ++tot.fragmented; }
        }
    }

    const double assoc = (tot.correct + tot.wrong) > 0
                             ? 100.0 * tot.correct / static_cast<double>(tot.correct + tot.wrong)
                             : 0.0;
    std::printf("\nreceivers       %zu\n", receivers.size());
    std::printf("motion vectors  %ld\n", tot.vectors);
    std::printf("  dropped       %ld (failed plausibility)\n", tot.dropped);
    std::printf("  started       %ld new trajectories\n", tot.started);
    std::printf("  fused         %ld\n", tot.fused);
    std::printf("  rejected      %ld\n", tot.rejected);
    std::printf("\nassociation, scored against the oracle\n");
    std::printf("  scorable      %ld fusions into a track with a known identity\n", tot.fusedWithOracle);
    std::printf("  correct       %ld\n", tot.correct);
    std::printf("  wrong         %ld\n", tot.wrong);
    std::printf("  accuracy      %.2f %%\n", assoc);
    std::printf("  id switches   %ld\n", tot.idSwitches);
    const double frag = tot.trueObjectsTracked > 0
                            ? static_cast<double>(tot.tracksForThose) / tot.trueObjectsTracked
                            : 0.0;
    std::printf("\nfragmentation, per (receiver, real object) pair\n");
    std::printf("  pairs tracked %ld\n", tot.trueObjectsTracked);
    std::printf("  tracks used   %ld\n", tot.tracksForThose);
    std::printf("  mean          %.3f tracks per real object (1.000 is perfect)\n", frag);
    std::printf("  fragmented    %ld pairs needed more than one track (%.2f %%)\n", tot.fragmented,
                tot.trueObjectsTracked > 0
                    ? 100.0 * tot.fragmented / static_cast<double>(tot.trueObjectsTracked)
                    : 0.0);
    std::printf("\ngating\n");
    std::printf("  mean k        %.2f candidates inside the gate per vector\n",
                tot.vectors > 0 ? static_cast<double>(tot.gatedSum) / tot.vectors : 0.0);
    std::printf("  contested     %ld vectors had more than one candidate\n", tot.contested);
    std::printf("\nconfidence\n");
    std::printf("  mean, correct associations  %.3f\n",
                tot.correct > 0 ? tot.confidenceCorrect / tot.correct : 0.0);
    std::printf("  mean, wrong associations    %.3f\n",
                tot.wrong > 0 ? tot.confidenceWrong / tot.wrong : 0.0);
    if (o.noCsv) {
        // One machine-readable line, for scripts/run_experiment_batch.py to collect.
        std::printf("\nBATCH %s gate=%.3f pos_var=%.3f retire=%.2f confirm=%d class=%d "
                    "accuracy=%.4f switches=%ld wrong=%ld started=%ld mean_k=%.4f "
                    "conf_correct=%.4f conf_wrong=%.4f fragmentation=%.4f\n",
                    o.label.empty() ? "run" : o.label.c_str(), o.params.gate, o.params.noise.px,
                    o.params.retireAfter, o.params.confirmAfter, o.params.classMustMatch ? 1 : 0,
                    assoc, tot.idSwitches, tot.wrong, tot.started,
                    tot.vectors > 0 ? static_cast<double>(tot.gatedSum) / tot.vectors : 0.0,
                    tot.correct > 0 ? tot.confidenceCorrect / tot.correct : 0.0,
                    tot.wrong > 0 ? tot.confidenceWrong / tot.wrong : 0.0, frag);
    } else {
        std::printf("\nrows written to %s\n", o.out.c_str());
    }
    return 0;
}
