#ifndef OT_HUNGARIAN_H_
#define OT_HUNGARIAN_H_

/**
 * Joint assignment of motion vectors to trajectories.
 *
 * This file is OURS: it is a thin, explicit interface over a solver that is not.
 * The solver is scipy's rectangular linear sum assignment, vendored unmodified in
 * shared/scipy_lsap/, and it is the code `scipy.optimize.linear_sum_assignment` itself calls.
 * See shared/scipy_lsap/PROVENANCE.md for where it came from, its licence and its paper.
 *
 * That split is why this header lives in include/ and not in shared/: shared/ holds other
 * people's code, byte-identical to its source, and this is our wrapper around one of them.
 *
 * A NOTE ON THE NAME
 *
 * "Hungarian" is how the tracking literature usually refers to optimal linear assignment, and it
 * is the name this project has used since the first version, so it is kept.
 * The algorithm underneath is not literally Kuhn-Munkres: it is a shortest augmenting path method
 * for rectangular assignment, following Crouse (IEEE TAES 52(4), 2016).
 * Both compute the same optimum; they differ in how they get there and in that this one takes a
 * rectangle without padding it to a square.
 *
 * WHY JOINT ASSIGNMENT AND NOT NEAREST NEIGHBOUR PER VECTOR
 *
 * A CPM carries many objects, and choosing the best trajectory for each one independently lets
 * two objects claim the same trajectory.
 * The joint minimum enforces one vector per trajectory, and it is the part of this pipeline that
 * resists swaps when two objects cross.
 * Measured on the full dataset against greedy first-fit, it produced 4.3 per cent fewer identity
 * switches and no better per-vector accuracy, which is the honest way to state what it buys.
 * The ablation that produced that number was removed on 2026-10-11; the number is recorded in
 * docs/object_trajectory_mvp.md.
 */

#include "../shared/scipy_lsap/rectangular_lsap.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ot {

/**
 * The assignment stage, as a namespace with no state.
 *
 * Everything here is static and the class is never instantiated: it exists so that the call site
 * reads `Hungarian::solveAssignment(...)` and says plainly which stage is running, rather than a
 * bare function that could be anything.
 */
class Hungarian {
public:
    Hungarian() = delete;

    /// No trajectory was assigned to this vector.
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    /**
     * The cost of a pair the gate rejected.
     *
     * Large enough that the solver never prefers one, and finite rather than infinite so that the
     * problem stays feasible when a whole column is forbidden: the solver must always be able to
     * return something, and the caller discards the forbidden pairs afterwards.
     */
    static constexpr double kForbidden = 1e9;

    /// Rows are trajectories, columns are motion vectors, row major.
    struct CostMatrix {
        std::size_t rows = 0;
        std::size_t cols = 0;
        std::vector<double> cost;

        double at(std::size_t r, std::size_t c) const { return cost[r * cols + c]; }
        void set(std::size_t r, std::size_t c, double v) { cost[r * cols + c] = v; }
        void resize(std::size_t r, std::size_t c)
        {
            rows = r;
            cols = c;
            cost.assign(r * c, kForbidden);
        }
    };

    /**
     * Minimum-cost assignment. Returns, for each column, the row assigned to it, or kNone.
     *
     * A forbidden pair may still come back assigned when the solver had no cheaper way to cover a
     * column, so the caller checks the cost against kForbidden and discards those.
     */
    static std::vector<std::size_t> solveAssignment(const CostMatrix& m)
    {
        if (m.rows == 0 || m.cols == 0) {
            return std::vector<std::size_t>(m.cols, kNone);
        }

        // The solver writes min(rows, cols) pairs: rowOut[k] is a row, colOut[k] its column.
        const std::size_t pairs = (m.rows < m.cols) ? m.rows : m.cols;
        std::vector<int64_t> rowOut(pairs, 0), colOut(pairs, 0);

        // It takes the cost array by non-const pointer but does not modify it; the copy is cheap
        // and keeps the promise this signature makes.
        std::vector<double> costCopy = m.cost;

        const int status = solve_rectangular_linear_sum_assignment(
            static_cast<intptr_t>(m.rows), static_cast<intptr_t>(m.cols), costCopy.data(),
            /*maximize=*/false, rowOut.data(), colOut.data());

        std::vector<std::size_t> columnToRow(m.cols, kNone);
        if (status != 0) {
            return columnToRow;   // infeasible; every vector starts its own trajectory
        }
        for (std::size_t k = 0; k < pairs; ++k) {
            const std::size_t c = static_cast<std::size_t>(colOut[k]);
            const std::size_t r = static_cast<std::size_t>(rowOut[k]);
            if (c < m.cols && r < m.rows) {
                columnToRow[c] = r;
            }
        }
        return columnToRow;
    }
};

}  // namespace ot

#endif  // OT_HUNGARIAN_H_
