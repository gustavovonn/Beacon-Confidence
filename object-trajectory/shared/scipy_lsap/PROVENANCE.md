# scipy's rectangular linear sum assignment, vendored

This directory is not our code.
It is the solver behind `scipy.optimize.linear_sum_assignment`, copied in unmodified.

## What and where from

    project   SciPy
    files     scipy/optimize/rectangular_lsap/rectangular_lsap.{cpp,h}
    source    https://github.com/scipy/scipy/blob/main/scipy/optimize/rectangular_lsap/
    taken     2026-10-11
    author    PM Larsen
    licence   BSD 3-clause, header retained verbatim in the source files
    algorithm shortest augmenting path for rectangular assignment, following
              D. F. Crouse, "On implementing 2D rectangular assignment algorithms",
              IEEE Transactions on Aerospace and Electronic Systems 52(4), 2016,
              doi 10.1109/TAES.2016.140952

## Changes

None.
Both files are byte-identical to the ones in that repository.

## Why this one

It IS the reference.
`scipy.optimize.linear_sum_assignment` calls exactly this code, so validating against scipy and
using scipy become the same act, and the algorithm has a citable paper behind it.

It is also rectangular natively.
Our cost matrix is N trajectories by m motion vectors, with N usually far larger than m; the
solver we lifted from cp-trustfulness pads that to a square N by N and runs in cubic time in N,
which is why that code needed a compaction step to stay affordable.
This one takes the rectangle as it is.

It carries no dependencies beyond the standard library: `<cmath>`, `<vector>`, `<numeric>`,
`<algorithm>`, `<stdint.h>`, `<stdbool.h>`.
That keeps the engine buildable on the VisionFive 2 with a compiler and nothing else.

## Interface

    int solve_rectangular_linear_sum_assignment(intptr_t nr, intptr_t nc,
                                                double* input_cost, bool maximize,
                                                int64_t* a, int64_t* b);

Row major cost, output arrays of length min(nr, nc), returns 0 on success.
