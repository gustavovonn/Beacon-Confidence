# Eigen 3.4.0, vendored

This directory is not our code.
It is the Eigen linear algebra library, copied in unmodified.

## What and where from

    library   Eigen
    version   3.4.0
    source    https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    sha256    8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    taken     2026-10-10
    licence   MPL2 for the library itself; see COPYING.README for the parts under BSD and LGPL

## Changes

None.
The `Eigen/` directory is byte-identical to the one in that archive.
Only `unsupported/`, `doc/`, `test/`, `demos/` and `bench/` were left out, because nothing here
uses them.

## Why 3.4.0 and not the newest

The distribution's package is newer, but 3.4.0 is the release the C++ ecosystem has shaken out
over years of deployment.
When the point of taking a library is to stop having to validate the code yourself, the most
widely deployed version is the one that buys the most.

## Why vendored rather than installed

Eigen is header-only, so vendoring costs nothing at runtime and nothing in the build.
It is what keeps the engine buildable on the VisionFive 2 with a compiler and `make` and no
network: copy the directory, type `make`, done.
An installed system package would not survive that.

## What we use it for

The matrix arithmetic of the filter, and in particular the Cholesky decomposition the unscented
transform needs to place its sigma points.
Hand-written Cholesky is the single most delicate piece of numerical code a UKF contains, and it
is exactly the kind of thing that should not be ours to get right.
