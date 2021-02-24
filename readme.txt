Deprecated: this includes perturbations and tour merging. I am moving towards maintaining just the hill climbing bit, and factoring out the perturbation and tour merging as python scripts.

Code style disclaimer: the style can be a little inconsistent, because I am migrating to a style closer to the Google C++ Style Guide.
See "Style notes" for more detail.

This is an iterated local search TSP solver that implements a k-opt hill climber in the style of Lin-Kernighan (LK) or Lin-Kernighan-Helsgaun (LKH).
Novel features of this TSP solver:

1. Increase efficiency by using a quadtree to dynamically determine and limit the k-opt search,
    such that candidate edges that violate the LK gain criterion are skipped.
    In other words, all possible improving steps (deletion of one edge and addition of one edge) are considered.
    This approach is novel because fixed search neighborhoods are typically used.
    Original LK limited search neighborhoods to 5 nearest points. LKH limits search to 5 "alpha-nearest" points.
    Alpha-nearness measures the similarity of new edges to those in a minimum spanning tree.
    Computing alpha-nearness is done once before hill-climbing and takes O(n^2) work.
    I estimate the work complexity of each improvement step in the present implementation to be linearithmic,
    perhaps O(n * (log(n) ^ (kmax - 1))).
    For those unfamiliar with LK gain criterion, it is essentially an improvement upon naive k-opt search for sequential moves.
    Sequential moves are moves that can be made by alternating deletion and addition of edges that share a common point.
    LK looks directly for a feature of improving moves, which is that there is at least 1 new edge that shares a common point with a longer deleted edge.
    This prunes out a lot of the k-opt search space.

2. Preserve sequential-move k-optimality. LK / LKH gives up sequential-move k-optimality.
    Note that sequential-move k-optimality is equivalent to full k-optimality for k in [2, 3].
    Finding higher-opt moves with this implementation is probably slower than in LKH (disregarding LKH's initial O(n^2) MST creation),
    but there might be moves that are found by this implementation that would be undetected in LKH. Proof:
        Using 3-opt hill-climber on first world record (by LKH) of lrb744710 quickly returns improvements.
        Although it is an old world record, it proves that LKH (and LK) gives up k-optimality by arbitrarily fixing search neighborhoods.
        http://www.math.uwaterloo.ca/tsp/vlsi/lrb744710.log.html

3. Increase inter-iteration efficiency by skipping searches that cannot result in any changes.
    Associate every point with a box that covers the entire domain that was explored in its k-opt search.
    Then redo the search for said point only if changed points fall within said box.

4. A new, efficient tour merging heuristic, which I call Difference Decomposition (DD). DD finds a set of k-exchange moves that fully describes the difference between two tours
    and recombines the moves to try to find an improvement, excluding moves that only decrease improvement.
    This was motivated by the observation that as problem instances get larger, disparate sections of the tour appear to have their own local optima.
    I do not yet have a metric to compare this to similar methods such as Iterative Partial Transcription (IPT) and Generalized Partition Crossover (GPX).
    DD compares 2 tours: a current best tour and a candidate tour generated from a perturbed hill-climb.
    Common edges are ignored, and different edges are subdivided into groups. Within each group, edges are comprised of points that are disjoint from those of other groups.
    Each group is then a k-opt move, and we now have a set of k-opt moves that fully describe the difference between the 2 tours!
    Combinations of these moves are then considered to try to find the best (highest improvement) combination that
    also does not break the tour into multiple cycles. Many moves can be ignored or removed based on whether their inclusion would cause the improvement to be <= 0.

Compilation:
1. Make sure "CXX" in "makefile" is set to the desired compiler.
2. Run "make".

Running:
1. Obtain tsp instance and tour files. You can use download_tsp_data.py.
2. Modify config.txt to point to your tsp instance and (optional) tour file.
3. Run "./k-opt.out" for usage details.

Style notes:
1. Namespaces follow directory structure. If an entire namespace is in a single header file, the header file name will be the namespace name.
2. Headers are grouped from most to least specific to this repo (e.g. repo header files will come before standard library headers).
3. Put one line break in between function definitions for convenient vim navigation via ctrl + { and ctrl + }.
4. Filenames are lower case and underscore-separated.
5. Header extension is .hh and source extension is .cc.
6. Reference and pointer symbols are right-associated, e.g. "T &ref" vs. "T& ref".
7. Exceptions are not caught.
8. Compile-time constants are capitalized with underscores.
9. Member variables are indicated as such with trailing underscores.
10. Scope start brace is not new-line separated.
11. Prefer English-word boolean operators: "and", "not".

TODO:
1. Memoize moves between 2 points that are better than the direct distance between them.
2. Check combinations of non-feasible non-sequential moves that may make a feasible non-sequential move.

