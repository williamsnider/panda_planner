How to use this directory

(1) Adjust variables in quartet_common as necessary

(2) run quartet_find_optimal_xyz.m - this identifies the xyz position of the A (pointing up) and W (pointing toward monkey) orientations that work with a quartet (4 shapes) at 4 rotations each.

(3) Selecting the best result from quartet_find_optimal_xyz.m, run quartet_find_optimal_q_extreme.m to calculate 2 joint configurations for each shape+orientation. (Then there will be 32 combinations = 2 configruations * 4 shapes * 4 orientations for each of A and W).

