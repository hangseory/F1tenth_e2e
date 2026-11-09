-- Frozen original map + in-memory mapping during this drive.
-- No pure-localization trimmer: retain new submaps throughout this session.
-- The launch does not save state on exit; next start reloads the original.
include "cartographer_2d.lua"
options.publish_tracked_pose = true
POSE_GRAPH.optimize_every_n_nodes = 20
return options
