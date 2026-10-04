# robot-sim validation reports

This directory stores small, reproducible validation outputs that belong to robot-sim.

`phase2a-umi-episode0-frames0-99.jsonl` records a dry-run of the official UMI episode 0 TCP poses against the current UR5 `umi_tcp` kinematics. It contains one run record, one record per frame, adjacent-frame continuity checks, and a summary. Its start-pose alignment is synthetic and does not calibrate the tag frame to a physical UR5 base.

`phase2b1-umi-episode0-frames0-399.jsonl` extends the same single-alignment, sequential-seed dry-run to the first 400 frames. It keeps the Phase 2A report as the frame 0/65/99 regression baseline and adds TCP workspace bounds, per-joint ranges and joint-limit margins.
