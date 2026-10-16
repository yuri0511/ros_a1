# Local V2 calibration experiment

Upstream72ab6f2, independent schema2 package. No Gazebo deployment. See the seven reports in ~/yuri_gazebo_performance/model_calibration_v2/.

Use experiment_tools/ and root REPRO_COMMANDS.txt. Paths deliberately refer to this local experiment and original immutable integration data. Keep original outputs before a rerun.

The frozen config uses32 pieces from complete pairs1/2; pair3 is held out. Empirical profile resampling is a bounded sensitivity assumption, not a validated physical failure probability. READY is dispatcher availability, not certified physical standby. New tables remain deployment_ready=false / overlap_validated=false.
