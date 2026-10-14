# 记录器对一件工件的独立验收。run_trials 等这个消息,不把执行器的 COMPLETED 当成成功。
string batch_id
string task_id
string part_id
# 生成时的给定类别。空串表示这轮没有清单。
string ground_truth_category
# 来料消息里的类别:默认等于给定类别,视觉开启时是识别结果。
string predicted_category
# 执行器打算放入的料区。失败且还没决定料区时为空。
string commanded_bin
# 规划场景里观察到的料区。不在任何料区时为空。
string final_bin
# PLACED / MISPLACED / OUTSIDE / FAILED / NOT_FOUND
string result
string detail
# 从 PICKING 到终态的秒数。没有测到时为 0。
float64 elapsed_sec
