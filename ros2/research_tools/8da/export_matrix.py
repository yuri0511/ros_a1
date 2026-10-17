import json,csv,sys
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b');allcases=[];rows=[]
for case in sorted((root/'runs').iterdir()):
 if not (case/'analysis.json').exists():continue
 a=json.loads((case/'analysis.json').read_text());meta=json.loads((case/'metadata.json').read_text());allcases.append(a)
 extra=json.loads((case/'extra-analysis.json').read_text()) if (case/'extra-analysis.json').exists() else []
 for idx,i in enumerate(a['instances']):
  run=Path(i['path']);e=extra[idx] if len(extra)>idx else {};contacts=json.loads((run/'contacts-task-summary.json').read_text()) if (run/'contacts-task-summary.json').exists() else None
  counts={key:sum(x['result']==key for x in i['results']) for key in ('PLACED','FAILED','MISPLACED','OUTSIDE')}
  manifest=json.loads((case/'configuration-manifest.json').read_text()) if (case/'configuration-manifest.json').exists() else {}
  row={'configuration_hash':manifest.get('configuration_hash'),'source_version':manifest.get('source_version'),'native_controller_aborts':e.get('controller_abort_count'),'controller_execution_error_log_lines':e.get('controller_execution_error_lines'),'return_standby_failures':e.get('return_standby_failures'),'case':case.name,'cohort':'fixture' if meta.get('fixture')=='1' else ('contact_validation' if meta['contact_mode']=='1' else 'low_interference'),'commit':meta['commit'],'source_hash':meta['source_hashes'],'physics_ms':meta['step_ms'],'fast_waits':meta['fast_waits'],'instances':meta['instances'],'instance':idx+1,'target_rtf':meta['rate'],'lp_threads':meta['threads'],'reason':a['reason'],'instance_all8_correct':i['successful'],'rows':i['row_count'],**counts,'instance_wall_s':i['wall_s'],'task_sim_s':e.get('batch_task_sim_s'),'instance_rtf_steady':i['actual_rtf'],'instance_rtf_full_stats':i['rtf_from_first_stats'],'rtf_5s_p10':i['rtf_5s_p10'],'rtf_5s_p90':i['rtf_5s_p90'],'retries':i['retries'],'whole_case_success_batches':a['successful_batches'],'whole_case_wall_s':a['experiment_wall_s'],'whole_case_ep_per_hour':a['episodes_per_hour'],'whole_case_lifecycle_ep_per_hour':a['lifecycle_episodes_per_hour'],'cpu_percent':a['cpu_mean_percent'],'cpu_p95_percent':a['cpu_p95_percent'],'min_available_ram_gib':a['min_mem_available_gib'],'swapout_delta':a['swapout_delta'],'oom_delta':a['oom_delta'],'vision_errors':len(i['vision_mismatches']),'controller_errors':len(e.get('controller_runtime_messages',[])),'interlock_errors':len(i['interlock_violations']),'cross_ros_markers':len(i['isolation']['ros_foreign']),'cross_gz_markers':len(i['isolation']['gz_foreign']),'placement_count':e.get('placement_count'),'max_xy_error_m':e.get('max_xy_error_m'),'mean_xy_error_m':e.get('mean_xy_error_m'),'contact_rows':contacts['rows'] if contacts else None,'contact_bad_rows':contacts['bad_rows'] if contacts else None,'interarm_contact_points':contacts['categories'].get('interarm',{}).get('points',0) if contacts else None}
  for label,d in e.get('waits',{}).items():row[label+'_calls']=d['count'];row[label+'_wall_sum_s']=d['wall_sum_s']
  rows.append(row)
(root/'all-analyses.json').write_text(json.dumps(allcases,indent=2));fields=list(dict.fromkeys(k for r in rows for k in r))
with (root/'EXPERIMENT_MATRIX.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
print('Exported',len(allcases),'cases',len(rows),'instance rows')
