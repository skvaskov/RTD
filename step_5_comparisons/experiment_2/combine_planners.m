nmpc_dir = dir('./nmpc0');
rrt_rtd_dir = dir('./rtd_rrt');

file_save_header = 'rover_experiment_2_summary';

rrt_idx = 1;


for i = 1001:1002
   rtemp = load(['./rtd_rrt/',rrt_rtd_dir(i).name]);
   ntemp = load(['./nmpc0/',nmpc_dir(i).name]);
   
   if rtemp.w_idx == ntemp.w_idx
       
       summary(1) = rtemp.summary(3);
       summary(2) = rtemp.summary(rrt_idx);
       summary(3) = ntemp.summary;
       
       save([file_save_header,'_world_',num2str(ntemp.w_idx,'%04d'),'.mat'],'summary','w_idx')
       
   end
   
   
end