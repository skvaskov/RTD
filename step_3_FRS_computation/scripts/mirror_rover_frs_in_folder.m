clear; close

frs_dir = '/Users/seanvaskov/MATLAB/IJRR_bridging_the_gap/step_3_FRS_computation/data/rover_reconstructed/';

dir_info = dir(frs_dir);

for i = 3:length(dir_info)
   mirror_rover_frs([dir_info(i).name]) 
end