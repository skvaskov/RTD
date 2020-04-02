function  mirror_rover_frs(file_name)
%MIRROR_ROVER_FRS_ABOUT_HEADING Summary of this function goes here
%   Detailed explanation goes here
load(file_name)

w0_temp = -[w0_des_max,w0_des_min];

w0_des_min = w0_temp(1);
w0_des_max = w0_temp(2);

psiend_temp = -[psi_end_max,psi_end_min];

psi_end_min = psiend_temp(1);
psi_end_max = psiend_temp(2);

psi0_temp = -[psi0_max,psi0_min];
psi0_min = psi0_temp(1);
psi0_max = psi0_temp(2);

delta0_temp = -[delta0_max,delta0_min];
delta0_min = delta0_temp(1);
delta0_max = delta0_temp(2);

clearvars *temp


w = subs(w,[z(2);k(1);k(2)],[-z(2);-k(1);-k(2)]);

zoffset(2) = -zoffset(2);

idx_str = strfind(file_name,'delta0');

file_name = [file_name(1:idx_str+6),num2str(delta0_min,'%0.2f'),'_to_',num2str(delta0_max,'%0.2f'),'.mat'];

clearvars idx_str

save(['mirrored_',file_name])
end

