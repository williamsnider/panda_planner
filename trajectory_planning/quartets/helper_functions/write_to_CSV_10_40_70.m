function   write_to_CSV_10_40_70(traj_10, traj_40, traj_70, fname_base)

%WRITE_TO_CSV_10_40_70 Summary of this function goes here
fname = strcat(fname_base,"_",num2str(10),"%.csv");
writematrix(traj_10,fname)
fname = strcat(fname_base,"_",num2str(40),"%.csv");
writematrix(traj_40,fname)
fname = strcat(fname_base,"_",num2str(70),"%.csv");
writematrix(traj_70,fname)
end



        