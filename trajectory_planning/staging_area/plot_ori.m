function  plot_ori(ori_cell)

num_ori = numel(ori_cell);
for i = 1:num_ori

    shift = 1.5;

    ori = eye(4);
    R = ori_cell{i};
    ori(1:3, 1:3) = R;
    ori(1:3,4) = ori(1:3,4) - ori(1:3,3) * shift;

    plotTransforms(se3(ori)); hold on;
    xlabel('x')
    ylabel('y')
    zlabel('z')

end

end

