function plot_foot(ax, pos, ori, color)
%PLOT_FOOT Plots outline of physical foot and foot dimensions used by
%controller

foot_half_len = 0.11;
foot_half_wid = 0.04;
foot_half_len_ctrl = 0.08;
foot_half_wid_ctrl = 0.03;

rmat = quat2mat(ori);
[xx, yy] = meshgrid(linspace(-foot_half_len, foot_half_len, 2), ...
                     linspace(-foot_half_wid, foot_half_wid, 2));
[xx_ctrl, yy_ctrl] = ...
    meshgrid(linspace(-foot_half_len_ctrl, foot_half_len_ctrl, 2),...
    linspace(-foot_half_wid_ctrl, foot_half_wid_ctrl, 2));
stack_xx_yy = cat(3, xx, yy);
stack_xx_yy = permute(stack_xx_yy, [3 2 1]);
stack_xx_yy = pagetranspose(stack_xx_yy);
out = einsum('ji, mni->jmn', rmat(1:2, 1:2), stack_xx_yy);
xx = out(:, :, 1);
yy = out(:, :, 2);
xx = xx + pos(1);
yy = yy + pos(2);

stack_xx_yy_ctrl = cat(3, xx_ctrl, yy_ctrl);
stack_xx_yy_ctrl = permute(stack_xx_yy_ctrl, [3 2 1]);
stack_xx_yy_ctrl = pagetranspose(stack_xx_yy_ctrl);
out = einsum('ji, mni->jmn', rmat(1:2, 1:2), stack_xx_yy_ctrl);
xx_ctrl = out(:, :, 1);
yy_ctrl = out(:, :, 2);
xx_ctrl = xx_ctrl + pos(1);
yy_ctrl = yy_ctrl + pos(2);

plot(ax, xx, yy, color, 'LineWidth', 1.5)
plot(ax, xx', yy', color, 'LineWidth', 1.5)
plot(ax, xx_ctrl, yy_ctrl, color, 'LineWidth', 1.5, 'LineStyle','--')
plot(ax, xx_ctrl',yy_ctrl', color, 'LineWidth', 1.5, 'LineStyle','--')

end