function message = helper_functions
    assignin('base','set_fig_opt', @set_fig_opt);
    assignin('base','plot_phase', @plot_phase);
    message.set_fig_opt = @set_fig_opt;
    message.plot_phase = @plot_phase;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%helper functions list
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure opt
function [] = set_fig_opt()
    set(gca, 'LineWidth', 3)
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', 30)
    set(gca, 'Color', 'white')
end

% plot_phase
function [] = plot_phase(time, state, min, max, phase_color)
    if min == 0 && max ==0
        min = min - 1;
        max = max + 1;
    end
    ylim([min, max])
    % fill out the background of the plot based on the phase(state)
    time_idx_at_phase_change = [];
    state_at_phase_change = [];
    for idx = 1 : length(time) - 1
        if state(idx) ~= state(idx + 1)
            time_idx_at_phase_change = [time_idx_at_phase_change, idx + 1];
            state_at_phase_change = [state_at_phase_change, state(idx)];
        end
    end

    prev_time = 0;
    state_idx = 1;
    for idx = time_idx_at_phase_change
        time_at_phase_change = time(idx);
        %fill out the plot
        x = [prev_time, prev_time, time_at_phase_change, time_at_phase_change];
        y = [min, max, max, min];
        fill(x, y, 'w', 'FaceColor', phase_color(state_at_phase_change(state_idx)),'FaceAlpha', 0.2)
        prev_time = time_at_phase_change;
        state_idx = state_idx + 1;
    end
    x = [prev_time, prev_time, time(end), time(end)];
    y = [min, max, max, min];
    fill(x, y, 'w', 'FaceColor',phase_color(state(end)), 'FaceAlpha',0.2)

    %TODO: legend showing map: phase_color -> state(int) -> state(string)
end
