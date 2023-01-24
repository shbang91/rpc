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


% use einsum as in numpy
function Out=einsum(str,varargin)
    %% Einsum for MATlab
    % Einstein Sommation like Numpy's einsum
    % Input :
    %   - str : String like 'ik,kj-> ij'
    %   - varargin : Double called
    % Output :
    %   - Out : Double
    % Usage :
    % Matrix multiplication C = A*B
    % A(ik)*B(kj) = C(ij) -> C=einsum('ik,kj->ij',A,B)
    % Limitations :
    % - Take only "diagonal terms" for an nd-array isn't permitted
    %       ex : 'iij,jk -> ik'
    % - Be careful about approximation error
    %       ex : Doing A^12 with einsum gives you a N_inf(relative_error) around 1E-10

    % Stephane Nachar - LMT - stephane.nachar@lmt.ens-cachan.fr

    %str='ijkl,ikm,jkn -> lmn';
    %arrays = {rand(6,6,100,100),rand(6,100,50),rand(6,100,50)};

    %% Check string to know sum componants and output componants
    ARROW = [45 62];
    SPACE = 32;
    VIRGULE = 44;
    % Clear spaces
    str = double(str);str(str==SPACE) = [];
    % Split with the arrow
    test = find(str(1:end-1)==ARROW(1) & str(2:end)==ARROW(2));
    if ~isscalar(test); error('Wrong input - str must be like ''ik,kj->ij'''); end
    outstr  = str(test+2:end);
    instr   = str(1:test-1);
    % Split input componant information
    delimiter = [0,find(instr==VIRGULE),numel(instr)+1];
    id_comps = cell(numel(delimiter)-1,1);
    for id = 1:numel(id_comps)
        id_comps{id} = instr(delimiter(id)+1:delimiter(id+1)-1);
    end
    instr(instr==VIRGULE) = [];

    % List of componants : Summed componants will be at the end of expressions
    all_comps = unique([outstr instr],'stable');

    % Convert input componant information
    ncomp = numel(all_comps);
    for id = 1:numel(id_comps)
        [~,id_comps{id}] = ismember(id_comps{id},all_comps);
    end

    nout = numel(outstr);           % n output componants
    comps_sum = nout+1:ncomp;       % list summed componants
    size_comps = size_comps_fct(all_comps,varargin,id_comps); % size of each componants
    nb_ops = numel(comps_sum)+1;    % operation number

    %% Take the best path to sum componants
    [path,ops,memory_max] = score_path(size_comps,comps_sum,id_comps);

    %if memory_max > 8E9
    %    warning('Asked memory may be too important');
    %end

    %% Compute operations
    Out = 1;
    for id = 1:nb_ops
        ids_array = find(ops(id,:));        % Find arrays needed to be computed
        for id_array = ids_array
            idx_permutation = dims(ncomp,id_comps{id_array});
            varargin{id_array} = permute(varargin{id_array},idx_permutation);
            Out = Out.*varargin{id_array};  % Compute elementwise multiplication
            varargin{id_array} = [];        % Suppress array from memory
        end
        if id ~= nb_ops % Because the last operation is for arrays who don't need to be summed (just multiplied)
            Out = sum(Out,path(id));
            ncomp = ncomp-1;
        end
    end
    end

    function nids = dims(n,ids)
    % For permutation
    nids = zeros(n,1);
    nids(ids) = 1:numel(ids);
    not_dims = nids==0;
    nids(not_dims)=numel(ids)+(1:sum(not_dims));
    end

    function siz = size_comps_fct(all_comps,arrays,in)
    siz = zeros(1,numel(all_comps));
    for id = 1:numel(arrays)
        tmp = [size(arrays{id}) ones(1,numel(in{id})-ndims(arrays{id}))];
        siz(in{id}) = tmp;
    end
    end

    function is_comp = is_in(in,comp_sum)
    % Check if a componant is in an array for summation
    is_comp = false(1,numel(in));
    for id = 1:numel(in)
        is_comp(id) = any(in{id}==comp_sum);
    end
    end

    function [path,ops,memory_max] = score_path(size_comps,comps_sum,in)
    %% Path searcher
    % Search the best operation to reduce the memory used
    if factorial(numel(comps_sum))<1E3
        paths = perms(comps_sum);       % Paths allowed
    else
        warning('Too many paths are possible - Test only someone');
        paths = zeros(10,numel(comps_sum));
        for id = 1:10
            paths(id,:) = comps_sum(randperm(numel(comps_sum)));
        end
    end
    nb_ops = numel(comps_sum)+1;
    nb_paths = size(paths,1);
    nb_array = numel(in);
    mem_score = zeros(1,nb_paths);
    tops = cell(1,nb_paths);        % Cell to keep operations for each path

    for id_path = 1:nb_paths
        path = paths(id_path,:);
        not_computed = true(1,nb_array);
        numel_out = 0;
        init = 0;
        ops = false(nb_ops,nb_array);
        for i_path = 1:numel(path)
            comp_sum = path(i_path);
            is_comp = is_in(in,comp_sum);
            is_comp = find(is_comp);
            numel_max = numel_out;
            n = init;
            for i_comp = is_comp
                if not_computed(i_comp)
                    ops(i_path,i_comp) = true;
                    size_array = size_comps(in{i_comp});
                    numel_max = max([numel_max,prod(size_array)]);
                    n = n+1;
                    not_computed(i_comp) = false;
                end
                mem_score(id_path) = max(mem_score(id_path),8*numel_max);
            end
            init = 1;
            numel_out = numel_max;
        end
        ops(end,:) = not_computed;
        tops{id_path} = ops;
    end
    [memory_max,id] = min(mem_score);
    ops = tops{id};
    path = paths(id,:);
end