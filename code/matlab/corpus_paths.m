function paths = corpus_paths()
%CORPUS_PATHS Resolve portable project paths from this file's location.
% This avoids machine-specific drive letters and user-directory paths.

    matlabDir = fileparts(mfilename('fullpath'));
    paths.root = fileparts(fileparts(matlabDir));
    paths.rawUniform = fullfile(paths.root, 'data', 'ns3_rf_measurements', 'fence24_uniform');
    paths.rawTiered = fullfile(paths.root, 'data', 'ns3_rf_measurements', 'fence24_tiered_wall');
    paths.processed = fullfile(paths.root, 'reproduced_output', 'processed_results');
    paths.simulink = fullfile(paths.root, 'reproduced_output', 'simulink');
    paths.layouts = fullfile(paths.root, 'reproduced_output', 'sensor_layouts');
    paths.figures = fullfile(paths.root, 'reproduced_output', 'figures');
    paths.models = fullfile(paths.root, 'reproduced_output', 'models');
    paths.scenarios = fullfile(paths.root, 'reproduced_output', '100_seed_scenarios');

    outputDirs = {paths.processed, paths.simulink, paths.layouts, ...
        paths.figures, paths.models, paths.scenarios};
    for i = 1:numel(outputDirs)
        if ~exist(outputDirs{i}, 'dir')
            mkdir(outputDirs{i});
        end
    end
end
