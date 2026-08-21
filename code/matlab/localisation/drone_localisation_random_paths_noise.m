clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% RANDOM-WAYPOINT FENCE24 NOISE LOCALISATION:
% Reads seeded random waypoint ns-3/FlyNetSim paths and directly tests noisy
% localisation. This intentionally skips the ideal-only plot stage.

fileInfo = dir(fullfile(paths.rawUniform, 'prison_fence24_random_seed*.csv'));
if isempty(fileInfo)
    error('No random-path CSV files found in the corpus data directories.');
end

dataFiles = strings(numel(fileInfo), 1);
for i = 1:numel(fileInfo)
    dataFiles(i) = string(fullfile(fileInfo(i).folder, fileInfo(i).name));
end

noiseSigmaDbList = [1, 2, 3, 5];
numTrials = 100;
minArea2 = 1.0;
rng(202);

txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
frequencyHz = 2.4e9;

friisRange = @(rxPowerDbm) 10.^((txPowerDbm + txGainDbi + rxGainDbi ...
    - rxPowerDbm - 20*log10(frequencyHz) + 147.55) / 20);

allData = table();
for f = 1:numel(dataFiles)
    T = readtable(dataFiles(f), 'TextType', 'string');
    [~, fileName, fileExt] = fileparts(char(dataFiles(f)));
    T.experiment_file = repmat(string([fileName fileExt]), height(T), 1);
    allData = [allData; T]; %#ok<AGROW>
end

[groupIds, ~] = findgroups(allData.experiment_file, allData.random_seed, allData.time);
baseRecords = struct('rows', {});
for g = 1:max(groupIds)
    groupRows = allData(groupIds == g, :);
    cleanDetected = groupRows(groupRows.detected == 1, :);
    if height(cleanDetected) >= 3 && hasNonCollinearGeometry(cleanDetected.sensor_x, cleanDetected.sensor_y)
        baseRecords(end + 1).rows = groupRows; %#ok<SAGROW>
    end
end

nBase = numel(baseRecords);
methodNames = ["top3_trilateration", "all_sensor_wls", "robust_wls"];
nRows = nBase * numel(noiseSigmaDbList) * numTrials * numel(methodNames);

experimentFile = strings(nRows, 1);
layout = strings(nRows, 1);
dronePath = strings(nRows, 1);
method = strings(nRows, 1);
randomSeed = zeros(nRows, 1);
randomTargetX = zeros(nRows, 1);
randomTargetY = zeros(nRows, 1);
noiseSigmaDb = zeros(nRows, 1);
trialId = zeros(nRows, 1);
timeS = zeros(nRows, 1);
uavX = zeros(nRows, 1);
uavY = zeros(nRows, 1);
uavZ = zeros(nRows, 1);
zone = strings(nRows, 1);
timeToBoundaryS = zeros(nRows, 1);
cleanSensorCount = zeros(nRows, 1);
noisySensorCount = zeros(nRows, 1);
usedSensorCount = zeros(nRows, 1);
rejectedSensorCount = zeros(nRows, 1);
maxNormResidual = nan(nRows, 1);
localised = false(nRows, 1);
estimatedX = nan(nRows, 1);
estimatedY = nan(nRows, 1);
localisationError = nan(nRows, 1);
wlsIterations = nan(nRows, 1);
wlsConverged = false(nRows, 1);

rowIndex = 1;

for s = 1:numel(noiseSigmaDbList)
    sigmaDb = noiseSigmaDbList(s);

    for trial = 1:numTrials
        for bg = 1:nBase
            groupRows = baseRecords(bg).rows;
            noisyRxAll = groupRows.rx_power_dbm + sigmaDb * randn(height(groupRows), 1);
            noisyDetectedMask = noisyRxAll >= groupRows.threshold_dbm;
            noisyRows = groupRows(noisyDetectedMask, :);
            noisyRx = noisyRxAll(noisyDetectedMask);

            top3 = top3Estimate(noisyRows, noisyRx, friisRange, minArea2);
            allWls = allSensorWlsEstimate(noisyRows, noisyRx, friisRange, sigmaDb, top3);
            robust = robustWlsEstimate(noisyRows, noisyRx, friisRange, sigmaDb, top3);
            results = [top3, allWls, robust];

            for r = 1:numel(results)
                experimentFile(rowIndex) = groupRows.experiment_file(1);
                layout(rowIndex) = groupRows.sensor_layout(1);
                dronePath(rowIndex) = groupRows.drone_path(1);
                method(rowIndex) = results(r).method;
                randomSeed(rowIndex) = groupRows.random_seed(1);
                randomTargetX(rowIndex) = groupRows.random_target_x(1);
                randomTargetY(rowIndex) = groupRows.random_target_y(1);
                noiseSigmaDb(rowIndex) = sigmaDb;
                trialId(rowIndex) = trial;
                timeS(rowIndex) = groupRows.time(1);
                uavX(rowIndex) = groupRows.uav_x(1);
                uavY(rowIndex) = groupRows.uav_y(1);
                uavZ(rowIndex) = groupRows.uav_z(1);
                zone(rowIndex) = groupRows.zone(1);
                timeToBoundaryS(rowIndex) = groupRows.time_to_boundary(1);
                cleanSensorCount(rowIndex) = groupRows.num_sensors_detecting(1);
                noisySensorCount(rowIndex) = height(noisyRows);
                usedSensorCount(rowIndex) = results(r).usedCount;
                rejectedSensorCount(rowIndex) = results(r).rejectedCount;
                maxNormResidual(rowIndex) = results(r).maxNormResidual;
                localised(rowIndex) = results(r).ok;
                estimatedX(rowIndex) = results(r).x;
                estimatedY(rowIndex) = results(r).y;
                wlsIterations(rowIndex) = results(r).iterations;
                wlsConverged(rowIndex) = results(r).converged;

                if results(r).ok
                    localisationError(rowIndex) = sqrt((results(r).x - groupRows.uav_x(1))^2 + ...
                        (results(r).y - groupRows.uav_y(1))^2);
                end

                rowIndex = rowIndex + 1;
            end
        end
    end
end

R = table(experimentFile, layout, dronePath, randomSeed, randomTargetX, randomTargetY, ...
    method, noiseSigmaDb, trialId, timeS, uavX, uavY, uavZ, zone, timeToBoundaryS, ...
    cleanSensorCount, noisySensorCount, usedSensorCount, rejectedSensorCount, ...
    maxNormResidual, localised, estimatedX, estimatedY, localisationError, ...
    wlsIterations, wlsConverged);

R.Properties.VariableNames = {'experiment_file', 'layout', 'drone_path', ...
    'random_seed', 'random_target_x', 'random_target_y', 'method', ...
    'noise_sigma_db', 'trial_id', 'time_s', 'uav_x', 'uav_y', 'uav_z', ...
    'zone', 'time_to_boundary_s', 'clean_sensor_count', 'noisy_sensor_count', ...
    'used_sensor_count', 'rejected_sensor_count', 'max_normalised_residual', ...
    'localised', 'estimated_uav_x', 'estimated_uav_y', 'localisation_error_m', ...
    'wls_iterations', 'wls_converged'};

detailOutput = fullfile(paths.processed, 'prison_fence24_random_paths_noise_results.csv');
writetable(R, detailOutput);

S = buildSummary(R, noiseSigmaDbList);
summaryOutput = fullfile(paths.processed, 'prison_fence24_random_paths_noise_summary.csv');
writetable(S, summaryOutput);

fprintf('\nRandom waypoint noise localisation completed.\n');
fprintf('Random CSV files: %d\n', numel(dataFiles));
fprintf('Clean localisation-ready timestamps: %d\n', nBase);
fprintf('Noise levels: %s dB\n', mat2str(noiseSigmaDbList));
fprintf('Trials per noise level: %d\n', numTrials);
fprintf('Detailed output: %s\n', detailOutput);
fprintf('Summary output: %s\n', summaryOutput);

overall = S(S.random_seed == 0, :);
disp(overall(:, {'noise_sigma_db', 'method', 'total_cases', 'localised_cases', ...
    'success_rate_pct', 'mean_localisation_error_m', 'rmse_localisation_error_m', ...
    'p95_localisation_error_m', 'mean_rejected_sensor_count'}));

plotOverallRmse(S);
plotOverallP95(S);
plotPerSeedWls(S);

function result = blankResult(methodName)
    result = struct('method', methodName, 'ok', false, 'x', NaN, 'y', NaN, ...
        'usedCount', 0, 'rejectedCount', 0, 'maxNormResidual', NaN, ...
        'iterations', NaN, 'converged', false);
end

function result = top3Estimate(noisyRows, noisyRx, friisRange, minArea2)
    result = blankResult("top3_trilateration");
    [triangleIdx, ~] = selectBestTriangle(noisyRows, noisyRx, minArea2);
    if numel(triangleIdx) ~= 3
        return;
    end
    selectedRows = noisyRows(triangleIdx, :);
    selectedRanges3d = friisRange(noisyRx(triangleIdx));
    heightDelta = selectedRows.uav_z - selectedRows.sensor_z;
    ranges = sqrt(max(selectedRanges3d.^2 - heightDelta.^2, 0));
    p1 = [selectedRows.sensor_x(1), selectedRows.sensor_y(1)];
    p2 = [selectedRows.sensor_x(2), selectedRows.sensor_y(2)];
    p3 = [selectedRows.sensor_x(3), selectedRows.sensor_y(3)];
    [result.x, result.y] = trilaterate2d(p1, ranges(1), p2, ranges(2), p3, ranges(3));
    result.ok = isfinite(result.x) && isfinite(result.y);
    result.usedCount = 3;
end

function result = allSensorWlsEstimate(noisyRows, noisyRx, friisRange, sigmaDb, top3)
    result = blankResult("all_sensor_wls");
    if height(noisyRows) < 3 || ~hasNonCollinearGeometry(noisyRows.sensor_x, noisyRows.sensor_y)
        return;
    end
    [sensorPositions, ranges, rangeStd] = sensorInputs(noisyRows, noisyRx, friisRange, sigmaDb);
    weights = 1 ./ (rangeStd .^ 2);
    weights = weights / max(weights);
    initialPosition = initialEstimate(sensorPositions, weights, top3);
    [result.x, result.y, result.iterations, result.converged] = weightedLeastSquares2d( ...
        sensorPositions, ranges, weights, initialPosition);
    result.ok = isfinite(result.x) && isfinite(result.y);
    result.usedCount = height(noisyRows);
end

function result = robustWlsEstimate(noisyRows, noisyRx, friisRange, sigmaDb, top3)
    result = blankResult("robust_wls");
    if height(noisyRows) < 3 || ~hasNonCollinearGeometry(noisyRows.sensor_x, noisyRows.sensor_y)
        return;
    end
    [sensorPositions, ranges, rangeStd] = sensorInputs(noisyRows, noisyRx, friisRange, sigmaDb);
    weights = 1 ./ (rangeStd .^ 2);
    weights = weights / max(weights);
    initialPosition = initialEstimate(sensorPositions, weights, top3);
    [result.x, result.y, result.iterations, result.converged, usedMask, ...
        result.rejectedCount, result.maxNormResidual] = robustWeightedLeastSquares2d( ...
        sensorPositions, ranges, rangeStd, initialPosition);
    result.ok = isfinite(result.x) && isfinite(result.y);
    result.usedCount = sum(usedMask);
end

function [sensorPositions, ranges, rangeStd] = sensorInputs(noisyRows, noisyRx, friisRange, sigmaDb)
    sensorPositions = [noisyRows.sensor_x, noisyRows.sensor_y];
    ranges3d = friisRange(noisyRx);
    heightDelta = noisyRows.uav_z - noisyRows.sensor_z;
    ranges = sqrt(max(ranges3d.^2 - heightDelta.^2, 0));
    effectiveSigmaDb = max(sigmaDb, 0.25);
    rangeStd = max((log(10) / 20) * effectiveSigmaDb * ranges, 1.0);
end

function initialPosition = initialEstimate(sensorPositions, weights, top3)
    if top3.ok
        initialPosition = [top3.x, top3.y];
    else
        weights = weights(:) / sum(weights);
        initialPosition = [sum(sensorPositions(:, 1) .* weights), ...
                           sum(sensorPositions(:, 2) .* weights)];
    end
end

function [bestIdx, bestArea2] = selectBestTriangle(rows, noisyRx, minArea2)
    bestIdx = [];
    bestArea2 = NaN;
    if height(rows) < 3
        return;
    end
    [~, sortIdx] = sort(noisyRx, 'descend');
    sortIdx = sortIdx(1:min(numel(sortIdx), 8));
    combos = nchoosek(sortIdx, 3);
    for c = 1:size(combos, 1)
        idx = combos(c, :);
        area2 = triangleArea2(rows.sensor_x(idx), rows.sensor_y(idx));
        if area2 > minArea2
            bestIdx = idx;
            bestArea2 = area2;
            return;
        end
    end
end

function ok = hasNonCollinearGeometry(x, y)
    if numel(x) < 3
        ok = false;
        return;
    end
    centred = [x(:) - mean(x), y(:) - mean(y)];
    ok = rank(centred) >= 2;
end

function area2 = triangleArea2(x, y)
    area2 = abs((x(2) - x(1)) * (y(3) - y(1)) - ...
                (x(3) - x(1)) * (y(2) - y(1)));
end

function S = buildSummary(R, noiseSigmaDbList)
    summaryRows = {};
    summaryIndex = 1;
    seeds = unique(R.random_seed, 'stable');
    methods = unique(R.method, 'stable');
    summarySeeds = [0; seeds(:)];

    for s = 1:numel(noiseSigmaDbList)
        sigmaDb = noiseSigmaDbList(s);
        for m = 1:numel(methods)
            methodName = methods(m);
            for sd = 1:numel(summarySeeds)
                seedValue = summarySeeds(sd);
                mask = R.noise_sigma_db == sigmaDb & R.method == methodName;
                if seedValue ~= 0
                    mask = mask & R.random_seed == seedValue;
                end

                totalCases = sum(mask);
                localisedMask = mask & R.localised & isfinite(R.localisation_error_m);
                errors = R.localisation_error_m(localisedMask);
                rejected = R.rejected_sensor_count(localisedMask);
                if isempty(errors)
                    meanError = NaN; rmseError = NaN; medianError = NaN;
                    p95Error = NaN; maxError = NaN; meanRejected = NaN;
                else
                    meanError = mean(errors, 'omitnan');
                    rmseError = sqrt(mean(errors.^2, 'omitnan'));
                    medianError = median(errors, 'omitnan');
                    p95Error = percentileValue(errors, 95);
                    maxError = max(errors);
                    meanRejected = mean(rejected, 'omitnan');
                end
                localisedCases = numel(errors);
                successRate = 100 * localisedCases / max(totalCases, 1);
                summaryRows(summaryIndex, :) = {sigmaDb, methodName, seedValue, ...
                    totalCases, localisedCases, successRate, meanError, rmseError, ...
                    medianError, p95Error, maxError, meanRejected};
                summaryIndex = summaryIndex + 1;
            end
        end
    end

    S = cell2table(summaryRows, 'VariableNames', { ...
        'noise_sigma_db', 'method', 'random_seed', 'total_cases', ...
        'localised_cases', 'success_rate_pct', 'mean_localisation_error_m', ...
        'rmse_localisation_error_m', 'median_localisation_error_m', ...
        'p95_localisation_error_m', 'max_localisation_error_m', ...
        'mean_rejected_sensor_count'});
end

function value = percentileValue(values, percentile)
    values = sort(values(:));
    if isempty(values)
        value = NaN;
        return;
    end
    rankPosition = 1 + (numel(values) - 1) * percentile / 100;
    lowIndex = floor(rankPosition);
    highIndex = ceil(rankPosition);
    if lowIndex == highIndex
        value = values(lowIndex);
    else
        weight = rankPosition - lowIndex;
        value = values(lowIndex) * (1 - weight) + values(highIndex) * weight;
    end
end

function plotOverallRmse(S)
    paths = corpus_paths();
    figure; hold on; grid on;
    methods = unique(S.method, 'stable');
    for m = 1:numel(methods)
        rows = S(S.random_seed == 0 & S.method == methods(m), :);
        plot(rows.noise_sigma_db, rows.rmse_localisation_error_m, '-o', 'LineWidth', 1.5);
    end
    xlabel('RF noise standard deviation (dB)');
    ylabel('RMSE localisation error (m)');
    title('Random Waypoint Paths: Noisy Localisation RMSE');
    legend(methods, 'Location', 'northwest', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_random_paths_noise_rmse.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_random_paths_noise_rmse.png'), 'Resolution', 200);
end

function plotOverallP95(S)
    paths = corpus_paths();
    figure; hold on; grid on;
    methods = unique(S.method, 'stable');
    for m = 1:numel(methods)
        rows = S(S.random_seed == 0 & S.method == methods(m), :);
        plot(rows.noise_sigma_db, rows.p95_localisation_error_m, '-s', 'LineWidth', 1.5);
    end
    xlabel('RF noise standard deviation (dB)');
    ylabel('95th percentile localisation error (m)');
    title('Random Waypoint Paths: 95th Percentile Error');
    legend(methods, 'Location', 'northwest', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_random_paths_noise_p95.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_random_paths_noise_p95.png'), 'Resolution', 200);
end

function plotPerSeedWls(S)
    paths = corpus_paths();
    figure; hold on; grid on;
    seeds = unique(S.random_seed(S.random_seed ~= 0), 'stable');
    for sd = 1:numel(seeds)
        seedValue = seeds(sd);
        rows = S(S.method == "all_sensor_wls" & S.random_seed == seedValue, :);
        plot(rows.noise_sigma_db, rows.rmse_localisation_error_m, '-o', 'LineWidth', 1.4);
    end
    xlabel('RF noise standard deviation (dB)');
    ylabel('All-sensor WLS RMSE (m)');
    title('Random Waypoint Paths: Per-Seed WLS RMSE');
    legend("seed " + string(seeds), 'Location', 'northwest');
    savefig(fullfile(paths.figures, 'fence24_random_paths_per_seed_wls_rmse.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_random_paths_per_seed_wls_rmse.png'), 'Resolution', 200);
end
