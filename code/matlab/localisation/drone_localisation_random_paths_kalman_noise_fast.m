clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% FAST RANDOM-WAYPOINT KALMAN NOISE LOCALISATION:
% Summary-focused version for comparing uniform and tiered_wall profiles.
% It uses the same modelling logic as the detailed script:
%   1. add Gaussian RSSI noise in dB,
%   2. convert noisy RSSI to range using inverse Friis,
%   3. estimate position with all-sensor WLS,
%   4. smooth WLS estimates with a constant-velocity Kalman filter.
% This version stores summary statistics instead of every Monte Carlo row.

fileInfo = [dir(fullfile(paths.rawUniform, 'prison_fence24_random_seed*_*.csv')); ...
    dir(fullfile(paths.rawTiered, 'prison_fence24_tiered_wall_random_seed*_*.csv'))];
if isempty(fileInfo)
    error('No random-path CSV files found in the corpus data directories.');
end

noiseSigmaDbList = [1, 2, 3, 5, 7];
numTrials = 20;  % Fast review run; increase to 100 for the final dissertation batch.
minArea2 = 1.0;
rng(404);

txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
frequencyHz = 2.4e9;
friisRange = @(rxPowerDbm) 10.^((txPowerDbm + txGainDbi + rxGainDbi ...
    - rxPowerDbm - 20*log10(frequencyHz) + 147.55) / 20);

records = buildBaseRecords(fileInfo);
if isempty(records)
    error('No clean localisation-ready records found.');
end

profiles = unique(string({records.profile}), 'stable');
seeds = unique([records.seed], 'stable');
methods = ["all_sensor_wls", "kalman_wls"];

fprintf('\nFast Kalman noise localisation starting.\n');
fprintf('Input CSV files: %d\n', numel(fileInfo));
fprintf('Clean localisation-ready timestamps: %d\n', numel(records));
fprintf('Profiles: %s\n', strjoin(profiles, ', '));
fprintf('Noise levels: %s dB\n', mat2str(noiseSigmaDbList));
fprintf('Trials per noise level: %d\n\n', numTrials);

resultRows = {};
resultIndex = 1;

for p = 1:numel(profiles)
    profileName = profiles(p);
    profileIdx = find(string({records.profile}) == profileName);
    profileRecords = records(profileIdx);

    for s = 1:numel(noiseSigmaDbList)
        sigmaDb = noiseSigmaDbList(s);
        fprintf('Profile=%s, sigma=%g dB\n', profileName, sigmaDb);

        rawErrorsBySeed = containers.Map('KeyType', 'char', 'ValueType', 'any');
        kalmanErrorsBySeed = containers.Map('KeyType', 'char', 'ValueType', 'any');
        rawTotalBySeed = containers.Map('KeyType', 'char', 'ValueType', 'double');
        kalmanTotalBySeed = containers.Map('KeyType', 'char', 'ValueType', 'double');
        noisyCountsBySeed = containers.Map('KeyType', 'char', 'ValueType', 'any');
        usedCountsBySeed = containers.Map('KeyType', 'char', 'ValueType', 'any');

        for sd = 1:numel(seeds)
            seedKey = seedToKey(seeds(sd));
            rawErrorsBySeed(seedKey) = [];
            kalmanErrorsBySeed(seedKey) = [];
            rawTotalBySeed(seedKey) = 0;
            kalmanTotalBySeed(seedKey) = 0;
            noisyCountsBySeed(seedKey) = [];
            usedCountsBySeed(seedKey) = [];
        end

        for trial = 1:numTrials
            rawEstimates = repmat(blankEstimate(), numel(profileRecords), 1);
            for i = 1:numel(profileRecords)
                rec = profileRecords(i);
                noisyRx = rec.rxPowerDbm + sigmaDb * randn(numel(rec.rxPowerDbm), 1);
                detectedMask = noisyRx >= rec.thresholdDbm;

                top3 = top3Estimate(rec, noisyRx, detectedMask, friisRange, minArea2);
                rawEstimates(i) = allSensorWlsEstimate(rec, noisyRx, detectedMask, friisRange, sigmaDb, top3);
                rawEstimates(i).noisyCount = sum(detectedMask);
            end

            kalmanEstimates = applyKalmanByRun(profileRecords, rawEstimates, sigmaDb);

            for i = 1:numel(profileRecords)
                rec = profileRecords(i);
                seedKey = seedToKey(rec.seed);

                rawTotalBySeed(seedKey) = rawTotalBySeed(seedKey) + 1;
                kalmanTotalBySeed(seedKey) = kalmanTotalBySeed(seedKey) + 1;
                noisyCountsBySeed(seedKey) = [noisyCountsBySeed(seedKey); rawEstimates(i).noisyCount];

                if rawEstimates(i).ok
                    rawError = hypot(rawEstimates(i).x - rec.uavX, rawEstimates(i).y - rec.uavY);
                    rawErrorsBySeed(seedKey) = [rawErrorsBySeed(seedKey); rawError];
                    usedCountsBySeed(seedKey) = [usedCountsBySeed(seedKey); rawEstimates(i).usedCount];
                end

                if kalmanEstimates(i).ok
                    kalmanError = hypot(kalmanEstimates(i).x - rec.uavX, kalmanEstimates(i).y - rec.uavY);
                    kalmanErrorsBySeed(seedKey) = [kalmanErrorsBySeed(seedKey); kalmanError];
                end
            end
        end

        summarySeeds = [0, seeds];
        for m = 1:numel(methods)
            methodName = methods(m);
            for sd = 1:numel(summarySeeds)
                seedValue = summarySeeds(sd);
                if seedValue == 0
                    totalCases = 0;
                    errors = [];
                    noisyCounts = [];
                    usedCounts = [];
                    for k = 1:numel(seeds)
                        seedKey = seedToKey(seeds(k));
                        if methodName == "all_sensor_wls"
                            totalCases = totalCases + rawTotalBySeed(seedKey);
                            errors = [errors; rawErrorsBySeed(seedKey)]; %#ok<AGROW>
                        else
                            totalCases = totalCases + kalmanTotalBySeed(seedKey);
                            errors = [errors; kalmanErrorsBySeed(seedKey)]; %#ok<AGROW>
                        end
                        noisyCounts = [noisyCounts; noisyCountsBySeed(seedKey)]; %#ok<AGROW>
                        usedCounts = [usedCounts; usedCountsBySeed(seedKey)]; %#ok<AGROW>
                    end
                else
                    seedKey = seedToKey(seedValue);
                    if methodName == "all_sensor_wls"
                        totalCases = rawTotalBySeed(seedKey);
                        errors = rawErrorsBySeed(seedKey);
                    else
                        totalCases = kalmanTotalBySeed(seedKey);
                        errors = kalmanErrorsBySeed(seedKey);
                    end
                    noisyCounts = noisyCountsBySeed(seedKey);
                    usedCounts = usedCountsBySeed(seedKey);
                end

                [meanError, rmseError, medianError, p95Error, maxError] = errorStats(errors);
                localisedCases = numel(errors);
                successRate = 100 * localisedCases / max(totalCases, 1);
                meanNoisyCount = mean(noisyCounts, 'omitnan');
                meanUsedCount = mean(usedCounts, 'omitnan');

                resultRows(resultIndex, :) = {profileName, sigmaDb, methodName, seedValue, ...
                    totalCases, localisedCases, successRate, meanError, rmseError, ...
                    medianError, p95Error, maxError, meanNoisyCount, meanUsedCount};
                resultIndex = resultIndex + 1;
            end
        end
    end
end

S = cell2table(resultRows, 'VariableNames', { ...
    'sensor_profile', 'noise_sigma_db', 'method', 'random_seed', ...
    'total_cases', 'localised_cases', 'success_rate_pct', ...
    'mean_localisation_error_m', 'rmse_localisation_error_m', ...
    'median_localisation_error_m', 'p95_localisation_error_m', ...
    'max_localisation_error_m', 'mean_noisy_sensor_count', ...
    'mean_used_sensor_count'});

summaryOutput = fullfile(paths.processed, 'prison_fence24_random_paths_kalman_noise_fast_summary.csv');
writetable(S, summaryOutput);

overall = S(S.random_seed == 0, :);
disp(overall(:, {'sensor_profile', 'noise_sigma_db', 'method', ...
    'total_cases', 'localised_cases', 'success_rate_pct', ...
    'mean_localisation_error_m', 'rmse_localisation_error_m', ...
    'p95_localisation_error_m', 'mean_noisy_sensor_count'}));

plotOverallRmse(S);
plotOverallP95(S);
plotMeanSensorCount(S);

fprintf('\nSummary output: %s\n', summaryOutput);

function records = buildBaseRecords(fileInfo)
    records = struct([]);
    index = 1;
    for f = 1:numel(fileInfo)
        path = fullfile(fileInfo(f).folder, fileInfo(f).name);
        T = readtable(path, 'TextType', 'string');
        [~, fileName, fileExt] = fileparts(path);
        if ~ismember('sensor_profile', T.Properties.VariableNames)
            T.sensor_profile = repmat("uniform", height(T), 1);
        end
        T.experiment_file = repmat(string([fileName fileExt]), height(T), 1);

        [groupIds, ~] = findgroups(T.time);
        for g = 1:max(groupIds)
            rows = T(groupIds == g, :);
            cleanDetected = rows(rows.detected == 1, :);
            if height(cleanDetected) < 3 || ~hasNonCollinearGeometry(cleanDetected.sensor_x, cleanDetected.sensor_y)
                continue;
            end

            records(index).file = char(rows.experiment_file(1));
            records(index).profile = char(rows.sensor_profile(1));
            records(index).seed = rows.random_seed(1);
            records(index).time = rows.time(1);
            records(index).uavX = rows.uav_x(1);
            records(index).uavY = rows.uav_y(1);
            records(index).uavZ = rows.uav_z(1);
            records(index).sensorX = rows.sensor_x;
            records(index).sensorY = rows.sensor_y;
            records(index).sensorZ = rows.sensor_z;
            records(index).rxPowerDbm = rows.rx_power_dbm;
            records(index).thresholdDbm = rows.threshold_dbm;
            index = index + 1;
        end
    end
end

function result = blankEstimate()
    result = struct('ok', false, 'x', NaN, 'y', NaN, ...
        'usedCount', 0, 'noisyCount', 0, 'converged', false);
end

function result = top3Estimate(rec, noisyRx, detectedMask, friisRange, minArea2)
    result = blankEstimate();
    detectedIdx = find(detectedMask);
    if numel(detectedIdx) < 3
        return;
    end
    [~, sortOrder] = sort(noisyRx(detectedIdx), 'descend');
    candidateIdx = detectedIdx(sortOrder(1:min(numel(sortOrder), 8)));
    if numel(candidateIdx) < 3
        return;
    end
    combos = nchoosek(candidateIdx, 3);
    chosen = [];
    for c = 1:size(combos, 1)
        idx = combos(c, :);
        area2 = triangleArea2(rec.sensorX(idx), rec.sensorY(idx));
        if area2 > minArea2
            chosen = idx;
            break;
        end
    end
    if numel(chosen) ~= 3
        return;
    end
    ranges3d = friisRange(noisyRx(chosen));
    heightDelta = rec.uavZ - rec.sensorZ(chosen);
    ranges = sqrt(max(ranges3d.^2 - heightDelta.^2, 0));
    p1 = [rec.sensorX(chosen(1)), rec.sensorY(chosen(1))];
    p2 = [rec.sensorX(chosen(2)), rec.sensorY(chosen(2))];
    p3 = [rec.sensorX(chosen(3)), rec.sensorY(chosen(3))];
    [result.x, result.y] = trilaterate2d(p1, ranges(1), p2, ranges(2), p3, ranges(3));
    result.ok = isfinite(result.x) && isfinite(result.y);
    result.usedCount = 3;
end

function result = allSensorWlsEstimate(rec, noisyRx, detectedMask, friisRange, sigmaDb, top3)
    result = blankEstimate();
    idx = find(detectedMask);
    result.noisyCount = numel(idx);
    if numel(idx) < 3 || ~hasNonCollinearGeometry(rec.sensorX(idx), rec.sensorY(idx))
        return;
    end
    sensorPositions = [rec.sensorX(idx), rec.sensorY(idx)];
    ranges3d = friisRange(noisyRx(idx));
    heightDelta = rec.uavZ - rec.sensorZ(idx);
    ranges = sqrt(max(ranges3d.^2 - heightDelta.^2, 0));
    effectiveSigmaDb = max(sigmaDb, 0.25);
    rangeStd = max((log(10) / 20) * effectiveSigmaDb * ranges, 1.0);
    weights = 1 ./ (rangeStd .^ 2);
    weights = weights / max(weights);

    if top3.ok
        initialPosition = [top3.x, top3.y];
    else
        w = weights(:) / sum(weights);
        initialPosition = [sum(sensorPositions(:, 1) .* w), sum(sensorPositions(:, 2) .* w)];
    end

    [result.x, result.y, ~, result.converged] = weightedLeastSquares2d( ...
        sensorPositions, ranges, weights, initialPosition);
    result.ok = isfinite(result.x) && isfinite(result.y);
    result.usedCount = numel(idx);
end

function kalmanEstimates = applyKalmanByRun(records, rawEstimates, sigmaDb)
    kalmanEstimates = rawEstimates;
    for i = 1:numel(kalmanEstimates)
        kalmanEstimates(i).ok = false;
        kalmanEstimates(i).x = NaN;
        kalmanEstimates(i).y = NaN;
    end

    runKeys = strings(numel(records), 1);
    for i = 1:numel(records)
        runKeys(i) = string(records(i).file) + "|" + string(records(i).profile) + "|" + string(records(i).seed);
    end
    uniqueRuns = unique(runKeys, 'stable');

    for r = 1:numel(uniqueRuns)
        idx = find(runKeys == uniqueRuns(r));
        [~, order] = sort([records(idx).time]);
        idx = idx(order);
        state = [];
        P = [];
        previousTime = NaN;

        for k = 1:numel(idx)
            currentIndex = idx(k);
            t = records(currentIndex).time;
            dt = 1.0;
            if isfinite(previousTime)
                dt = max(t - previousTime, 0.1);
            end

            if isempty(state)
                if rawEstimates(currentIndex).ok
                    state = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y; 0; 0];
                    P = diag([100, 100, 25, 25]);
                else
                    previousTime = t;
                    continue;
                end
            else
                F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
                q = max(2.0, sigmaDb);
                Q = q^2 * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; ...
                           dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
                state = F * state;
                P = F * P * F' + Q;
            end

            if rawEstimates(currentIndex).ok
                z = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y];
                H = [1 0 0 0; 0 1 0 0];
                measurementSigma = max(5.0, 8.0 * sigmaDb);
                R = measurementSigma^2 * eye(2);
                K = P * H' / (H * P * H' + R);
                state = state + K * (z - H * state);
                P = (eye(4) - K * H) * P;
            end

            kalmanEstimates(currentIndex).ok = isfinite(state(1)) && isfinite(state(2));
            kalmanEstimates(currentIndex).x = state(1);
            kalmanEstimates(currentIndex).y = state(2);
            kalmanEstimates(currentIndex).usedCount = rawEstimates(currentIndex).usedCount;
            kalmanEstimates(currentIndex).noisyCount = rawEstimates(currentIndex).noisyCount;
            previousTime = t;
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

function key = seedToKey(seed)
    key = sprintf('%g', seed);
end

function [meanError, rmseError, medianError, p95Error, maxError] = errorStats(errors)
    if isempty(errors)
        meanError = NaN; rmseError = NaN; medianError = NaN; p95Error = NaN; maxError = NaN;
        return;
    end
    meanError = mean(errors, 'omitnan');
    rmseError = sqrt(mean(errors.^2, 'omitnan'));
    medianError = median(errors, 'omitnan');
    p95Error = percentileValue(errors, 95);
    maxError = max(errors);
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
    labels = strings(0, 1);
    profiles = unique(S.sensor_profile, 'stable');
    methods = unique(S.method, 'stable');
    for p = 1:numel(profiles)
        for m = 1:numel(methods)
            rows = S(S.random_seed == 0 & S.sensor_profile == profiles(p) & S.method == methods(m), :);
            plot(rows.noise_sigma_db, rows.rmse_localisation_error_m, '-o', 'LineWidth', 1.5);
            labels(end + 1) = profiles(p) + " / " + methods(m); %#ok<AGROW>
        end
    end
    xlabel('RF noise standard deviation (dB)');
    ylabel('RMSE localisation error (m)');
    title('Fast Random Waypoint: WLS vs Kalman RMSE');
    legend(labels, 'Location', 'northwest', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_random_paths_kalman_fast_rmse.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_random_paths_kalman_fast_rmse.png'), 'Resolution', 200);
end

function plotOverallP95(S)
    paths = corpus_paths();
    figure; hold on; grid on;
    labels = strings(0, 1);
    profiles = unique(S.sensor_profile, 'stable');
    methods = unique(S.method, 'stable');
    for p = 1:numel(profiles)
        for m = 1:numel(methods)
            rows = S(S.random_seed == 0 & S.sensor_profile == profiles(p) & S.method == methods(m), :);
            plot(rows.noise_sigma_db, rows.p95_localisation_error_m, '-s', 'LineWidth', 1.5);
            labels(end + 1) = profiles(p) + " / " + methods(m); %#ok<AGROW>
        end
    end
    xlabel('RF noise standard deviation (dB)');
    ylabel('95th percentile localisation error (m)');
    title('Fast Random Waypoint: WLS vs Kalman 95th Percentile Error');
    legend(labels, 'Location', 'northwest', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_random_paths_kalman_fast_p95.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_random_paths_kalman_fast_p95.png'), 'Resolution', 200);
end

function plotMeanSensorCount(S)
    paths = corpus_paths();
    figure; hold on; grid on;
    profiles = unique(S.sensor_profile, 'stable');
    for p = 1:numel(profiles)
        rows = S(S.random_seed == 0 & S.sensor_profile == profiles(p) & S.method == "all_sensor_wls", :);
        plot(rows.noise_sigma_db, rows.mean_noisy_sensor_count, '-d', 'LineWidth', 1.5);
    end
    xlabel('RF noise standard deviation (dB)');
    ylabel('Mean sensors above threshold');
    title('Fast Random Waypoint: Mean Noisy Detection Count');
    legend(profiles, 'Location', 'best', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_random_paths_kalman_fast_sensor_count.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_random_paths_kalman_fast_sensor_count.png'), 'Resolution', 200);
end
