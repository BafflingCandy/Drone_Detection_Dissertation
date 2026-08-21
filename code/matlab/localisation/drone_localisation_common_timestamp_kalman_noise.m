clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% COMMON-TIMESTAMP PROFILE COMPARISON:
% Compares uniform and tiered_wall only at the same seed/time/UAV positions.
% This separates an availability claim from an accuracy claim:
%   - tiered_wall availability is measured by how many more timestamps it can localise,
%   - common-timestamp accuracy is measured only where both profiles have usable
%     clean geometry before noise is added.
%
% The same RSSI noise vector is applied to both profiles at each matched
% timestamp/trial. This keeps the comparison fair: the intentional difference
% is the threshold profile, not a different random noise sample.

uniformFiles = dir(fullfile(paths.rawUniform, 'prison_fence24_random_seed*_*.csv'));
tieredFiles = dir(fullfile(paths.rawTiered, 'prison_fence24_tiered_wall_random_seed*_*.csv'));
if isempty(uniformFiles) || isempty(tieredFiles)
    error('Uniform and tiered_wall random seed CSV files are both required in the corpus data directories.');
end

noiseSigmaDbList = [1, 2, 3, 5, 7];
numTrials = 100;
minArea2 = 1.0;
rng(505);

txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
frequencyHz = 2.4e9;
friisRange = @(rxPowerDbm) 10.^((txPowerDbm + txGainDbi + rxGainDbi ...
    - rxPowerDbm - 20*log10(frequencyHz) + 147.55) / 20);

uniformRecords = buildRecords(uniformFiles, "uniform");
tieredRecords = buildRecords(tieredFiles, "tiered_wall");
[pairs, allCommonCount] = buildCommonPairs(uniformRecords, tieredRecords);

if isempty(pairs)
    error('No common clean-localisation-ready timestamps were found.');
end

fprintf('\nCommon-timestamp Kalman comparison starting.\n');
fprintf('Uniform CSV files: %d\n', numel(uniformFiles));
fprintf('Tiered-wall CSV files: %d\n', numel(tieredFiles));
fprintf('Common seed/time/UAV positions before clean-geometry filtering: %d\n', allCommonCount);
fprintf('Common clean-localisation-ready timestamps used: %d\n', numel(pairs));
fprintf('Noise levels: %s dB\n', mat2str(noiseSigmaDbList));
fprintf('Trials per noise level: %d\n\n', numTrials);

profiles = ["uniform", "tiered_wall"];
methods = ["all_sensor_wls", "kalman_wls"];
seeds = unique([pairs.seed], 'stable');
resultRows = {};
resultIndex = 1;

for s = 1:numel(noiseSigmaDbList)
    sigmaDb = noiseSigmaDbList(s);
    fprintf('Sigma=%g dB\n', sigmaDb);

    stats = initialiseStats(profiles, methods, seeds);

    for trial = 1:numTrials
        rawUniform = repmat(blankEstimate(), numel(pairs), 1);
        rawTiered = repmat(blankEstimate(), numel(pairs), 1);

        for i = 1:numel(pairs)
            uniformRec = pairs(i).uniform;
            tieredRec = pairs(i).tiered;

            sharedNoise = sigmaDb * randn(numel(uniformRec.rxPowerDbm), 1);
            uniformNoisyRx = uniformRec.rxPowerDbm + sharedNoise;
            tieredNoisyRx = tieredRec.rxPowerDbm + sharedNoise;

            uniformMask = uniformNoisyRx >= uniformRec.thresholdDbm;
            tieredMask = tieredNoisyRx >= tieredRec.thresholdDbm;

            top3Uniform = top3Estimate(uniformRec, uniformNoisyRx, uniformMask, friisRange, minArea2);
            top3Tiered = top3Estimate(tieredRec, tieredNoisyRx, tieredMask, friisRange, minArea2);
            rawUniform(i) = allSensorWlsEstimate(uniformRec, uniformNoisyRx, uniformMask, friisRange, sigmaDb, top3Uniform);
            rawTiered(i) = allSensorWlsEstimate(tieredRec, tieredNoisyRx, tieredMask, friisRange, sigmaDb, top3Tiered);
            rawUniform(i).noisyCount = sum(uniformMask);
            rawTiered(i).noisyCount = sum(tieredMask);
        end

        kalmanUniform = applyKalmanByRun(pairs, rawUniform, "uniform", sigmaDb);
        kalmanTiered = applyKalmanByRun(pairs, rawTiered, "tiered_wall", sigmaDb);

        stats = addTrialStats(stats, pairs, "uniform", "all_sensor_wls", rawUniform);
        stats = addTrialStats(stats, pairs, "uniform", "kalman_wls", kalmanUniform);
        stats = addTrialStats(stats, pairs, "tiered_wall", "all_sensor_wls", rawTiered);
        stats = addTrialStats(stats, pairs, "tiered_wall", "kalman_wls", kalmanTiered);
    end

    summarySeeds = [0, seeds];
    for p = 1:numel(profiles)
        for m = 1:numel(methods)
            for sd = 1:numel(summarySeeds)
                seedValue = summarySeeds(sd);
                values = collectStats(stats, profiles(p), methods(m), seedValue, seeds);
                [meanError, rmseError, medianError, p95Error, maxError] = errorStats(values.errors);
                localisedCases = numel(values.errors);
                successRate = 100 * localisedCases / max(values.totalCases, 1);
                resultRows(resultIndex, :) = {profiles(p), sigmaDb, methods(m), seedValue, ...
                    values.totalCases, localisedCases, successRate, meanError, rmseError, ...
                    medianError, p95Error, maxError, mean(values.noisyCounts, 'omitnan'), ...
                    mean(values.usedCounts, 'omitnan')};
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

summaryOutput = fullfile(paths.processed, 'prison_fence24_common_timestamp_kalman_noise_summary.csv');
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

function records = buildRecords(files, profileName)
    records = containers.Map('KeyType', 'char', 'ValueType', 'any');
    for f = 1:numel(files)
        path = fullfile(files(f).folder, files(f).name);
        T = readtable(path, 'TextType', 'string');
        if ~ismember('sensor_profile', T.Properties.VariableNames)
            T.sensor_profile = repmat(profileName, height(T), 1);
        end

        [groupIds, ~] = findgroups(T.time);
        for g = 1:max(groupIds)
            rows = T(groupIds == g, :);
            key = makeKey(rows.random_seed(1), rows.time(1));
            detected = rows(rows.detected == 1, :);
            cleanReady = height(detected) >= 3 && hasNonCollinearGeometry(detected.sensor_x, detected.sensor_y);

            rec = struct();
            rec.profile = char(profileName);
            rec.seed = rows.random_seed(1);
            rec.time = rows.time(1);
            rec.uavX = rows.uav_x(1);
            rec.uavY = rows.uav_y(1);
            rec.uavZ = rows.uav_z(1);
            rec.sensorX = rows.sensor_x;
            rec.sensorY = rows.sensor_y;
            rec.sensorZ = rows.sensor_z;
            rec.rxPowerDbm = rows.rx_power_dbm;
            rec.thresholdDbm = rows.threshold_dbm;
            rec.cleanReady = cleanReady;
            records(key) = rec;
        end
    end
end

function [pairs, allCommonCount] = buildCommonPairs(uniformRecords, tieredRecords)
    uniformKeys = string(keys(uniformRecords));
    tieredKeys = string(keys(tieredRecords));
    commonKeys = intersect(uniformKeys, tieredKeys, 'stable');
    allCommonCount = numel(commonKeys);
    pairs = struct([]);
    pairIndex = 1;

    for i = 1:numel(commonKeys)
        key = char(commonKeys(i));
        u = uniformRecords(key);
        t = tieredRecords(key);
        samePosition = abs(u.uavX - t.uavX) < 0.01 && ...
                       abs(u.uavY - t.uavY) < 0.01 && ...
                       abs(u.uavZ - t.uavZ) < 0.01;
        if samePosition && u.cleanReady && t.cleanReady
            pairs(pairIndex).seed = u.seed;
            pairs(pairIndex).time = u.time;
            pairs(pairIndex).uniform = u;
            pairs(pairIndex).tiered = t;
            pairIndex = pairIndex + 1;
        end
    end
end

function stats = initialiseStats(profiles, methods, seeds)
    stats = containers.Map('KeyType', 'char', 'ValueType', 'any');
    for p = 1:numel(profiles)
        for m = 1:numel(methods)
            for sd = 1:numel(seeds)
                emptyStats = struct('totalCases', 0, 'errors', [], 'noisyCounts', [], 'usedCounts', []);
                stats(statsKey(profiles(p), methods(m), seeds(sd))) = emptyStats;
            end
        end
    end
end

function stats = addTrialStats(stats, pairs, profileName, methodName, estimates)
    for i = 1:numel(pairs)
        key = statsKey(profileName, methodName, pairs(i).seed);
        values = stats(key);
        values.totalCases = values.totalCases + 1;
        values.noisyCounts = [values.noisyCounts; estimates(i).noisyCount];

        if estimates(i).ok
            rec = pairs(i).(profileField(profileName));
            values.errors = [values.errors; hypot(estimates(i).x - rec.uavX, estimates(i).y - rec.uavY)];
            values.usedCounts = [values.usedCounts; estimates(i).usedCount];
        end
        stats(key) = values;
    end
end

function values = collectStats(stats, profileName, methodName, seedValue, seeds)
    values = struct('totalCases', 0, 'errors', [], 'noisyCounts', [], 'usedCounts', []);
    if seedValue ~= 0
        values = stats(statsKey(profileName, methodName, seedValue));
        return;
    end
    for i = 1:numel(seeds)
        seedValues = stats(statsKey(profileName, methodName, seeds(i)));
        values.totalCases = values.totalCases + seedValues.totalCases;
        values.errors = [values.errors; seedValues.errors]; %#ok<AGROW>
        values.noisyCounts = [values.noisyCounts; seedValues.noisyCounts]; %#ok<AGROW>
        values.usedCounts = [values.usedCounts; seedValues.usedCounts]; %#ok<AGROW>
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

function kalmanEstimates = applyKalmanByRun(pairs, rawEstimates, profileName, sigmaDb)
    kalmanEstimates = rawEstimates;
    for i = 1:numel(kalmanEstimates)
        kalmanEstimates(i).ok = false;
        kalmanEstimates(i).x = NaN;
        kalmanEstimates(i).y = NaN;
    end

    seeds = unique([pairs.seed], 'stable');
    for s = 1:numel(seeds)
        idx = find([pairs.seed] == seeds(s));
        [~, order] = sort([pairs(idx).time]);
        idx = idx(order);
        state = [];
        P = [];
        previousTime = NaN;

        for k = 1:numel(idx)
            currentIndex = idx(k);
            rec = pairs(currentIndex).(profileField(profileName));
            dt = 1.0;
            if isfinite(previousTime)
                dt = max(rec.time - previousTime, 0.1);
            end

            if isempty(state)
                if rawEstimates(currentIndex).ok
                    state = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y; 0; 0];
                    P = diag([100, 100, 25, 25]);
                else
                    previousTime = rec.time;
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
            previousTime = rec.time;
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

function key = makeKey(seed, time)
    key = sprintf('%g|%.6f', seed, time);
end

function key = statsKey(profileName, methodName, seed)
    key = char(string(profileName) + "|" + string(methodName) + "|" + string(seed));
end

function fieldName = profileField(profileName)
    if string(profileName) == "tiered_wall"
        fieldName = "tiered";
    else
        fieldName = "uniform";
    end
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
    title('Common Timestamp: WLS vs Kalman RMSE');
    legend(labels, 'Location', 'northwest', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_common_timestamp_kalman_rmse.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_common_timestamp_kalman_rmse.png'), 'Resolution', 200);
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
    title('Common Timestamp: WLS vs Kalman 95th Percentile Error');
    legend(labels, 'Location', 'northwest', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_common_timestamp_kalman_p95.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_common_timestamp_kalman_p95.png'), 'Resolution', 200);
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
    title('Common Timestamp: Mean Noisy Detection Count');
    legend(profiles, 'Location', 'best', 'Interpreter', 'none');
    savefig(fullfile(paths.figures, 'fence24_common_timestamp_kalman_sensor_count.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'fence24_common_timestamp_kalman_sensor_count.png'), 'Resolution', 200);
end
