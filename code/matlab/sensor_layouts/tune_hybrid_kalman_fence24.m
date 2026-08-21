clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% HYBRID FENCE24 + WEIGHTED OMNI + KALMAN TUNING:
% Reuses the existing ns-3/FlyNetSim random drone paths. No new ns-3 run is
% required. MATLAB simulates a hybrid sensor model over those paths:
%  - 16 directional fence sensors: RSSI + AoA
%  - 4 fence corners + 4 secure-wall sensors: omnidirectional RSSI only
% The experiment down-weights omni range measurements and tunes a constant
% velocity Kalman tracker to test whether availability can improve without
% losing too much localisation accuracy.

dataFiles = dir(fullfile(paths.rawUniform, 'prison_fence24_random_seed*_*.csv'));
if isempty(dataFiles)
    error('No uniform fence24 random-path CSV files found in the corpus data directories.');
end

outSummaryCsv = fullfile(paths.processed, 'hybrid_kalman_tuning_summary_refined.csv');
outBestCsv = fullfile(paths.processed, 'hybrid_kalman_best_validation_results_refined.csv');
outBestEstimateCsv = fullfile(paths.processed, 'hybrid_kalman_best_validation_estimates_refined.csv');

thresholdList = [-70];
rssiNoiseDbList = [1];
aoaNoiseDeg = 0.5;
omniWeightList = [0.01, 0.02, 0.05, 0.08, 0.10];
accelStdList = [0.1, 0.2, 0.5, 1];
measStdList = [3, 5, 8, 12, 20];
gateList = [20, 40, 80];
numTrials = 5;

trainSeeds = [1, 2, 3];
validationSeeds = [4, 5];
rng(2107);

frequencyHz = 2.4e9;
txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;

fprintf('\nHybrid weighted fusion + Kalman tuning starting.\n');
fprintf('Input files: %d uniform fence24 random-path CSVs\n', numel(dataFiles));
fprintf('Thresholds: %s dBm\n', mat2str(thresholdList));
fprintf('RSSI noise: %s dB, AoA noise: %.2f deg\n', mat2str(rssiNoiseDbList), aoaNoiseDeg);
fprintf('Omni weights: %s\n', mat2str(omniWeightList));
fprintf('Kalman accel std: %s, measurement std: %s, gates: %s m\n\n', ...
    mat2str(accelStdList), mat2str(measStdList), mat2str(gateList));

records = loadTrajectoryRecords(dataFiles);
records = records(ismember([records.seed], [trainSeeds, validationSeeds]));
sensorsTemplate = buildHybridFence24Sensors();

summaryRows = {};
summaryIndex = 1;
bestTrainScore = inf;
bestConfig = struct();
bestValidationEstimates = table();

for th = 1:numel(thresholdList)
    thresholdDbm = thresholdList(th);
    sensors = sensorsTemplate;
    sensors.threshold_dbm(:) = thresholdDbm;

    for rn = 1:numel(rssiNoiseDbList)
        rssiSigmaDb = rssiNoiseDbList(rn);

        for ow = 1:numel(omniWeightList)
            omniWeight = omniWeightList(ow);

            fprintf('Measurement set: threshold=%g dBm, RSSI=%g dB, AoA=%g deg, omniWeight=%g\n', ...
                thresholdDbm, rssiSigmaDb, aoaNoiseDeg, omniWeight);

            rawAllTrials = table();
            for trial = 1:numTrials
                rawTrial = simulateHybridRaw(records, sensors, thresholdDbm, rssiSigmaDb, aoaNoiseDeg, ...
                    omniWeight, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi, trial);
                rawAllTrials = [rawAllTrials; rawTrial]; %#ok<AGROW>
            end

            rawTrain = rawAllTrials(ismember(rawAllTrials.seed, trainSeeds), :);
            rawVal = rawAllTrials(ismember(rawAllTrials.seed, validationSeeds), :);
            trainRawMetrics = calculateMetrics(rawTrain, "raw");
            valRawMetrics = calculateMetrics(rawVal, "raw");

            summaryRows(summaryIndex, :) = metricsToRow("hybrid_weighted_raw", thresholdDbm, rssiSigmaDb, ...
                aoaNoiseDeg, omniWeight, NaN, NaN, NaN, trainRawMetrics, valRawMetrics);
            summaryIndex = summaryIndex + 1;

            for qa = 1:numel(accelStdList)
                accelStd = accelStdList(qa);
                for mr = 1:numel(measStdList)
                    measStd = measStdList(mr);
                    for gt = 1:numel(gateList)
                        gateM = gateList(gt);

                        kalmanAll = applyKalmanToTrials(rawAllTrials, accelStd, measStd, gateM);
                        kalmanTrain = kalmanAll(ismember(kalmanAll.seed, trainSeeds), :);
                        kalmanVal = kalmanAll(ismember(kalmanAll.seed, validationSeeds), :);
                        trainMetrics = calculateMetrics(kalmanTrain, "kalman");
                        valMetrics = calculateMetrics(kalmanVal, "kalman");

                        summaryRows(summaryIndex, :) = metricsToRow("hybrid_weighted_kalman", thresholdDbm, rssiSigmaDb, ...
                            aoaNoiseDeg, omniWeight, accelStd, measStd, gateM, trainMetrics, valMetrics);
                        summaryIndex = summaryIndex + 1;

                        % Training objective: prioritise accuracy, but softly reward
                        % availability so a low-error but almost-never-localising model
                        % does not win.
                        score = trainMetrics.rmse_error_m + 0.05 * trainMetrics.p95_error_m ...
                            - 0.03 * trainMetrics.success_rate_pct;

                        if isfinite(score) && score < bestTrainScore
                            bestTrainScore = score;
                            bestConfig.method = "hybrid_weighted_kalman";
                            bestConfig.threshold_dbm = thresholdDbm;
                            bestConfig.rssi_noise_db = rssiSigmaDb;
                            bestConfig.aoa_noise_deg = aoaNoiseDeg;
                            bestConfig.omni_weight = omniWeight;
                            bestConfig.kalman_accel_std = accelStd;
                            bestConfig.kalman_measurement_std = measStd;
                            bestConfig.kalman_gate_m = gateM;
                            bestConfig.train = trainMetrics;
                            bestConfig.validation = valMetrics;
                            bestValidationEstimates = kalmanVal;
                        end
                    end
                end
            end
        end
    end
end

summary = cell2table(summaryRows, 'VariableNames', { ...
    'method', 'threshold_dbm', 'rssi_noise_db', 'aoa_noise_deg', 'omni_weight', ...
    'kalman_accel_std', 'kalman_measurement_std', 'kalman_gate_m', ...
    'train_total_cases', 'train_localised_cases', 'train_success_rate_pct', ...
    'train_mean_error_m', 'train_rmse_error_m', 'train_median_error_m', 'train_p95_error_m', ...
    'train_pct_below_1m', 'train_pct_below_2m', 'train_pct_below_5m', 'train_mean_sensor_count', ...
    'validation_total_cases', 'validation_localised_cases', 'validation_success_rate_pct', ...
    'validation_mean_error_m', 'validation_rmse_error_m', 'validation_median_error_m', 'validation_p95_error_m', ...
    'validation_pct_below_1m', 'validation_pct_below_2m', 'validation_pct_below_5m', 'validation_mean_sensor_count'});
writetable(summary, outSummaryCsv);

best = struct2table(flattenBestConfig(bestConfig));
writetable(best, outBestCsv);
writetable(bestValidationEstimates, outBestEstimateCsv);

plotHybridTuningResults(summary, bestValidationEstimates, bestConfig);

fprintf('\nTuning summary saved: %s\n', outSummaryCsv);
fprintf('Best validation summary saved: %s\n', outBestCsv);
fprintf('Best validation estimates saved: %s\n', outBestEstimateCsv);
disp(best);

function records = loadTrajectoryRecords(dataFiles)
    records = struct([]);
    index = 1;
    for f = 1:numel(dataFiles)
        T = readtable(fullfile(dataFiles(f).folder, dataFiles(f).name), 'TextType', 'string');
        [groups, ~] = findgroups(T.random_seed, T.time);
        for g = 1:max(groups)
            rows = T(groups == g, :);
            records(index).seed = rows.random_seed(1);
            records(index).time = rows.time(1);
            records(index).x = rows.uav_x(1);
            records(index).y = rows.uav_y(1);
            records(index).z = rows.uav_z(1);
            records(index).zone = rows.zone(1);
            records(index).timeToBoundary = rows.time_to_boundary(1);
            index = index + 1;
        end
    end

    [~, order] = sortrows([[records.seed]', [records.time]']);
    records = records(order);
end

function sensors = buildHybridFence24Sensors()
    coords = [
        0 250 300 15
        1 250 350 15
        2 250 450 15
        3 250 550 15
        4 250 650 15
        5 250 700 15
        6 300 700 15
        7 400 700 15
        8 600 700 15
        9 700 700 15
        10 750 700 15
        11 750 650 15
        12 750 550 15
        13 750 450 15
        14 750 350 15
        15 750 300 15
        16 700 300 15
        17 600 300 15
        18 400 300 15
        19 300 300 15
        20 300 500 15
        21 500 350 15
        22 700 500 15
        23 500 650 15
    ];
    centre = [500, 500];
    sensorId = coords(:, 1);
    x = coords(:, 2);
    y = coords(:, 3);
    z = coords(:, 4);
    orientation = atan2d(y - centre(2), x - centre(1));
    fov = 120 * ones(size(sensorId));
    threshold = -65 * ones(size(sensorId));
    mountType = repmat("fence", size(sensorId));
    mountType(sensorId >= 20) = "wall";
    sensorType = repmat("directional", size(sensorId));
    omniIds = [0; 5; 10; 15; 20; 21; 22; 23];
    sensorType(ismember(sensorId, omniIds)) = "omni";
    fov(sensorType == "omni") = 360;
    sensors = table(sensorId, x, y, z, orientation, fov, threshold, mountType, sensorType, ...
        'VariableNames', {'sensor_id', 'x', 'y', 'z', 'orientation_deg', 'fov_deg', 'threshold_dbm', 'mount_type', 'sensor_type'});
end

function rawTable = simulateHybridRaw(records, sensors, thresholdDbm, rssiSigmaDb, aoaSigmaDeg, omniWeight, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi, trial)
    n = numel(records);
    rawTable = table();
    rawTable.seed = [records.seed]';
    rawTable.time = [records.time]';
    rawTable.trial = trial * ones(n, 1);
    rawTable.true_x = [records.x]';
    rawTable.true_y = [records.y]';
    rawTable.zone = string({records.zone})';
    rawTable.threshold_dbm = thresholdDbm * ones(n, 1);
    rawTable.rssi_noise_db = rssiSigmaDb * ones(n, 1);
    rawTable.aoa_noise_deg = aoaSigmaDeg * ones(n, 1);
    rawTable.omni_weight = omniWeight * ones(n, 1);
    rawTable.estimated_x = nan(n, 1);
    rawTable.estimated_y = nan(n, 1);
    rawTable.error_m = nan(n, 1);
    rawTable.ok = false(n, 1);
    rawTable.used_sensor_count = zeros(n, 1);
    rawTable.directional_count = zeros(n, 1);
    rawTable.omni_count = zeros(n, 1);

    for i = 1:n
        [x, y, ok, usedCount, directionalCount, omniCount] = hybridAoaRssiEstimate(records(i), sensors, ...
            rssiSigmaDb, aoaSigmaDeg, omniWeight, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
        rawTable.used_sensor_count(i) = usedCount;
        rawTable.directional_count(i) = directionalCount;
        rawTable.omni_count(i) = omniCount;
        rawTable.ok(i) = ok;
        if ok
            rawTable.estimated_x(i) = x;
            rawTable.estimated_y(i) = y;
            rawTable.error_m(i) = hypot(x - records(i).x, y - records(i).y);
        end
    end
end

function [estX, estY, ok, usedCount, directionalCount, omniCount] = hybridAoaRssiEstimate(record, sensors, rssiSigmaDb, aoaSigmaDeg, omniWeight, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi)
    uav = [record.x, record.y, record.z];
    numSensors = height(sensors);
    rxNoisy = zeros(numSensors, 1);
    measuredAoa = nan(numSensors, 1);
    detected = false(numSensors, 1);
    bearingCapable = false(numSensors, 1);

    for s = 1:numSensors
        sensorPos = [sensors.x(s), sensors.y(s), sensors.z(s)];
        distance = norm(uav - sensorPos);
        rxIdeal = friisRxPowerDbm(distance, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
        rxNoisy(s) = rxIdeal + rssiSigmaDb * randn();
        trueAoa = atan2d(record.y - sensors.y(s), record.x - sensors.x(s));

        if sensors.sensor_type(s) == "omni"
            inFov = true;
        else
            inFov = abs(wrap180(trueAoa - sensors.orientation_deg(s))) <= sensors.fov_deg(s) / 2;
            measuredAoa(s) = wrap180(trueAoa + aoaSigmaDeg * randn());
            bearingCapable(s) = true;
        end

        detected(s) = inFov && rxNoisy(s) >= sensors.threshold_dbm(s);
    end

    idx = find(detected);
    usedCount = numel(idx);
    directionalCount = sum(sensors.sensor_type(idx) == "directional");
    omniCount = sum(sensors.sensor_type(idx) == "omni");
    ok = false;
    estX = NaN;
    estY = NaN;

    hasEnoughGeometry = directionalCount >= 2 || usedCount >= 3 || (directionalCount >= 1 && usedCount >= 2);
    if ~hasEnoughGeometry
        return;
    end

    sensorXY = [sensors.x(idx), sensors.y(idx)];
    ranges = inverseFriisRange(rxNoisy(idx), frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
    rangeStd = max((log(10) / 20) * max(rssiSigmaDb, 0.1) .* ranges, 1);
    rangeWeightScale = ones(usedCount, 1);
    rangeWeightScale(sensors.sensor_type(idx) == "omni") = omniWeight;

    bearingMask = bearingCapable(idx);
    measuredBearings = measuredAoa(idx(bearingMask));
    bearingStdRad = deg2rad(max(aoaSigmaDeg, 0.1));

    if any(bearingMask)
        bearingRad = deg2rad(measuredAoa(idx(bearingMask)));
        bearingSensors = sensorXY(bearingMask, :);
        bearingRanges = ranges(bearingMask);
        initial = [mean(bearingSensors(:, 1) + bearingRanges .* cos(bearingRad)), ...
                   mean(bearingSensors(:, 2) + bearingRanges .* sin(bearingRad))];
    else
        initial = mean(sensorXY, 1);
    end

    objective = @(p) hybridObjective(p, sensorXY, ranges, rangeStd, rangeWeightScale, ...
        bearingMask, measuredBearings, bearingStdRad);
    options = optimset('Display', 'off', 'MaxIter', 80, 'TolX', 1e-5);
    estimate = fminsearch(objective, initial, options);

    estX = estimate(1);
    estY = estimate(2);
    ok = isfinite(estX) && isfinite(estY) && estX > -500 && estX < 1500 && estY > -500 && estY < 1500;
end

function value = hybridObjective(p, sensorXY, ranges, rangeStd, rangeWeightScale, bearingMask, measuredBearings, bearingStdRad)
    dx = p(1) - sensorXY(:, 1);
    dy = p(2) - sensorXY(:, 2);
    predictedRange = max(sqrt(dx.^2 + dy.^2), 1e-6);
    rangeResidual = (predictedRange - ranges) ./ rangeStd;
    value = sum((rangeWeightScale .* rangeResidual).^2);

    if any(bearingMask)
        predictedBearing = atan2d(dy(bearingMask), dx(bearingMask));
        bearingResidual = deg2rad(wrap180(predictedBearing - measuredBearings)) ./ bearingStdRad;
        value = value + sum(bearingResidual.^2);
    end
end

function kalmanTable = applyKalmanToTrials(rawTable, accelStd, measStd, gateM)
    kalmanTable = rawTable;
    kalmanTable.kalman_accel_std = accelStd * ones(height(rawTable), 1);
    kalmanTable.kalman_measurement_std = measStd * ones(height(rawTable), 1);
    kalmanTable.kalman_gate_m = gateM * ones(height(rawTable), 1);
    kalmanTable.measurement_update = false(height(rawTable), 1);

    kalmanTable.estimated_x(:) = NaN;
    kalmanTable.estimated_y(:) = NaN;
    kalmanTable.error_m(:) = NaN;
    kalmanTable.ok(:) = false;

    keys = unique(rawTable(:, {'seed', 'trial'}), 'rows');
    H = [1 0 0 0; 0 1 0 0];
    R = (measStd^2) * eye(2);

    for k = 1:height(keys)
        idx = find(rawTable.seed == keys.seed(k) & rawTable.trial == keys.trial(k));
        [~, order] = sort(rawTable.time(idx));
        idx = idx(order);

        state = zeros(4, 1);
        P = diag([100, 100, 25, 25]);
        initialised = false;
        lastTime = rawTable.time(idx(1));

        for ii = 1:numel(idx)
            row = idx(ii);
            if initialised
                dt = max(rawTable.time(row) - lastTime, 1);
                F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
                q = accelStd^2;
                Q = q * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
                state = F * state;
                P = F * P * F' + Q;
            end

            if rawTable.ok(row)
                z = [rawTable.estimated_x(row); rawTable.estimated_y(row)];
                if ~initialised
                    state = [z(1); z(2); 0; 0];
                    initialised = true;
                    kalmanTable.measurement_update(row) = true;
                else
                    residual = z - H * state;
                    if norm(residual) <= gateM
                        S = H * P * H' + R;
                        K = P * H' / S;
                        state = state + K * residual;
                        P = (eye(4) - K * H) * P;
                        kalmanTable.measurement_update(row) = true;
                    end
                end
            end

            if initialised
                kalmanTable.estimated_x(row) = state(1);
                kalmanTable.estimated_y(row) = state(2);
                kalmanTable.error_m(row) = hypot(state(1) - rawTable.true_x(row), state(2) - rawTable.true_y(row));
                kalmanTable.ok(row) = true;
            end
            lastTime = rawTable.time(row);
        end
    end
end

function metrics = calculateMetrics(T, estimateKind)
    if estimateKind == "kalman"
        okMask = T.ok;
    else
        okMask = T.ok;
    end
    errors = T.error_m(okMask & isfinite(T.error_m));
    metrics.total_cases = height(T);
    metrics.localised_cases = numel(errors);
    metrics.success_rate_pct = 100 * numel(errors) / max(height(T), 1);
    metrics.mean_error_m = meanOrNaN(errors);
    metrics.rmse_error_m = sqrt(meanOrNaN(errors.^2));
    metrics.median_error_m = medianOrNaN(errors);
    metrics.p95_error_m = percentileOrNaN(errors, 95);
    metrics.pct_below_1m = 100 * meanOrNaN(errors <= 1);
    metrics.pct_below_2m = 100 * meanOrNaN(errors <= 2);
    metrics.pct_below_5m = 100 * meanOrNaN(errors <= 5);
    metrics.mean_sensor_count = meanOrNaN(T.used_sensor_count);
end

function row = metricsToRow(method, thresholdDbm, rssiSigmaDb, aoaSigmaDeg, omniWeight, accelStd, measStd, gateM, trainMetrics, valMetrics)
    row = {char(method), thresholdDbm, rssiSigmaDb, aoaSigmaDeg, omniWeight, accelStd, measStd, gateM, ...
        trainMetrics.total_cases, trainMetrics.localised_cases, trainMetrics.success_rate_pct, ...
        trainMetrics.mean_error_m, trainMetrics.rmse_error_m, trainMetrics.median_error_m, trainMetrics.p95_error_m, ...
        trainMetrics.pct_below_1m, trainMetrics.pct_below_2m, trainMetrics.pct_below_5m, trainMetrics.mean_sensor_count, ...
        valMetrics.total_cases, valMetrics.localised_cases, valMetrics.success_rate_pct, ...
        valMetrics.mean_error_m, valMetrics.rmse_error_m, valMetrics.median_error_m, valMetrics.p95_error_m, ...
        valMetrics.pct_below_1m, valMetrics.pct_below_2m, valMetrics.pct_below_5m, valMetrics.mean_sensor_count};
end

function flat = flattenBestConfig(bestConfig)
    flat = struct();
    flat.method = bestConfig.method;
    flat.threshold_dbm = bestConfig.threshold_dbm;
    flat.rssi_noise_db = bestConfig.rssi_noise_db;
    flat.aoa_noise_deg = bestConfig.aoa_noise_deg;
    flat.omni_weight = bestConfig.omni_weight;
    flat.kalman_accel_std = bestConfig.kalman_accel_std;
    flat.kalman_measurement_std = bestConfig.kalman_measurement_std;
    flat.kalman_gate_m = bestConfig.kalman_gate_m;
    flat.train_success_rate_pct = bestConfig.train.success_rate_pct;
    flat.train_mean_error_m = bestConfig.train.mean_error_m;
    flat.train_rmse_error_m = bestConfig.train.rmse_error_m;
    flat.train_p95_error_m = bestConfig.train.p95_error_m;
    flat.validation_success_rate_pct = bestConfig.validation.success_rate_pct;
    flat.validation_mean_error_m = bestConfig.validation.mean_error_m;
    flat.validation_rmse_error_m = bestConfig.validation.rmse_error_m;
    flat.validation_p95_error_m = bestConfig.validation.p95_error_m;
    flat.validation_pct_below_1m = bestConfig.validation.pct_below_1m;
    flat.validation_pct_below_2m = bestConfig.validation.pct_below_2m;
    flat.validation_pct_below_5m = bestConfig.validation.pct_below_5m;
end

function plotHybridTuningResults(summary, bestEstimates, bestConfig)
    paths = corpus_paths();
    rmsePng = fullfile(paths.figures, 'hybrid_kalman_validation_rmse_by_threshold.png');
    availabilityPng = fullfile(paths.figures, 'hybrid_kalman_validation_success_by_threshold.png');
    pathPng = fullfile(paths.figures, 'hybrid_kalman_best_validation_path.png');

    kalmanRows = summary(strcmp(summary.method, 'hybrid_weighted_kalman'), :);
    thresholds = unique(kalmanRows.threshold_dbm, 'stable');
    rssiVals = unique(kalmanRows.rssi_noise_db, 'stable');
    bestRmse = nan(numel(thresholds), numel(rssiVals));
    bestSuccess = nan(numel(thresholds), numel(rssiVals));

    for i = 1:numel(thresholds)
        for j = 1:numel(rssiVals)
            rows = kalmanRows(kalmanRows.threshold_dbm == thresholds(i) & kalmanRows.rssi_noise_db == rssiVals(j), :);
            [~, idx] = min(rows.validation_rmse_error_m);
            bestRmse(i, j) = rows.validation_rmse_error_m(idx);
            bestSuccess(i, j) = rows.validation_success_rate_pct(idx);
        end
    end

    figure('Position', [100 100 820 460]);
    bar(categorical(string(thresholds)), bestRmse);
    grid on;
    xlabel('Uniform threshold (dBm)');
    ylabel('Best validation RMSE (m)');
    title('Hybrid Kalman Tuning: Validation RMSE');
    legend(compose('%g dB RSSI noise', rssiVals), 'Location', 'bestoutside');
    exportgraphics(gcf, rmsePng, 'Resolution', 200);

    figure('Position', [100 100 820 460]);
    bar(categorical(string(thresholds)), bestSuccess);
    grid on;
    xlabel('Uniform threshold (dBm)');
    ylabel('Validation tracked estimate availability (%)');
    title('Hybrid Kalman Tuning: Validation Availability');
    legend(compose('%g dB RSSI noise', rssiVals), 'Location', 'bestoutside');
    exportgraphics(gcf, availabilityPng, 'Resolution', 200);

    seedRows = bestEstimates(bestEstimates.seed == 4 & bestEstimates.trial == 1, :);
    if isempty(seedRows)
        seedRows = bestEstimates(bestEstimates.trial == bestEstimates.trial(1), :);
    end
    figure('Position', [100 100 980 560]);
    plot(seedRows.true_x, seedRows.true_y, 'b-', 'LineWidth', 1.5); hold on; grid on; axis equal;
    valid = seedRows.ok & isfinite(seedRows.estimated_x);
    scatter(seedRows.estimated_x(valid), seedRows.estimated_y(valid), 30, seedRows.error_m(valid), 'filled');
    rectangle('Position', [250 300 500 400], 'EdgeColor', [0.45 0.45 0.45], 'LineStyle', '--', 'LineWidth', 1.2);
    rectangle('Position', [300 350 400 300], 'EdgeColor', 'k', 'LineWidth', 2);
    xlim([0 900]); ylim([120 760]); colorbar;
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('Best Hybrid Kalman Validation Path: th=%g dBm, omni weight=%.2f', ...
        bestConfig.threshold_dbm, bestConfig.omni_weight));
    exportgraphics(gcf, pathPng, 'Resolution', 200);
end

function rxPower = friisRxPowerDbm(distance, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi)
    safeDistance = max(distance, 1);
    pathLossDb = 20 * log10(safeDistance) + 20 * log10(frequencyHz) - 147.55;
    rxPower = txPowerDbm + txGainDbi + rxGainDbi - pathLossDb;
end

function range = inverseFriisRange(rxPowerDbm, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi)
    range = 10.^((txPowerDbm + txGainDbi + rxGainDbi - rxPowerDbm ...
        - 20 * log10(frequencyHz) + 147.55) / 20);
end

function angle = wrap180(angle)
    angle = mod(angle + 180, 360) - 180;
end

function value = meanOrNaN(x)
    if isempty(x)
        value = NaN;
    else
        value = mean(x);
    end
end

function value = medianOrNaN(x)
    if isempty(x)
        value = NaN;
    else
        value = median(x);
    end
end

function value = percentileOrNaN(x, p)
    if isempty(x)
        value = NaN;
    else
        value = prctile(x, p);
    end
end
