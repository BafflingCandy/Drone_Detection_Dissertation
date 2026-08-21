clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% AOA/RSSI SENSOR OPTIMISATION + KALMAN EXPERIMENT:
% This is the advanced MATLAB-only experiment. ns-3/FlyNetSim still provides
% repeatable drone trajectories, while MATLAB tests a directional RF sensing
% model with AoA, RSSI, optimised placement, and Kalman smoothing.

dataFiles = dir(fullfile(paths.rawUniform, 'prison_fence24_random_seed*_*.csv'));
if isempty(dataFiles)
    error('No uniform random-path CSV files found in the corpus data directories.');
end

% Facility geometry copied from the ns-3 prison sensor scenario.
prisonMinX = 300; prisonMaxX = 700;
prisonMinY = 350; prisonMaxY = 650;
clearZoneM = 50;
fenceMinX = prisonMinX - clearZoneM; fenceMaxX = prisonMaxX + clearZoneM;
fenceMinY = prisonMinY - clearZoneM; fenceMaxY = prisonMaxY + clearZoneM;
facilityCentre = [500, 500];

numSensorsToSelect = 24;
candidateSpacingM = 50;
sensorHeightM = 15;
rfThresholdDbm = -65;
fieldOfViewDeg = 90;
numTrials = 20;  % Review run. Increase after the method is accepted.
rssiNoiseDbList = [1, 2, 3, 5];
aoaNoiseDegList = [0.5, 1, 2, 5];
rng(606);

txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
frequencyHz = 2.4e9;

fprintf('\nAoA/RSSI optimisation experiment starting.\n');
fprintf('Candidate spacing: %g m\n', candidateSpacingM);
fprintf('Selected sensors: %d\n', numSensorsToSelect);
fprintf('Directional FOV: %g degrees\n', fieldOfViewDeg);
fprintf('RSSI threshold: %g dBm\n', rfThresholdDbm);
fprintf('Monte Carlo trials: %d\n\n', numTrials);

records = loadTrajectoryRecords(dataFiles);
candidates = buildCandidateSensors(fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
    prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, candidateSpacingM, ...
    sensorHeightM, facilityCentre, fieldOfViewDeg, rfThresholdDbm);

selectedIdx = optimiseGreedyLayout(candidates, records, numSensorsToSelect, frequencyHz, ...
    txPowerDbm, txGainDbi, rxGainDbi);
sensors = candidates(selectedIdx, :);
sensors.sensor_id = (0:height(sensors)-1)';

layoutOutput = fullfile(paths.layouts, 'aoa_opt_sensor_layout.csv');
writetable(sensors, layoutOutput);

fprintf('Candidate sensors: %d\n', height(candidates));
fprintf('Optimised layout saved: %s\n\n', layoutOutput);

plotOptimisedLayout(sensors, candidates, records, prisonMinX, prisonMaxX, prisonMinY, ...
    prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY);

resultRows = {};
resultIndex = 1;

for rn = 1:numel(rssiNoiseDbList)
    rssiSigmaDb = rssiNoiseDbList(rn);
    for an = 1:numel(aoaNoiseDegList)
        aoaSigmaDeg = aoaNoiseDegList(an);
        fprintf('Testing RSSI sigma=%g dB, AoA sigma=%g deg\n', rssiSigmaDb, aoaSigmaDeg);

        rawErrors = [];
        kalmanErrors = [];
        rawSensorCounts = [];
        rawTotalCases = 0;
        kalmanTotalCases = 0;

        for trial = 1:numTrials
            rawEstimates = repmat(blankEstimate(), numel(records), 1);

            for i = 1:numel(records)
                rec = records(i);
                rawEstimates(i) = hybridAoaRssiEstimate(rec, sensors, rssiSigmaDb, ...
                    aoaSigmaDeg, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
            end

            kalmanEstimates = applyKalman(records, rawEstimates, rssiSigmaDb, aoaSigmaDeg);

            for i = 1:numel(records)
                rawTotalCases = rawTotalCases + 1;
                kalmanTotalCases = kalmanTotalCases + 1;
                rawSensorCounts = [rawSensorCounts; rawEstimates(i).usedCount]; %#ok<AGROW>

                if rawEstimates(i).ok
                    rawErrors = [rawErrors; hypot(rawEstimates(i).x - records(i).x, ...
                        rawEstimates(i).y - records(i).y)]; %#ok<AGROW>
                end

                if kalmanEstimates(i).ok
                    kalmanErrors = [kalmanErrors; hypot(kalmanEstimates(i).x - records(i).x, ...
                        kalmanEstimates(i).y - records(i).y)]; %#ok<AGROW>
                end
            end
        end

        resultRows(resultIndex, :) = summaryRow("hybrid_aoa_rssi", rssiSigmaDb, aoaSigmaDeg, ...
            rawTotalCases, rawErrors, rawSensorCounts);
        resultIndex = resultIndex + 1;
        resultRows(resultIndex, :) = summaryRow("hybrid_aoa_rssi_kalman", rssiSigmaDb, aoaSigmaDeg, ...
            kalmanTotalCases, kalmanErrors, rawSensorCounts);
        resultIndex = resultIndex + 1;
    end
end

summary = cell2table(resultRows, 'VariableNames', { ...
    'method', 'rssi_noise_db', 'aoa_noise_deg', 'total_cases', ...
    'localised_cases', 'success_rate_pct', 'mean_error_m', 'rmse_error_m', ...
    'median_error_m', 'p95_error_m', 'max_error_m', ...
    'pct_below_1m', 'pct_below_2m', 'pct_below_5m', 'pct_below_10m', ...
    'mean_sensor_count'});

summaryOutput = fullfile(paths.processed, 'aoa_rssi_kalman_summary.csv');
writetable(summary, summaryOutput);

disp(summary);
fprintf('\nSummary saved: %s\n', summaryOutput);

plotHeatmap(summary, "hybrid_aoa_rssi_kalman", "rmse_error_m", ...
    'Kalman Hybrid AoA/RSSI RMSE (m)', fullfile(paths.figures, 'aoa_opt_kalman_rmse.png'));
plotHeatmap(summary, "hybrid_aoa_rssi_kalman", "p95_error_m", ...
    'Kalman Hybrid AoA/RSSI 95th Percentile Error (m)', fullfile(paths.figures, 'aoa_opt_kalman_p95.png'));
plotHeatmap(summary, "hybrid_aoa_rssi_kalman", "pct_below_1m", ...
    'Kalman Hybrid AoA/RSSI Estimates Below 1 m (%)', fullfile(paths.figures, 'aoa_opt_kalman_below_1m.png'));

function records = loadTrajectoryRecords(dataFiles)
    records = struct([]);
    index = 1;
    for f = 1:numel(dataFiles)
        path = fullfile(dataFiles(f).folder, dataFiles(f).name);
        T = readtable(path, 'TextType', 'string');
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

    % Remove duplicate stationary target samples because they overweight the
    % final parked position after the drone reaches the vulnerable point.
    keep = true(numel(records), 1);
    for i = 2:numel(records)
        sameSeed = records(i).seed == records(i-1).seed;
        samePosition = hypot(records(i).x - records(i-1).x, records(i).y - records(i-1).y) < 0.01;
        if sameSeed && samePosition && records(i).time > 80
            keep(i) = false;
        end
    end
    records = records(keep);
end

function candidates = buildCandidateSensors(fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
    prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, spacing, heightM, centre, fovDeg, thresholdDbm)

    rows = [];
    types = strings(0, 1);

    % Outer fence candidates look inward toward the facility/clear zone.
    for x = fenceMinX:spacing:fenceMaxX
        [rows, types] = addCandidate(rows, types, x, fenceMinY, heightM, "fence", centre);
        [rows, types] = addCandidate(rows, types, x, fenceMaxY, heightM, "fence", centre);
    end
    for y = (fenceMinY + spacing):spacing:(fenceMaxY - spacing)
        [rows, types] = addCandidate(rows, types, fenceMinX, y, heightM, "fence", centre);
        [rows, types] = addCandidate(rows, types, fenceMaxX, y, heightM, "fence", centre);
    end

    % Secure-wall candidates look outward into the clear zone, because their
    % role is supporting approaching-drone localisation before wall crossing.
    for x = prisonMinX:spacing:prisonMaxX
        [rows, types] = addCandidate(rows, types, x, prisonMinY, heightM, "wall", [x, prisonMinY - 100]);
        [rows, types] = addCandidate(rows, types, x, prisonMaxY, heightM, "wall", [x, prisonMaxY + 100]);
    end
    for y = (prisonMinY + spacing):spacing:(prisonMaxY - spacing)
        [rows, types] = addCandidate(rows, types, prisonMinX, y, heightM, "wall", [prisonMinX - 100, y]);
        [rows, types] = addCandidate(rows, types, prisonMaxX, y, heightM, "wall", [prisonMaxX + 100, y]);
    end

    [uniqueRows, uniqueIdx] = unique(rows(:, 1:2), 'rows', 'stable');
    rows = rows(uniqueIdx, :);
    types = types(uniqueIdx);

    orientation = atan2d(rows(:, 5) - rows(:, 2), rows(:, 4) - rows(:, 1));
    candidates = table(uniqueRows(:, 1), uniqueRows(:, 2), rows(:, 3), orientation, ...
        repmat(fovDeg, size(rows, 1), 1), repmat(thresholdDbm, size(rows, 1), 1), ...
        types, 'VariableNames', {'x', 'y', 'z', 'orientation_deg', ...
        'fov_deg', 'threshold_dbm', 'mount_type'});
end

function [rows, types] = addCandidate(rows, types, x, y, z, mountType, target)
    rows = [rows; x, y, z, target(1), target(2)]; %#ok<AGROW>
    types(end + 1, 1) = mountType; %#ok<AGROW>
end

function selectedIdx = optimiseGreedyLayout(candidates, records, numSensors, frequencyHz, ...
    txPowerDbm, txGainDbi, rxGainDbi)

    selectedIdx = [];
    remainingIdx = 1:height(candidates);
    for k = 1:numSensors
        bestScore = -Inf;
        bestCandidate = NaN;

        for c = 1:numel(remainingIdx)
            testIdx = [selectedIdx, remainingIdx(c)];
            score = layoutScore(candidates(testIdx, :), records, frequencyHz, ...
                txPowerDbm, txGainDbi, rxGainDbi);
            if score > bestScore
                bestScore = score;
                bestCandidate = remainingIdx(c);
            end
        end

        selectedIdx = [selectedIdx, bestCandidate]; %#ok<AGROW>
        remainingIdx = setdiff(remainingIdx, bestCandidate, 'stable');
        fprintf('Selected sensor %02d/%02d, score %.2f\n', k, numSensors, bestScore);
    end
end

function score = layoutScore(sensors, records, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi)
    score = 0;
    for i = 1:numel(records)
        rec = records(i);
        [insideCone, rxPower, bearings] = deterministicSensorView(rec, sensors, frequencyHz, ...
            txPowerDbm, txGainDbi, rxGainDbi);
        detected = insideCone & rxPower >= sensors.threshold_dbm;
        count = sum(detected);
        if count >= 2
            geometry = bearingGeometryScore(bearings(detected));
            margin = mean(min(max(rxPower(detected) - sensors.threshold_dbm(detected), 0), 20));
            score = score + 80 + 12 * min(count, 6) + 40 * geometry + 1.5 * margin;
        else
            score = score + 8 * count;
        end
    end
end

function [insideCone, rxPower, bearings] = deterministicSensorView(rec, sensors, frequencyHz, ...
    txPowerDbm, txGainDbi, rxGainDbi)

    dx = rec.x - sensors.x;
    dy = rec.y - sensors.y;
    dz = rec.z - sensors.z;
    distance = sqrt(dx.^2 + dy.^2 + dz.^2);
    bearings = atan2d(dy, dx);
    angleError = wrap180(bearings - sensors.orientation_deg);
    insideCone = abs(angleError) <= sensors.fov_deg / 2;
    rxPower = friisRxPower(distance, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
end

function result = hybridAoaRssiEstimate(rec, sensors, rssiSigmaDb, aoaSigmaDeg, frequencyHz, ...
    txPowerDbm, txGainDbi, rxGainDbi)

    result = blankEstimate();
    [insideCone, trueRx, trueBearings] = deterministicSensorView(rec, sensors, frequencyHz, ...
        txPowerDbm, txGainDbi, rxGainDbi);
    noisyRx = trueRx + rssiSigmaDb * randn(height(sensors), 1);
    detected = insideCone & noisyRx >= sensors.threshold_dbm;
    idx = find(detected);
    result.usedCount = numel(idx);

    if numel(idx) < 2
        return;
    end

    measuredBearings = trueBearings(idx) + aoaSigmaDeg * randn(numel(idx), 1);
    ranges3d = inverseFriisRange(noisyRx(idx), frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
    heightDelta = rec.z - sensors.z(idx);
    ranges = sqrt(max(ranges3d.^2 - heightDelta.^2, 0));
    sensorXY = [sensors.x(idx), sensors.y(idx)];

    initial = bearingLeastSquares(sensorXY, measuredBearings);
    if any(~isfinite(initial))
        initial = mean(sensorXY, 1);
    end

    rangeStd = max((log(10) / 20) * max(rssiSigmaDb, 0.25) * ranges, 1.0);
    bearingStdRad = deg2rad(max(aoaSigmaDeg, 0.1));

    objective = @(p) hybridObjective(p, sensorXY, ranges, rangeStd, measuredBearings, bearingStdRad);
    options = optimset('Display', 'off', 'MaxIter', 120, 'TolX', 1e-4, 'TolFun', 1e-4);
    estimate = fminsearch(objective, initial, options);

    result.x = estimate(1);
    result.y = estimate(2);
    result.ok = all(isfinite(estimate));
end

function value = hybridObjective(p, sensorXY, ranges, rangeStd, measuredBearings, bearingStdRad)
    dx = p(1) - sensorXY(:, 1);
    dy = p(2) - sensorXY(:, 2);
    predictedRange = sqrt(dx.^2 + dy.^2);
    predictedBearing = atan2d(dy, dx);
    rangeResidual = (predictedRange - ranges) ./ rangeStd;
    bearingResidual = deg2rad(wrap180(predictedBearing - measuredBearings)) ./ bearingStdRad;
    value = sum(rangeResidual.^2) + sum(bearingResidual.^2);
end

function initial = bearingLeastSquares(sensorXY, bearingsDeg)
    A = zeros(numel(bearingsDeg), 2);
    b = zeros(numel(bearingsDeg), 1);
    for i = 1:numel(bearingsDeg)
        theta = deg2rad(bearingsDeg(i));
        normal = [-sin(theta), cos(theta)];
        A(i, :) = normal;
        b(i) = normal * sensorXY(i, :)';
    end
    if rank(A) < 2
        initial = [NaN, NaN];
    else
        initial = (A \ b)';
    end
end

function kalmanEstimates = applyKalman(records, rawEstimates, rssiSigmaDb, aoaSigmaDeg)
    kalmanEstimates = rawEstimates;
    for i = 1:numel(kalmanEstimates)
        kalmanEstimates(i).ok = false;
        kalmanEstimates(i).x = NaN;
        kalmanEstimates(i).y = NaN;
    end

    seeds = unique([records.seed], 'stable');
    for s = 1:numel(seeds)
        idx = find([records.seed] == seeds(s));
        [~, order] = sort([records(idx).time]);
        idx = idx(order);
        state = [];
        P = [];
        previousTime = NaN;

        for k = 1:numel(idx)
            currentIndex = idx(k);
            rec = records(currentIndex);
            dt = 1;
            if isfinite(previousTime)
                dt = max(rec.time - previousTime, 0.1);
            end

            if isempty(state)
                if rawEstimates(currentIndex).ok
                    state = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y; 0; 0];
                    P = diag([25, 25, 10, 10]);
                else
                    previousTime = rec.time;
                    continue;
                end
            else
                F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
                q = max(0.5, 0.4 * rssiSigmaDb + 0.8 * aoaSigmaDeg);
                Q = q^2 * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; ...
                           dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
                state = F * state;
                P = F * P * F' + Q;
            end

            if rawEstimates(currentIndex).ok
                z = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y];
                H = [1 0 0 0; 0 1 0 0];
                measurementSigma = max(1.0, 4.0 * aoaSigmaDeg + 1.5 * rssiSigmaDb);
                R = measurementSigma^2 * eye(2);
                K = P * H' / (H * P * H' + R);
                state = state + K * (z - H * state);
                P = (eye(4) - K * H) * P;
            end

            kalmanEstimates(currentIndex).x = state(1);
            kalmanEstimates(currentIndex).y = state(2);
            kalmanEstimates(currentIndex).ok = isfinite(state(1)) && isfinite(state(2));
            kalmanEstimates(currentIndex).usedCount = rawEstimates(currentIndex).usedCount;
            previousTime = rec.time;
        end
    end
end

function result = blankEstimate()
    result = struct('ok', false, 'x', NaN, 'y', NaN, 'usedCount', 0);
end

function row = summaryRow(method, rssiSigmaDb, aoaSigmaDeg, totalCases, errors, sensorCounts)
    localisedCases = numel(errors);
    [meanError, rmseError, medianError, p95Error, maxError] = errorStats(errors);
    row = {method, rssiSigmaDb, aoaSigmaDeg, totalCases, localisedCases, ...
        100 * localisedCases / max(totalCases, 1), meanError, rmseError, ...
        medianError, p95Error, maxError, percentBelow(errors, 1), ...
        percentBelow(errors, 2), percentBelow(errors, 5), percentBelow(errors, 10), ...
        mean(sensorCounts, 'omitnan')};
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

function pct = percentBelow(errors, threshold)
    if isempty(errors)
        pct = NaN;
    else
        pct = 100 * sum(errors <= threshold) / numel(errors);
    end
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

function rxPower = friisRxPower(distance, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi)
    safeDistance = max(distance, 1.0);
    pathLossDb = 20 * log10(safeDistance) + 20 * log10(frequencyHz) - 147.55;
    rxPower = txPowerDbm + txGainDbi + rxGainDbi - pathLossDb;
end

function range = inverseFriisRange(rxPowerDbm, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi)
    range = 10.^((txPowerDbm + txGainDbi + rxGainDbi - rxPowerDbm ...
        - 20 * log10(frequencyHz) + 147.55) / 20);
end

function score = bearingGeometryScore(bearingsDeg)
    if numel(bearingsDeg) < 2
        score = 0;
        return;
    end
    best = 0;
    for i = 1:numel(bearingsDeg)
        for j = i+1:numel(bearingsDeg)
            separation = abs(wrap180(bearingsDeg(i) - bearingsDeg(j)));
            best = max(best, abs(sind(separation)));
        end
    end
    score = best;
end

function angle = wrap180(angle)
    angle = mod(angle + 180, 360) - 180;
end

function plotOptimisedLayout(sensors, candidates, records, prisonMinX, prisonMaxX, prisonMinY, ...
    prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY)
    paths = corpus_paths();

    figure; hold on; axis equal; grid on;
    rectangle('Position', [fenceMinX, fenceMinY, fenceMaxX - fenceMinX, fenceMaxY - fenceMinY], ...
        'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.2, 'LineStyle', '--');
    rectangle('Position', [prisonMinX, prisonMinY, prisonMaxX - prisonMinX, prisonMaxY - prisonMinY], ...
        'EdgeColor', 'k', 'LineWidth', 2);
    scatter(candidates.x, candidates.y, 18, [0.75 0.75 0.75], 'filled');
    scatter(sensors.x, sensors.y, 55, 'r', 'filled');
    scatter([records.x], [records.y], 8, [0.2 0.45 0.85], 'filled', 'MarkerFaceAlpha', 0.25);

    coneLen = 45;
    for i = 1:height(sensors)
        quiver(sensors.x(i), sensors.y(i), coneLen * cosd(sensors.orientation_deg(i)), ...
            coneLen * sind(sensors.orientation_deg(i)), 0, 'Color', [0.8 0 0], 'LineWidth', 1);
    end

    xlabel('x (m)'); ylabel('y (m)');
    title('Optimised Directional AoA/RSSI Sensor Layout');
    legend({'Outer fence', 'Secure wall', 'Candidates', 'Selected sensors', 'Drone path samples'}, ...
        'Location', 'bestoutside');
    savefig(fullfile(paths.figures, 'aoa_opt_sensor_layout.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'aoa_opt_sensor_layout.png'), 'Resolution', 200);
end

function plotHeatmap(summary, methodName, metricName, titleText, outputPng)
    rows = summary(summary.method == methodName, :);
    rssiVals = unique(rows.rssi_noise_db, 'stable');
    aoaVals = unique(rows.aoa_noise_deg, 'stable');
    values = nan(numel(aoaVals), numel(rssiVals));
    for i = 1:numel(aoaVals)
        for j = 1:numel(rssiVals)
            match = rows(rows.aoa_noise_deg == aoaVals(i) & rows.rssi_noise_db == rssiVals(j), :);
            values(i, j) = match.(metricName)(1);
        end
    end

    figure;
    imagesc(rssiVals, aoaVals, values);
    set(gca, 'YDir', 'normal');
    colorbar;
    xlabel('RSSI noise sigma (dB)');
    ylabel('AoA noise sigma (degrees)');
    title(titleText);
    for i = 1:numel(aoaVals)
        for j = 1:numel(rssiVals)
            text(rssiVals(j), aoaVals(i), sprintf('%.1f', values(i, j)), ...
                'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
        end
    end
    [folder, name, ~] = fileparts(outputPng);
    savefig(fullfile(folder, [name '.fig']));
    exportgraphics(gcf, outputPng, 'Resolution', 200);
end
