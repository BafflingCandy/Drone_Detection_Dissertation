clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% FIXED FENCE24 DIRECTIONAL AOA/RSSI + KALMAN EXPERIMENT:
% Uses the physical fence/wall sensor layout developed in the dissertation,
% rather than allowing MATLAB to move the sensors. This answers the question:
% "If the intended fence24 layout is made directional, does AoA/RSSI + Kalman
% improve localisation accuracy before the drone enters the facility?"

dataFiles = dir(fullfile(paths.rawUniform, 'prison_fence24_random_seed*_*.csv'));
if isempty(dataFiles)
    error('No uniform random-path CSV files found in the corpus data directories.');
end

rssiNoiseDbList = [1, 2, 3, 5];
aoaNoiseDegList = [0.5, 1, 2, 5];
perimeterFovDegList = [90, 120];
secureWallFovDeg = 120;
sensorModeList = ["directional_all", "hybrid_corner_wall_omni"];
numTrials = 15;  % Review run. Increase to 50 or 100 for a final batch.
rng(707);

txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
frequencyHz = 2.4e9;
sensorThresholdDbm = -65;

fprintf('\nFixed fence24 directional AoA/RSSI experiment starting.\n');
fprintf('Trials: %d\n', numTrials);
fprintf('Threshold: %g dBm\n', sensorThresholdDbm);
fprintf('Perimeter FOV values: %s degrees\n', mat2str(perimeterFovDegList));
fprintf('Secure-wall FOV: %g degrees\n', secureWallFovDeg);
fprintf('Sensor modes: %s\n', strjoin(sensorModeList, ', '));
fprintf('RSSI noise: %s dB\n', mat2str(rssiNoiseDbList));
fprintf('AoA noise: %s degrees\n\n', mat2str(aoaNoiseDegList));

records = loadTrajectoryRecords(dataFiles);
baseSensors = buildFence24DirectionalSensors(sensorThresholdDbm, secureWallFovDeg);
layoutSensors = configureSensorMode(baseSensors, "hybrid_corner_wall_omni", 120, secureWallFovDeg);

layoutOutput = fullfile(paths.layouts, 'aoa_fence24_sensor_layout.csv');
writetable(layoutSensors, layoutOutput);
plotFence24Layout(layoutSensors, records);

resultRows = {};
resultIndex = 1;

for sm = 1:numel(sensorModeList)
    sensorMode = sensorModeList(sm);
    for fv = 1:numel(perimeterFovDegList)
        perimeterFovDeg = perimeterFovDegList(fv);
        sensors = configureSensorMode(baseSensors, sensorMode, perimeterFovDeg, secureWallFovDeg);

        for rn = 1:numel(rssiNoiseDbList)
            rssiSigmaDb = rssiNoiseDbList(rn);
            for an = 1:numel(aoaNoiseDegList)
                aoaSigmaDeg = aoaNoiseDegList(an);
                fprintf('Testing mode=%s, perimeter FOV=%g deg, wall FOV=%g deg, RSSI=%g dB, AoA=%g deg\n', ...
                    sensorMode, perimeterFovDeg, secureWallFovDeg, rssiSigmaDb, aoaSigmaDeg);

                rawErrors = [];
                kalmanErrors = [];
                rawSensorCounts = [];
                rawTotalCases = 0;
                kalmanTotalCases = 0;

                for trial = 1:numTrials
                    rawEstimates = repmat(blankEstimate(), numel(records), 1);

                    for i = 1:numel(records)
                        rawEstimates(i) = hybridAoaRssiEstimate(records(i), sensors, ...
                            rssiSigmaDb, aoaSigmaDeg, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
                    end

                    kalmanEstimates = applyResidualGatedKalman(records, rawEstimates, ...
                        rssiSigmaDb, aoaSigmaDeg);

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

                resultRows(resultIndex, :) = summaryRow(sensorMode, "fence24_aoa_rssi", ...
                    perimeterFovDeg, secureWallFovDeg, rssiSigmaDb, aoaSigmaDeg, rawTotalCases, rawErrors, rawSensorCounts);
                resultIndex = resultIndex + 1;
                resultRows(resultIndex, :) = summaryRow(sensorMode, "fence24_aoa_rssi_gated_kalman", ...
                    perimeterFovDeg, secureWallFovDeg, rssiSigmaDb, aoaSigmaDeg, kalmanTotalCases, kalmanErrors, rawSensorCounts);
                resultIndex = resultIndex + 1;
            end
        end
    end
end

summary = cell2table(resultRows, 'VariableNames', { ...
    'sensor_mode', 'method', 'perimeter_fov_deg', 'wall_fov_deg', 'rssi_noise_db', 'aoa_noise_deg', ...
    'total_cases', 'localised_cases', 'success_rate_pct', ...
    'mean_error_m', 'rmse_error_m', 'median_error_m', 'p95_error_m', ...
    'max_error_m', 'pct_below_1m', 'pct_below_2m', 'pct_below_5m', ...
    'pct_below_10m', 'mean_sensor_count'});

summaryOutput = fullfile(paths.processed, 'aoa_fence24_kalman_summary.csv');
writetable(summary, summaryOutput);

disp(summary);
fprintf('\nLayout saved: %s\n', layoutOutput);
fprintf('Summary saved: %s\n', summaryOutput);

plotHeatmap(summary, "hybrid_corner_wall_omni", "fence24_aoa_rssi", 90, "mean_error_m", ...
    'Hybrid Fence24 AoA/RSSI Mean Error, 90 deg FOV', ...
    fullfile(paths.figures, 'aoa_fence24_mean_90.png'));
plotHeatmap(summary, "hybrid_corner_wall_omni", "fence24_aoa_rssi_gated_kalman", 90, "mean_error_m", ...
    'Hybrid Fence24 AoA/RSSI Gated Kalman Mean Error, 90 deg FOV', ...
    fullfile(paths.figures, 'aoa_fence24_kalman_mean_90.png'));
plotHeatmap(summary, "hybrid_corner_wall_omni", "fence24_aoa_rssi", 120, "mean_error_m", ...
    'Hybrid Fence24 AoA/RSSI Mean Error, 120 deg FOV', ...
    fullfile(paths.figures, 'aoa_fence24_mean_120.png'));
plotHeatmap(summary, "hybrid_corner_wall_omni", "fence24_aoa_rssi_gated_kalman", 120, "mean_error_m", ...
    'Hybrid Fence24 AoA/RSSI Gated Kalman Mean Error, 120 deg FOV', ...
    fullfile(paths.figures, 'aoa_fence24_kalman_mean_120.png'));

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

    keep = true(numel(records), 1);
    for i = 2:numel(records)
        sameSeed = records(i).seed == records(i-1).seed;
        samePosition = hypot(records(i).x - records(i-1).x, records(i).y - records(i-1).y) < 0.01;
        if sameSeed && samePosition && records(i).time > 80
            keep(i) = false;
        end
    end
    records = records(keep);

    % Outward-facing perimeter sensors are an early-warning model. Evaluate
    % only pre-entry/entry samples instead of penalising the layout for not
    % tracking the drone after it has entered internal airspace.
    zones = string({records.zone});
    approachMask = zones == "early_warning_zone" | ...
                   zones == "approach_zone" | ...
                   zones == "perimeter_crossing";
    records = records(approachMask);
end

function sensors = buildFence24DirectionalSensors(thresholdDbm, secureWallFovDeg)
    coords = [
        250 300 15
        250 350 15
        250 450 15
        250 550 15
        250 650 15
        250 700 15
        300 700 15
        400 700 15
        600 700 15
        700 700 15
        750 700 15
        750 650 15
        750 550 15
        750 450 15
        750 350 15
        750 300 15
        700 300 15
        600 300 15
        400 300 15
        300 300 15
        300 500 15
        500 350 15
        700 500 15
        500 650 15
    ];

    mountType = [
        repmat("fence", 20, 1)
        repmat("wall", 4, 1)
    ];

    orientations = zeros(24, 1);
    facilityCentre = [500, 500];
    for i = 1:20
        % Perimeter/fence sensors face outward for early warning before the
        % drone reaches the controlled facility boundary.
        orientations(i) = atan2d(coords(i, 2) - facilityCentre(2), ...
                                 coords(i, 1) - facilityCentre(1));
    end

    % Secure wall sensors face outward into the clear zone.
    orientations(21) = 180;  % west wall midpoint looks west
    orientations(22) = -90;  % south wall midpoint looks south
    orientations(23) = 0;    % east wall midpoint looks east
    orientations(24) = 90;   % north wall midpoint looks north

    fovDeg = [repmat(90, 20, 1); repmat(secureWallFovDeg, 4, 1)];

    sensors = table((0:23)', coords(:, 1), coords(:, 2), coords(:, 3), ...
        orientations, fovDeg, repmat(thresholdDbm, 24, 1), mountType, ...
        repmat("directional", 24, 1), ...
        'VariableNames', {'sensor_id', 'x', 'y', 'z', 'orientation_deg', ...
        'fov_deg', 'threshold_dbm', 'mount_type', 'sensor_type'});
end

function sensors = configureSensorMode(baseSensors, sensorMode, perimeterFovDeg, secureWallFovDeg)
    sensors = baseSensors;
    sensors.fov_deg(sensors.mount_type == "fence") = perimeterFovDeg;
    sensors.fov_deg(sensors.mount_type == "wall") = secureWallFovDeg;
    sensors.sensor_type(:) = "directional";

    if sensorMode == "hybrid_corner_wall_omni"
        cornerIds = [0; 5; 10; 15];
        wallIds = [20; 21; 22; 23];
        omniIds = [cornerIds; wallIds];
        sensors.sensor_type(ismember(sensors.sensor_id, omniIds)) = "omni";
        sensors.fov_deg(sensors.sensor_type == "omni") = 360;
    end
end

function result = hybridAoaRssiEstimate(rec, sensors, rssiSigmaDb, aoaSigmaDeg, frequencyHz, ...
    txPowerDbm, txGainDbi, rxGainDbi)

    result = blankEstimate();
    [insideCone, trueRx, trueBearings] = deterministicSensorView(rec, sensors, frequencyHz, ...
        txPowerDbm, txGainDbi, rxGainDbi);
    noisyRx = trueRx + rssiSigmaDb * randn(height(sensors), 1);
    isOmni = sensors.sensor_type == "omni";
    detected = (insideCone | isOmni) & noisyRx >= sensors.threshold_dbm;
    idx = find(detected);
    result.usedCount = numel(idx);

    if numel(idx) < 2
        return;
    end

    bearingMask = sensors.sensor_type(idx) == "directional";
    measuredBearings = trueBearings(idx(bearingMask)) + aoaSigmaDeg * randn(sum(bearingMask), 1);
    ranges3d = inverseFriisRange(noisyRx(idx), frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
    heightDelta = rec.z - sensors.z(idx);
    ranges = sqrt(max(ranges3d.^2 - heightDelta.^2, 0));
    sensorXY = [sensors.x(idx), sensors.y(idx)];

    initial = bearingLeastSquares(sensorXY(bearingMask, :), measuredBearings);
    if any(~isfinite(initial))
        initial = mean(sensorXY, 1);
    end

    rangeStd = max((log(10) / 20) * max(rssiSigmaDb, 0.25) * ranges, 1.0);
    bearingStdRad = deg2rad(max(aoaSigmaDeg, 0.1));
    objective = @(p) hybridObjective(p, sensorXY, ranges, rangeStd, bearingMask, measuredBearings, bearingStdRad);
    options = optimset('Display', 'off', 'MaxIter', 120, 'TolX', 1e-4, 'TolFun', 1e-4);
    estimate = fminsearch(objective, initial, options);

    result.x = estimate(1);
    result.y = estimate(2);
    result.ok = all(isfinite(estimate));
end

function kalmanEstimates = applyResidualGatedKalman(records, rawEstimates, rssiSigmaDb, aoaSigmaDeg)
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
            acceptedUpdate = false;
            dt = 1;
            if isfinite(previousTime)
                dt = max(rec.time - previousTime, 0.1);
            end

            if isempty(state)
                if rawEstimates(currentIndex).ok
                    state = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y; 0; 0];
                    P = diag([16, 16, 10, 10]);
                    acceptedUpdate = true;
                else
                    previousTime = rec.time;
                    continue;
                end
            else
                F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
                q = max(0.5, 0.35 * rssiSigmaDb + 0.65 * aoaSigmaDeg);
                Q = q^2 * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; ...
                           dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
                state = F * state;
                P = F * P * F' + Q;
            end

            if rawEstimates(currentIndex).ok
                z = [rawEstimates(currentIndex).x; rawEstimates(currentIndex).y];
                H = [1 0 0 0; 0 1 0 0];
                measurementSigma = max(1.0, 3.0 * aoaSigmaDeg + 1.0 * rssiSigmaDb);
                R = measurementSigma^2 * eye(2);
                innovation = z - H * state;
                innovationDistance = sqrt(innovation' * innovation);
                gateDistance = max(10, 4 * measurementSigma);

                if innovationDistance <= gateDistance
                    K = P * H' / (H * P * H' + R);
                    state = state + K * innovation;
                    P = (eye(4) - K * H) * P;
                    acceptedUpdate = true;
                end
            end

            kalmanEstimates(currentIndex).x = state(1);
            kalmanEstimates(currentIndex).y = state(2);
            % Count Kalman as a valid localisation only when a recent AoA/RSSI
            % measurement was accepted. Pure predictions are useful internally
            % but should not be reported as successful detections.
            kalmanEstimates(currentIndex).ok = acceptedUpdate && isfinite(state(1)) && isfinite(state(2));
            kalmanEstimates(currentIndex).usedCount = rawEstimates(currentIndex).usedCount;
            previousTime = rec.time;
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

function value = hybridObjective(p, sensorXY, ranges, rangeStd, bearingMask, measuredBearings, bearingStdRad)
    dx = p(1) - sensorXY(:, 1);
    dy = p(2) - sensorXY(:, 2);
    predictedRange = sqrt(dx.^2 + dy.^2);
    rangeResidual = (predictedRange - ranges) ./ rangeStd;
    predictedBearing = atan2d(dy(bearingMask), dx(bearingMask));
    bearingResidual = deg2rad(wrap180(predictedBearing - measuredBearings)) ./ bearingStdRad;
    value = sum(rangeResidual.^2) + sum(bearingResidual.^2);
end

function initial = bearingLeastSquares(sensorXY, bearingsDeg)
    if numel(bearingsDeg) < 2 || size(sensorXY, 1) < 2
        initial = [NaN, NaN];
        return;
    end
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

function result = blankEstimate()
    result = struct('ok', false, 'x', NaN, 'y', NaN, 'usedCount', 0);
end

function row = summaryRow(sensorMode, method, perimeterFovDeg, wallFovDeg, rssiSigmaDb, aoaSigmaDeg, totalCases, errors, sensorCounts)
    localisedCases = numel(errors);
    [meanError, rmseError, medianError, p95Error, maxError] = errorStats(errors);
    row = {sensorMode, method, perimeterFovDeg, wallFovDeg, rssiSigmaDb, aoaSigmaDeg, totalCases, localisedCases, ...
        100 * localisedCases / max(totalCases, 1), meanError, rmseError, medianError, ...
        p95Error, maxError, percentBelow(errors, 1), percentBelow(errors, 2), ...
        percentBelow(errors, 5), percentBelow(errors, 10), mean(sensorCounts, 'omitnan')};
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

function angle = wrap180(angle)
    angle = mod(angle + 180, 360) - 180;
end

function plotFence24Layout(sensors, records)
    paths = corpus_paths();
    prisonMinX = 300; prisonMaxX = 700;
    prisonMinY = 350; prisonMaxY = 650;
    fenceMinX = 250; fenceMaxX = 750;
    fenceMinY = 300; fenceMaxY = 700;

    figure; hold on; axis equal; grid on;
    rectangle('Position', [fenceMinX, fenceMinY, fenceMaxX - fenceMinX, fenceMaxY - fenceMinY], ...
        'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.2, 'LineStyle', '--');
    rectangle('Position', [prisonMinX, prisonMinY, prisonMaxX - prisonMinX, prisonMaxY - prisonMinY], ...
        'EdgeColor', 'k', 'LineWidth', 2);
    directionalMask = sensors.sensor_type == "directional";
    omniMask = sensors.sensor_type == "omni";
    scatter(sensors.x(directionalMask), sensors.y(directionalMask), 60, 'r', 'filled');
    scatter(sensors.x(omniMask), sensors.y(omniMask), 75, 'm', 'filled', 'Marker', 's');
    scatter([records.x], [records.y], 8, [0.2 0.45 0.85], 'filled', 'MarkerFaceAlpha', 0.25);

    coneLen = 45;
    directionalIds = find(directionalMask);
    for j = 1:numel(directionalIds)
        i = directionalIds(j);
        quiver(sensors.x(i), sensors.y(i), coneLen * cosd(sensors.orientation_deg(i)), ...
            coneLen * sind(sensors.orientation_deg(i)), 0, 'Color', [0.8 0 0], 'LineWidth', 1);
    end

    xlabel('x (m)'); ylabel('y (m)');
    title('Fixed Fence24 Hybrid Sensor Layout: Directional Cones + Omni Anchors');
    hFenceLegend = plot(nan, nan, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);
    hWallLegend = plot(nan, nan, '-', 'Color', 'k', 'LineWidth', 2);
    hSensorLegend = scatter(nan, nan, 60, 'r', 'filled');
    hOmniLegend = scatter(nan, nan, 75, 'm', 'filled', 'Marker', 's');
    hDroneLegend = scatter(nan, nan, 8, [0.2 0.45 0.85], 'filled');
    legend([hFenceLegend, hWallLegend, hSensorLegend, hOmniLegend, hDroneLegend], ...
        {'Outer fence', 'Secure wall', 'Directional sensors', 'Omnidirectional sensors', 'Drone path samples'}, ...
        'Location', 'bestoutside');
    savefig(fullfile(paths.figures, 'aoa_fence24_layout.fig'));
    exportgraphics(gcf, fullfile(paths.figures, 'aoa_fence24_layout.png'), 'Resolution', 200);
end

function plotHeatmap(summary, sensorMode, methodName, fovDeg, metricName, titleText, outputPng)
    rows = summary(summary.sensor_mode == sensorMode & summary.method == methodName & summary.perimeter_fov_deg == fovDeg, :);
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
