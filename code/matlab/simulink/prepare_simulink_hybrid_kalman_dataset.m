clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% SIMULINK HYBRID-KALMAN DATASET PREPARATION:
% Uses the best tuned hybrid configuration found in MATLAB:
%   threshold = -70 dBm
%   omni RSSI weight = 0.02
%   Kalman accel std = 0.2
%   Kalman measurement std = 3
%   Kalman residual gate = 80 m
% This prepares a single representative validation path for a Simulink
% comparison model. No ns-3 run is required.

sourceCsv = fullfile(paths.rawUniform, 'prison_fence24_random_seed4_-60.csv');
if ~isfile(sourceCsv)
    error("Source CSV not found: %s", sourceCsv);
end

outMeasurementCsv = fullfile(paths.simulink, 'simulink_hybrid_kalman_measurements.csv');
outReferenceCsv = fullfile(paths.simulink, 'simulink_hybrid_kalman_reference_estimates.csv');
outSensorCsv = fullfile(paths.simulink, 'simulink_hybrid_kalman_sensors.csv');
outMat = fullfile(paths.simulink, 'simulink_hybrid_kalman_dataset.mat');

thresholdDbm = -70;
rssiSigmaDb = 1.0;
aoaSigmaDeg = 0.5;
omniWeight = 0.02;
kalmanAccelStd = 0.2;
kalmanMeasurementStd = 3;
kalmanGateM = 80;
frequencyHz = 2.4e9;
txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
rng(2207);

raw = readtable(sourceCsv, "TextType", "string");
[groups, timeValues] = findgroups(raw.time);
numSteps = max(groups);

path = table();
path.time = timeValues;
path.sim_time = timeValues - timeValues(1);
path.uav_x = splitapply(@(x) x(1), raw.uav_x, groups);
path.uav_y = splitapply(@(x) x(1), raw.uav_y, groups);
path.uav_z = splitapply(@(x) x(1), raw.uav_z, groups);
path.zone = splitapply(@(x) x(1), raw.zone, groups);
path.time_to_boundary = splitapply(@(x) x(1), raw.time_to_boundary, groups);

sensors = buildHybridFence24Sensors(thresholdDbm);
writetable(sensors, outSensorCsv);

numSensors = height(sensors);
rssiNoise = rssiSigmaDb * randn(numSteps, numSensors);
aoaNoise = aoaSigmaDeg * randn(numSteps, numSensors);

measurementRows = cell(numSteps * numSensors, 22);
rawEstimates = table();
rawEstimates.time = path.time;
rawEstimates.sim_time = path.sim_time;
rawEstimates.true_x = path.uav_x;
rawEstimates.true_y = path.uav_y;
rawEstimates.zone = path.zone;
rawEstimates.raw_x = nan(numSteps, 1);
rawEstimates.raw_y = nan(numSteps, 1);
rawEstimates.raw_error_m = nan(numSteps, 1);
rawEstimates.raw_ok = false(numSteps, 1);
rawEstimates.detecting_sensor_count = zeros(numSteps, 1);
rawEstimates.directional_count = zeros(numSteps, 1);
rawEstimates.omni_count = zeros(numSteps, 1);

rowIndex = 1;
for k = 1:numSteps
    uav = [path.uav_x(k), path.uav_y(k), path.uav_z(k)];
    rxNoisy = zeros(numSensors, 1);
    measuredAoa = nan(numSensors, 1);
    detected = false(numSensors, 1);

    for s = 1:numSensors
        sensorPos = [sensors.x(s), sensors.y(s), sensors.z(s)];
        distance = norm(uav - sensorPos);
        rxIdeal = friisRxPowerDbm(distance, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
        rxNoisy(s) = rxIdeal + rssiNoise(k, s);
        trueAoa = atan2d(path.uav_y(k) - sensors.y(s), path.uav_x(k) - sensors.x(s));

        if sensors.sensor_type(s) == "omni"
            inFov = true;
            measuredAoa(s) = NaN;
        else
            inFov = abs(wrap180(trueAoa - sensors.orientation_deg(s))) <= sensors.fov_deg(s) / 2;
            measuredAoa(s) = wrap180(trueAoa + aoaNoise(k, s));
        end
        detected(s) = inFov && rxNoisy(s) >= thresholdDbm;

        measurementRows(rowIndex, :) = { ...
            path.time(k), path.sim_time(k), path.uav_x(k), path.uav_y(k), path.uav_z(k), path.zone(k), ...
            sensors.sensor_id(s), sensors.sensor_type(s), sensors.x(s), sensors.y(s), sensors.z(s), ...
            sensors.orientation_deg(s), sensors.fov_deg(s), distance, rxIdeal, rssiNoise(k, s), rxNoisy(s), ...
            trueAoa, aoaNoise(k, s), measuredAoa(s), double(inFov), double(detected(s))};
        rowIndex = rowIndex + 1;
    end

    [estX, estY, ok, usedCount, directionalCount, omniCount] = estimateHybridPosition( ...
        sensors, rxNoisy, measuredAoa, detected, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi, ...
        rssiSigmaDb, aoaSigmaDeg, omniWeight);

    rawEstimates.detecting_sensor_count(k) = usedCount;
    rawEstimates.directional_count(k) = directionalCount;
    rawEstimates.omni_count(k) = omniCount;
    rawEstimates.raw_ok(k) = ok;
    if ok
        rawEstimates.raw_x(k) = estX;
        rawEstimates.raw_y(k) = estY;
        rawEstimates.raw_error_m(k) = hypot(estX - path.uav_x(k), estY - path.uav_y(k));
    end
end

referenceEstimates = applyTunedKalman(rawEstimates, kalmanAccelStd, kalmanMeasurementStd, kalmanGateM);

measurements = cell2table(measurementRows, 'VariableNames', { ...
    'time', 'sim_time', 'uav_x', 'uav_y', 'uav_z', 'zone', ...
    'sensor_id', 'sensor_type', 'sensor_x', 'sensor_y', 'sensor_z', ...
    'orientation_deg', 'fov_deg', 'true_distance_m', 'rx_power_dbm_ideal', ...
    'rssi_noise_db', 'rx_power_dbm_noisy', 'true_aoa_deg', 'aoa_noise_deg', ...
    'measured_aoa_deg', 'inside_fov', 'detected'});

writetable(measurements, outMeasurementCsv);
writetable(referenceEstimates, outReferenceCsv);

sim_time = path.sim_time;
true_x_signal = [sim_time, path.uav_x];
true_y_signal = [sim_time, path.uav_y];
true_z_signal = [sim_time, path.uav_z];
rssi_noise_signal = [sim_time, rssiNoise];
aoa_noise_signal = [sim_time, aoaNoise];
reference_estimate_signal = [sim_time, referenceEstimates.kalman_x, referenceEstimates.kalman_y];
reference_error_signal = [sim_time, referenceEstimates.kalman_error_m];

save(outMat, ...
    "sim_time", "true_x_signal", "true_y_signal", "true_z_signal", ...
    "rssi_noise_signal", "aoa_noise_signal", "reference_estimate_signal", "reference_error_signal", ...
    "thresholdDbm", "rssiSigmaDb", "aoaSigmaDeg", "omniWeight", ...
    "kalmanAccelStd", "kalmanMeasurementStd", "kalmanGateM", ...
    "frequencyHz", "txPowerDbm", "txGainDbi", "rxGainDbi");

valid = referenceEstimates.kalman_error_m(isfinite(referenceEstimates.kalman_error_m));
fprintf("Hybrid-Kalman measurement CSV saved: %s\n", outMeasurementCsv);
fprintf("Hybrid-Kalman reference CSV saved: %s\n", outReferenceCsv);
fprintf("Hybrid-Kalman sensor CSV saved: %s\n", outSensorCsv);
fprintf("Hybrid-Kalman MAT dataset saved: %s\n", outMat);
fprintf("Kalman tracked timestamps: %d / %d\n", numel(valid), numSteps);
fprintf("Kalman mean error: %.3f m\n", mean(valid));
fprintf("Kalman RMSE: %.3f m\n", sqrt(mean(valid.^2)));

function sensors = buildHybridFence24Sensors(thresholdDbm)
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
    threshold = thresholdDbm * ones(size(sensorId));
    sensorType = repmat("directional", size(sensorId));
    omniIds = [0; 5; 10; 15; 20; 21; 22; 23];
    sensorType(ismember(sensorId, omniIds)) = "omni";
    fov(sensorType == "omni") = 360;
    sensors = table(sensorId, x, y, z, orientation, fov, threshold, sensorType, ...
        'VariableNames', {'sensor_id', 'x', 'y', 'z', 'orientation_deg', 'fov_deg', 'threshold_dbm', 'sensor_type'});
end

function [estX, estY, ok, usedCount, directionalCount, omniCount] = estimateHybridPosition(sensors, rxNoisy, measuredAoa, detected, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi, rssiSigmaDb, aoaSigmaDeg, omniWeight)
    idx = find(detected);
    usedCount = numel(idx);
    directionalCount = sum(sensors.sensor_type(idx) == "directional");
    omniCount = sum(sensors.sensor_type(idx) == "omni");
    estX = NaN; estY = NaN; ok = false;
    if ~(directionalCount >= 2 || usedCount >= 3 || (directionalCount >= 1 && usedCount >= 2))
        return;
    end

    sensorXY = [sensors.x(idx), sensors.y(idx)];
    ranges = inverseFriisRange(rxNoisy(idx), frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
    rangeStd = max((log(10) / 20) * rssiSigmaDb .* ranges, 1);
    rangeWeightScale = ones(usedCount, 1);
    rangeWeightScale(sensors.sensor_type(idx) == "omni") = omniWeight;
    bearingMask = sensors.sensor_type(idx) == "directional";
    measuredBearings = measuredAoa(idx(bearingMask));
    bearingStdRad = deg2rad(aoaSigmaDeg);

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
    estimate = fminsearch(objective, initial, optimset('Display', 'off', 'MaxIter', 80, 'TolX', 1e-5));
    estX = estimate(1);
    estY = estimate(2);
    ok = isfinite(estX) && isfinite(estY);
end

function value = hybridObjective(p, sensorXY, ranges, rangeStd, rangeWeightScale, bearingMask, measuredBearings, bearingStdRad)
    dx = p(1) - sensorXY(:, 1);
    dy = p(2) - sensorXY(:, 2);
    predictedRange = max(sqrt(dx.^2 + dy.^2), 1e-6);
    value = sum((rangeWeightScale .* ((predictedRange - ranges) ./ rangeStd)).^2);
    if any(bearingMask)
        predictedBearing = atan2d(dy(bearingMask), dx(bearingMask));
        bearingResidual = deg2rad(wrap180(predictedBearing - measuredBearings)) ./ bearingStdRad;
        value = value + sum(bearingResidual.^2);
    end
end

function T = applyTunedKalman(raw, accelStd, measStd, gateM)
    T = raw;
    T.kalman_x = nan(height(raw), 1);
    T.kalman_y = nan(height(raw), 1);
    T.kalman_error_m = nan(height(raw), 1);
    T.kalman_ok = false(height(raw), 1);
    T.measurement_update = false(height(raw), 1);
    H = [1 0 0 0; 0 1 0 0];
    R = (measStd^2) * eye(2);
    state = zeros(4, 1);
    P = diag([100, 100, 25, 25]);
    initialised = false;
    lastTime = raw.time(1);
    for i = 1:height(raw)
        if initialised
            dt = max(raw.time(i) - lastTime, 1);
            F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
            q = accelStd^2;
            Q = q * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
            state = F * state;
            P = F * P * F' + Q;
        end
        if raw.raw_ok(i)
            z = [raw.raw_x(i); raw.raw_y(i)];
            if ~initialised
                state = [z(1); z(2); 0; 0];
                initialised = true;
                T.measurement_update(i) = true;
            else
                residual = z - H * state;
                if norm(residual) <= gateM
                    S = H * P * H' + R;
                    K = P * H' / S;
                    state = state + K * residual;
                    P = (eye(4) - K * H) * P;
                    T.measurement_update(i) = true;
                end
            end
        end
        if initialised
            T.kalman_x(i) = state(1);
            T.kalman_y(i) = state(2);
            T.kalman_error_m(i) = hypot(state(1) - raw.true_x(i), state(2) - raw.true_y(i));
            T.kalman_ok(i) = true;
        end
        lastTime = raw.time(i);
    end
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
