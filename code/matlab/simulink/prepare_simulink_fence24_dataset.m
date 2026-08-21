clear; clc; close all;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

% SIMULINK DATASET PREPARATION:
% Builds a clean time-series dataset for the final Simulink model using the
% best current localisation candidate: fixed fence24, all-directional sensors,
% 120 degree FOV, RSSI + AoA measurements, and low-noise conditions.

sourceCsv = fullfile(paths.rawUniform, 'prison_fence24_random_seed1_-60.csv');
if ~isfile(sourceCsv)
    error("Source CSV not found: %s", sourceCsv);
end

outMeasurementCsv = fullfile(paths.simulink, 'simulink_fence24_directional_measurements.csv');
outEstimateCsv = fullfile(paths.simulink, 'simulink_fence24_directional_reference_estimates.csv');
outSensorCsv = fullfile(paths.simulink, 'simulink_fence24_directional_sensors.csv');
outMat = fullfile(paths.simulink, 'simulink_fence24_directional_dataset.mat');

rssiSigmaDb = 1.0;
aoaSigmaDeg = 0.5;
sensorThresholdDbm = -65;
frequencyHz = 2.4e9;
txPowerDbm = 20;
txGainDbi = 0;
rxGainDbi = 0;
rng(1207);

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

sensors = buildDirectionalFence24Sensors(sensorThresholdDbm);
writetable(sensors, outSensorCsv);

numSensors = height(sensors);
rssiNoise = rssiSigmaDb * randn(numSteps, numSensors);
aoaNoise = aoaSigmaDeg * randn(numSteps, numSensors);

measurementRows = cell(numSteps * numSensors, 20);
rowIndex = 1;

referenceEstimates = table();
referenceEstimates.time = path.time;
referenceEstimates.sim_time = path.sim_time;
referenceEstimates.true_x = path.uav_x;
referenceEstimates.true_y = path.uav_y;
referenceEstimates.zone = path.zone;
referenceEstimates.detecting_sensor_count = zeros(numSteps, 1);
referenceEstimates.confirmed_detection = zeros(numSteps, 1);
referenceEstimates.estimated_x = nan(numSteps, 1);
referenceEstimates.estimated_y = nan(numSteps, 1);
referenceEstimates.error_m = nan(numSteps, 1);

for k = 1:numSteps
    uav = [path.uav_x(k), path.uav_y(k), path.uav_z(k)];
    rxNoisy = zeros(numSensors, 1);
    measuredAoa = zeros(numSensors, 1);
    detected = false(numSensors, 1);

    for s = 1:numSensors
        sensorPos = [sensors.x(s), sensors.y(s), sensors.z(s)];
        distance = norm(uav - sensorPos);
        rxIdeal = friisRxPowerDbm(distance, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
        rxNoisy(s) = rxIdeal + rssiNoise(k, s);

        trueAoa = atan2d(path.uav_y(k) - sensors.y(s), path.uav_x(k) - sensors.x(s));
        measuredAoa(s) = wrap180(trueAoa + aoaNoise(k, s));
        angleFromBoresight = wrap180(trueAoa - sensors.orientation_deg(s));
        inFov = abs(angleFromBoresight) <= sensors.fov_deg(s) / 2;
        detected(s) = inFov && rxNoisy(s) >= sensors.threshold_dbm(s);

        measurementRows(rowIndex, :) = { ...
            path.time(k), path.sim_time(k), path.uav_x(k), path.uav_y(k), path.uav_z(k), path.zone(k), ...
            sensors.sensor_id(s), sensors.x(s), sensors.y(s), sensors.z(s), sensors.orientation_deg(s), ...
            sensors.fov_deg(s), distance, rxIdeal, rssiNoise(k, s), rxNoisy(s), ...
            trueAoa, aoaNoise(k, s), measuredAoa(s), double(detected(s))};
        rowIndex = rowIndex + 1;
    end

    [estX, estY, ok, usedCount] = estimateAoaRssiPosition( ...
        sensors, rxNoisy, measuredAoa, detected, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi, ...
        rssiSigmaDb, aoaSigmaDeg);

    referenceEstimates.detecting_sensor_count(k) = usedCount;
    referenceEstimates.confirmed_detection(k) = usedCount >= 2;
    if ok
        referenceEstimates.estimated_x(k) = estX;
        referenceEstimates.estimated_y(k) = estY;
        referenceEstimates.error_m(k) = hypot(estX - path.uav_x(k), estY - path.uav_y(k));
    end
end

measurements = cell2table(measurementRows, 'VariableNames', { ...
    'time', 'sim_time', 'uav_x', 'uav_y', 'uav_z', 'zone', ...
    'sensor_id', 'sensor_x', 'sensor_y', 'sensor_z', 'orientation_deg', 'fov_deg', ...
    'true_distance_m', 'rx_power_dbm_ideal', 'rssi_noise_db', 'rx_power_dbm_noisy', ...
    'true_aoa_deg', 'aoa_noise_deg', 'measured_aoa_deg', 'detected'});

writetable(measurements, outMeasurementCsv);
writetable(referenceEstimates, outEstimateCsv);

sim_time = path.sim_time;
true_x_signal = [sim_time, path.uav_x];
true_y_signal = [sim_time, path.uav_y];
true_z_signal = [sim_time, path.uav_z];
rssi_noise_signal = [sim_time, rssiNoise];
aoa_noise_signal = [sim_time, aoaNoise];
reference_estimate_signal = [sim_time, referenceEstimates.estimated_x, referenceEstimates.estimated_y];
reference_error_signal = [sim_time, referenceEstimates.error_m];

sim_sensor_ids = sensors.sensor_id;
sim_sensor_x = sensors.x;
sim_sensor_y = sensors.y;
sim_sensor_z = sensors.z;
sim_sensor_orientation_deg = sensors.orientation_deg;
sim_sensor_fov_deg = sensors.fov_deg;
sim_sensor_threshold_dbm = sensors.threshold_dbm;
sim_frequency_hz = frequencyHz;
sim_tx_power_dbm = txPowerDbm;
sim_tx_gain_dbi = txGainDbi;
sim_rx_gain_dbi = rxGainDbi;
sim_rssi_sigma_db = rssiSigmaDb;
sim_aoa_sigma_deg = aoaSigmaDeg;

save(outMat, ...
    "sim_time", "true_x_signal", "true_y_signal", "true_z_signal", ...
    "rssi_noise_signal", "aoa_noise_signal", ...
    "reference_estimate_signal", "reference_error_signal", ...
    "sim_sensor_ids", "sim_sensor_x", "sim_sensor_y", "sim_sensor_z", ...
    "sim_sensor_orientation_deg", "sim_sensor_fov_deg", "sim_sensor_threshold_dbm", ...
    "sim_frequency_hz", "sim_tx_power_dbm", "sim_tx_gain_dbi", "sim_rx_gain_dbi", ...
    "sim_rssi_sigma_db", "sim_aoa_sigma_deg");

fprintf("Simulink measurement CSV saved: %s\n", outMeasurementCsv);
fprintf("Simulink reference estimate CSV saved: %s\n", outEstimateCsv);
fprintf("Simulink sensor CSV saved: %s\n", outSensorCsv);
fprintf("Simulink MAT dataset saved: %s\n", outMat);

validErrors = referenceEstimates.error_m(~isnan(referenceEstimates.error_m));
fprintf("Reference localised timestamps: %d / %d\n", numel(validErrors), numSteps);
fprintf("Reference mean error: %.3f m\n", mean(validErrors));
fprintf("Reference RMSE: %.3f m\n", sqrt(mean(validErrors.^2)));

function sensors = buildDirectionalFence24Sensors(thresholdDbm)
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
    mountType = repmat("fence", size(sensorId));
    mountType(sensorId >= 20) = "wall";
    sensors = table(sensorId, x, y, z, orientation, fov, threshold, mountType, ...
        'VariableNames', {'sensor_id', 'x', 'y', 'z', 'orientation_deg', 'fov_deg', 'threshold_dbm', 'mount_type'});
end

function [estX, estY, ok, usedCount] = estimateAoaRssiPosition(sensors, rxPowerDbm, measuredAoaDeg, detected, frequencyHz, txPowerDbm, txGainDbi, rxGainDbi, rssiSigmaDb, aoaSigmaDeg)
    idx = find(detected);
    usedCount = numel(idx);
    ok = false;
    estX = NaN;
    estY = NaN;
    if usedCount < 2
        return;
    end

    sensorX = sensors.x(idx);
    sensorY = sensors.y(idx);
    ranges = inverseFriisRange(rxPowerDbm(idx), frequencyHz, txPowerDbm, txGainDbi, rxGainDbi);
    bearingsRad = deg2rad(measuredAoaDeg(idx));
    rangeStd = max((log(10) / 20) * max(rssiSigmaDb, 0.1) .* ranges, 1);
    bearingStd = deg2rad(max(aoaSigmaDeg, 0.1));

    projectedX = sensorX + ranges .* cos(bearingsRad);
    projectedY = sensorY + ranges .* sin(bearingsRad);
    pos = [mean(projectedX); mean(projectedY)];

    for iter = 1:25
        residual = zeros(2 * usedCount, 1);
        jacobian = zeros(2 * usedCount, 2);
        weights = zeros(2 * usedCount, 1);
        row = 1;

        for n = 1:usedCount
            dx = pos(1) - sensorX(n);
            dy = pos(2) - sensorY(n);
            predictedRange = max(sqrt(dx * dx + dy * dy), 1e-6);
            predictedBearing = atan2(dy, dx);

            residual(row) = predictedRange - ranges(n);
            jacobian(row, :) = [dx / predictedRange, dy / predictedRange];
            weights(row) = 1 / rangeStd(n);
            row = row + 1;

            residual(row) = wrapPi(predictedBearing - bearingsRad(n));
            jacobian(row, :) = [-dy / (predictedRange ^ 2), dx / (predictedRange ^ 2)];
            weights(row) = 1 / bearingStd;
            row = row + 1;
        end

        weightedJacobian = jacobian .* weights;
        weightedResidual = residual .* weights;
        step = -((weightedJacobian' * weightedJacobian + 1e-5 * eye(2)) \ (weightedJacobian' * weightedResidual));
        pos = pos + step;
        if norm(step) < 1e-4
            break;
        end
    end

    estX = pos(1);
    estY = pos(2);
    ok = isfinite(estX) && isfinite(estY);
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

function angle = wrapPi(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end
