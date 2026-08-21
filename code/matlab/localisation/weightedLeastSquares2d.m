function [x, y, iterations, converged] = weightedLeastSquares2d(sensorPositions, ranges, weights, initialPosition)
%WEIGHTEDLEASTSQUARES2D Estimate 2D UAV position from multiple sensor ranges.
%
% sensorPositions is an N-by-2 matrix of [x, y] sensor coordinates.
% ranges is an N-by-1 vector of estimated sensor-to-UAV distances.
% weights is an N-by-1 vector; larger weights make a sensor more influential.
% initialPosition is a 1-by-2 starting estimate for the iterative solver.
%
% This is used after RF noise is added to received power. More than three
% sensors can be used, so random range errors are averaged across the sensor
% network rather than allowing one noisy three-sensor triangle to dominate.

    maxIterations = 30;
    tolerance = 1e-4;
    damping = 1e-6;

    pos = initialPosition(:);
    weights = weights(:);
    ranges = ranges(:);
    converged = false;

    for iterations = 1:maxIterations
        dx = pos(1) - sensorPositions(:, 1);
        dy = pos(2) - sensorPositions(:, 2);
        predictedRanges = sqrt(dx.^2 + dy.^2);
        predictedRanges(predictedRanges < 1e-6) = 1e-6;

        residuals = predictedRanges - ranges;
        jacobian = [dx ./ predictedRanges, dy ./ predictedRanges];

        weightedJacobian = jacobian .* weights;
        hessianApprox = jacobian' * weightedJacobian + damping * eye(2);
        gradient = jacobian' * (weights .* residuals);

        if rcond(hessianApprox) < 1e-12
            x = NaN;
            y = NaN;
            return;
        end

        step = -hessianApprox \ gradient;
        pos = pos + step;

        if norm(step) < tolerance
            converged = true;
            break;
        end
    end

    x = pos(1);
    y = pos(2);
end
