function [x, y] = trilaterate2d(p1, r1, p2, r2, p3, r3)
%TRILATERATE2D Estimate 2D position from three sensor positions and ranges.
%
% p1, p2, p3 are [x, y] sensor coordinates.
% r1, r2, r3 are estimated distances from each sensor to the target.

    x1 = p1(1); y1 = p1(2);
    x2 = p2(1); y2 = p2(2);
    x3 = p3(1); y3 = p3(2);

    A = 2 * [x2 - x1, y2 - y1;
             x3 - x1, y3 - y1];

    b = [r1^2 - r2^2 - x1^2 + x2^2 - y1^2 + y2^2;
         r1^2 - r3^2 - x1^2 + x3^2 - y1^2 + y3^2];

    % FENCE24 LOCALISATION:
    % Trilateration requires non-collinear sensors. If the matrix is nearly
    % singular, return NaN so the caller can exclude the row from statistics.
    if rank(A) < 2 || rcond(A) < 1e-12
        x = NaN;
        y = NaN;
        return;
    end

    pos = A \ b;

    x = pos(1);
    y = pos(2);
end
