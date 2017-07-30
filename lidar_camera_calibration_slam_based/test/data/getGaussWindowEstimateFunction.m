function [ p ] = getGaussWindowEstimateFunction( X, Sigma )
    k = @(x)1/sqrt(det(2*pi*Sigma))*exp(-1/2 * sum(((x-X)' * inv(Sigma)) .* (x-X)', 2));
    p = @(x)mean(k(repmat(x, 1, size(X,2))));
end
