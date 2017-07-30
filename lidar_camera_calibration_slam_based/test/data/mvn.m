function p = mvn(X)
    Sigma = eye(length(X));
    p = exp(-1/2*X' * inv(Sigma) * X) / sqrt(det(2 * pi * Sigma));
end
