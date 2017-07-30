normal1D=randn(1,10000);
csvwrite('normal1D.txt',normal1D);
normal2D=mvnrnd(zeros(10000,2),eye(2));
csvwrite('normal2D_1.txt',(normal2D(:,1))')
csvwrite('normal2D_2.txt',(normal2D(:,2))')
normal2D = normal2D';
p1=getGaussWindowEstimateFunction(normal1D, 0.1 * eye(1));
p2=getGaussWindowEstimateFunction(normal2D, 0.1 * eye(2));

result1 = [];
result2 = [];
residual1 = [];
residual2 = [];
querys = -1:0.01:1;

csvwrite('querys.txt', querys);

for x=querys
    result1(end+1) = p1([x]); residual1(end+1) = p1([x])-mvn([x]);
    result2(end+1) = p2([x;x]); residual2(end+1) = p2([x;x])-mvn([x;x]);
end

csvwrite('result1.txt', result1);
csvwrite('result2.txt', result2);

norm(residual1)
norm(residual2)
