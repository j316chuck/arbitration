N = 100;
interval = 5;
n = N / interval; 
parfor i = 1:n
    st = (i-1) * interval + 1; en = i * interval; 
    fprintf("Running experiments %d %d\n", st, en);
    runner(st, en); 
end
