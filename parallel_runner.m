N = 100;
interval = 6;
n = ceil(N / interval); 
num_workers = 17; 
parfor (i = 1:n, num_workers)
    st = (i-1) * interval + 1; en = min(i * interval, N); 
    fprintf("Running experiments %d %d\n", st, en);
    runner(st, en); 
end
