% Test script to see if parallel loops working on cluster
num_limit = 25000;
num_runs = 96*4;

% Start parpool if not already running
if isempty(gcp('nocreate'))

    % Determine if running locally or in cluster
    clusterName = getenv('CMD_WLM_CLUSTER_NAME');
    if numel(clusterName) == 0
        inCluster = false
    elseif strcmp(clusterName, "slurm")
        inCluster = true
    else
        error("Unexpected value for CMD_WLM_CLUSTER_NAME: " + clusterName)
    end

    % Construct cluster with correct number of CPUs
    pc = parcluster('local')
    if inCluster
        nproc = str2num(getenv('SLURM_CPUS_PER_TASK')) * str2num(getenv("SLURM_NNODES"))
        job_folder = fullfile(getenv('TMPDIR'),getenv('SLURM_JOB_ID'));
    else
        nproc = pc.NumWorkers;
        job_folder = pwd+"/tmp_matlab/";
    end
    mkdir(job_folder);
    pc.NumWorkers = nproc;
    pc.JobStorageLocation = job_folder;
    tic;
    parpool(pc,nproc)
    disp(num2str(toc) + "s for creation of parpool")
end

% For loop
tic
for i = 1:num_runs
    primes = find_primes_up_to(num_limit);
end
disp(num2str(toc) + "s for normal for loop.")

% Parallel for loop
tic
parfor i = 1:num_runs
    primes = find_primes_up_to(num_limit);
end
disp(num2str(toc) + "s for parallel for loop with " +num2str(gcp().NumWorkers) + " workers.")


% % Parallel for loop - 24 workers
% delete(gcp('nocreate'));
% % Construct cluster with correct number of CPUs
% pc = parcluster('local');
% job_folder = fullfile(getenv('TMPDIR'),getenv('SLURM_JOB_ID'));
% nproc = 24;
% job_folder = pwd+"/tmp_matlab/";
% mkdir(job_folder);
% pc.NumWorkers = nproc;
% pc.JobStorageLocation = job_folder;
% parpool(pc,nproc);
% tic
% parfor i = 1:num_runs
%     primes = find_primes_up_to(num_limit);
% end
% disp(num2str(toc) + "s for parallel for loop with " +num2str(gcp().NumWorkers) + " workers.")



function primes = find_primes_up_to(x)
% Returns a list of all primes up to x
primes = [2];
for possible_prime = 3:x
    isPrime = true;

    num_primes = numel(primes);
    for idx = 1:num_primes
        known_prime = primes(idx);
        if mod(possible_prime, known_prime) == 0
            isPrime= false;
            break
        end
    end

    if isPrime
        primes(end+1) = possible_prime;
    end
end
end