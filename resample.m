function new_particles = resample(particles, weights)
    % Returns a new set of particles obtained by performing
    % stochastic universal sampling.
    %
    % particles (M x D): set of M particles to sample from. Each row contains a state hypothesis of dimension D.
    % weights (M x 1): weights of the particles. Each row contains a weight.
    M = size(particles, 1);
    new_particles = zeros(M,3);
    cdf=cumsum(weights);
    threshold=unifrnd(0,1/M);
    i=1;
    for j = 1:M
        while threshold>cdf(i)
            i=i+1;
        end
        new_particles(j,:)=particles(i,:);
        threshold=threshold+1/M;
    end
    %% TODO: complete this stub
end
