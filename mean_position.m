function mean_pos = mean_position(particles, weights)
    % Returns a single estimate of filter state based on the particle cloud.
    %
    % particles (M x 3): set of M particles to sample from. Each row contains a state hypothesis of dimension 3 (x, y, theta).
    % weights (M x 1): weights of the particles. Each row contains a weight.

    % initialize
    M=size(weights,1);
    mean_pos = zeros(1,3);
    for i=1:M
        particles(i,:)=particles(i,:)*weights(i);
    end
    mean_pos(1)=sum(particles(:,1));
    mean_pos(2)=sum(particles(:,2));
    mean_pos(3)=sum(particles(:,3));
    %Si tomo como criterio la máxima probabilidad , la posición tiene
    %Saltos bruscos
    %[val,idx]=max(weights); 
    %mean_pos=particles(idx,:);
    %% TODO: compute mean_pos    
end
