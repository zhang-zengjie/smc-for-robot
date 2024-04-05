function F = friction(qd)

% Generate friction forces

F = [2.6e-4 0 0;
     0 2.6e-4 0; 
     0 0 2.6e-4]*qd;

end

