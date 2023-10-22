% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  err = 0;

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    z = v2t(edge.measurement);

    err = t2v(pinv(z)*(pinv(x1)*x2));

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark
    z = edge.measurement;

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    X = v2t(x);
    Ri = X(1:2,1:2);
    err = Ri'*(l-x(1:2))-z;
  end

  Fx = Fx + err' * edge.information * err;

end
