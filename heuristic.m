function heur = heuristic(cell, goal)
% g(n) = costo del estado inicial hasta n.
% h(n) = costo estimado desde n hasta el objetivo.
% f(n) = g(n) + h(n), costo estimado de la solución de
% menor costo a través de n.

% Sea h*(n) el costo del camino óptimo desde n hasta el
% objetivo.
% h es admisible si se cumple para todo n que:
% h(n) ≤ h*(n)

% Para A*, se necesita que h sea admisible
% (la distancia recta es admisible en el espacio euclídeo).

  dx = cell(2) - goal(2);
  dy = cell(1) - goal(1);

  heur = sqrt(dx^2 + dy^2);

  %% Caso para k != 1:
  k = 5;
  heur = k * heur;

  %% Caso para solo Dijkstra
  % heur = 0;
end
