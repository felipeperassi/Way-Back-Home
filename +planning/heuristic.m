function heur = heuristic(cell, goal)
  % HEURISTIC Calcula la heurística para el algoritmo A*.
  %
  % Entradas:
  %   cell - Coordenadas de la celda actual [row, col]
  %   goal - Coordenadas de la celda objetivo [row, col]
  % Salidas:
  %   heur - Valor de la heurística (distancia euclidiana)

  % Calcula la distancia euclidiana entre la celda actual y el objetivo
  dx = cell(2) - goal(2);
  dy = cell(1) - goal(1);

  % Heurística: distancia euclidiana
  heur = sqrt(dx^2 + dy^2);

  %% Caso para k != 1:
  k = 5;
  heur = k * heur;

  %% Caso para solo Dijkstra
  % heur = 0;
end
