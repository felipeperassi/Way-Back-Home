function n = neighbors(cell, map_dimensions)
  % NEIGHBORS - Genera las celdas vecinas válidas de una celda dada en un mapa 2D.
  %
  % Entradas:
  %   cell            - Coordenadas de la celda actual [row, col]
  %   map_dimensions  - Dimensiones del mapa [n_rows, n_cols]
  %
  % Salidas:
  %   n               - Matriz N x 2 con las coordenadas de las celdas vecinas válidas [row, col]

  % Inicializa la lista de vecinos
  n = [];

  % Coordenadas de la celda actual
  pos_x = cell(2);
  pos_y = cell(1);

  % Dimensiones del mapa
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);

  % Movimientos en los 8 sentidos: N, S, E, O, NE, NO, SE, SO
  directions = [ -1,  0;  % Norte
                  1,  0;  % Sur
                  0, -1;  % Oeste
                  0,  1;  % Este
                -1, -1;  % Noroeste
                -1,  1;  % Noreste
                  1, -1;  % Suroeste
                  1,  1]; % Sureste

  % Itera sobre cada dirección para calcular las coordenadas de los vecinos
  % y verifica que estén dentro de los límites del mapa
  for i = 1:8
    % Calcula las nuevas coordenadas del vecino
    new_y = pos_y + directions(i,1);
    new_x = pos_x + directions(i,2);

    % Se verifica que el nuevo vecino esté dentro del mapa
    if new_y >= 1 && new_y <= size_y && new_x >= 1 && new_x <= size_x
      n = [n; new_y, new_x];
    end
  end
end
