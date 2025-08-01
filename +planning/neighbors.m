function n = neighbors(cell, map_dimensions)

  n = [];

  pos_x = cell(2);
  pos_y = cell(1);
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);
  
  %%% YOUR CODE FOR CALCULATING THE NEIGHBORS OF A CELL GOES HERE
  % Return nx2 vector with the cell coordinates of the neighbors. 
  % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
  % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]

  % Movimientos en los 8 sentidos: N, S, E, O, NE, NO, SE, SO
  directions = [ -1,  0;  % Norte
                  1,  0;  % Sur
                  0, -1;  % Oeste
                  0,  1;  % Este
                 -1, -1;  % Noroeste
                 -1,  1;  % Noreste
                  1, -1;  % Suroeste
                  1,  1]; % Sureste

  for i = 1:8
      new_y = pos_y + directions(i,1);
      new_x = pos_x + directions(i,2);

      % Verificamos que el nuevo vecino esté dentro del mapa
      if new_y >= 1 && new_y <= size_y && new_x >= 1 && new_x <= size_x
          n = [n; new_y, new_x];
      end
  end
end
