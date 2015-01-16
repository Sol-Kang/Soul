function [state_associate] = Associate(state)

%% Initialize Association set number


a = 1;

%% Associate
for k = 1:(numel(state)-1)
  for j = k+1:numel(state) % Sensors are compared each others
    d = pdist2(state{k}(1:2,:)', state{j}(1:2,:)','euclidean'); % Calculate distace between each sensor's data set
%     c = d < 15;  % Range
    [row, col] = find(d < 5);
    if(numel(row)>0 && numel(col)>0)
      for l = 1:numel(row)
        if(state{k}(9,row(l)) == 0 & state{j}(9,col(l)) == 0 ) % Is not assigned Group number
          state{k}(9,row(l)) = a;  % Group number assign
          state{j}(9,col(l)) = a;
          state_associate{a} = [state{k}(:,row(l)) state{j}(:,col(l))]; % Gather data set that have same group number
          a = a + 1;
          
        elseif(state{k}(9,row(l)) ~= 0 & state{j}(9,col(l)) == 0) % One is already assigned Group number
          state{j}(9,col(l)) = state{k}(9,row(l));
          state_associate{state{k}(9,row(l))} = [state_associate{state{k}(9,row(l))} state{j}(:,col(l))];
          
        elseif(state{j}(9,col(l)) ~= 0 & state{k}(9,row(l)) == 0) % One is already assigned Group number
          state{k}(9,row(l)) = state{j}(9,col(l));
          state_associate{state{j}(9,col(l))} = [state_associate{state{j}(9,col(l))} state{k}(:,row(l))];
        else
          
        end
      end
    end
  end
end


%% Single associate
% If any detected object appeared, assign single associate
for k = 1:numel(state)
  for j = 1:numel(state{k}(1,:))
    if(state{k}(9,j) == 0)
      state{k}(9,j) = a;
      state_associate{a} = state{k}(:,j);
      a = a+1;
    end
  end
end
end