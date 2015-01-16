function [estimate_mat, measurement_mat] = Matching(estimate_prev, measurement_associate)

a = 0;
for i = 1:numel(measurement_associate)
  
  
  sum_dev_x = 0; sum_dev_y = 0;
  sum_vel_dev_x = 0; sum_vel_dev_y = 0;
      for m = 1:numel(measurement_associate{i}(1,:))
        sum_dev_x = sum_dev_x + 1/measurement_associate{i}(5,m)^2;
        sum_dev_y = sum_dev_y + 1/measurement_associate{i}(6,m)^2;
        sum_vel_dev_x = sum_vel_dev_x + inv(measurement_associate{i}(7,m)^2); 
        sum_vel_dev_y = sum_vel_dev_y + inv(measurement_associate{i}(8,m)^2); 
      end
      es_x{i} = 0; es_y{i} = 0;
      es_vel_x{i} = 0; es_vel_y{i} = 0;
      for m = 1:numel(measurement_associate{i}(1,:))
        es_x{i} = es_x{i} + measurement_associate{i}(1,m)*inv(measurement_associate{i}(5,m)^2)/sum_dev_x;
        es_y{i} = es_y{i} + measurement_associate{i}(2,m)*inv(measurement_associate{i}(6,m)^2)/sum_dev_y;    
        es_vel_x{i} = es_vel_x{i} + measurement_associate{i}(3,m)*inv(measurement_associate{i}(7,m)^2)/sum_vel_dev_x;
        es_vel_y{i} = es_vel_y{i} + measurement_associate{i}(4,m)*inv(measurement_associate{i}(8,m)^2)/sum_vel_dev_y;
      end
  for k = 1:numel(measurement_associate{i}(1,:))
    measurement_associate{i}(10,k) = 0;
  end
end
for i = 1:numel(estimate_prev(1,:))
  estimate_prev(6,i) = 0;
end

for i = 1:numel(measurement_associate)
  for j = 1:numel(estimate_prev(1,:))
    diff = [es_x{i};es_y{i}] - estimate_prev(1:2,j);
    distance = sqrt(diff'*diff);
    if(distance < 5 & measurement_associate{i}(10,1) == 0 & estimate_prev(6,j) == 0)
      for k = 1:numel(measurement_associate{i}(1,:))
        measurement_associate{i}(10,k) = estimate_prev(5,j);
      end
      estimate_prev(6,j) = 1;
      estimate_prev(8,j) = 1;
    end
  end
end


b = numel(estimate_prev(1,:));
for i = 1:numel(estimate_prev(1,:))
  if(estimate_prev(6,b-i+1) == 0)
    estimate_prev(:,b-i+1) = [];
  end
end

for i = 1:numel(measurement_associate)
  for l = 1:numel(measurement_associate)
    if(measurement_associate{i}(10,1) == 0)
      if(sum(estimate_prev(5,:)==l) == 0)
        estimate_prev = [estimate_prev [es_x{i};es_y{i};es_vel_x{i} ;es_vel_y{i} ;l;1;0;0] ];
        for k = 1:numel(measurement_associate{i}(1,:))
          measurement_associate{i}(10,k) = l;
        end
      end
    end
  end
end

for i = 1:numel(estimate_prev(1,:))
  for j = 1:numel(measurement_associate)
    if(estimate_prev(5,i) == measurement_associate{j}(10,1))
      estimate_prev(7,i) = numel(measurement_associate{j}(1,:));
    end
  end
end

estimate_prev(6,:) = [];
estimate_mat = estimate_prev;
measurement_mat = measurement_associate;

end
