function [error_SC, error_LRR, error_FL, error_FR, error_fusion]  = error_model(estimate_prev, measurement_associate, GT_x, GT_y, i)
error_SC = 0;
error_LRR = 0;
error_FL = 0;
error_FR = 0;
for k = 1:numel(measurement_associate(1,:))
  diff = (measurement_associate(1:2,k) - [GT_x;GT_y]);
  if(measurement_associate(11,k) == 1)
    error_SC = sqrt(diff'*diff);
  elseif(measurement_associate(11,k) == 2)
    error_LRR = sqrt(diff'*diff);
  elseif(measurement_associate(11,k) == 3)
    error_FL = sqrt(diff'*diff);
  elseif(measurement_associate(11,k) == 4)
    error_FR = sqrt(diff'*diff);
  end
end

  diff_1 = estimate_prev(1:2,:) - [GT_x;GT_y];
  error_fusion = sqrt(diff_1'*diff_1);
end