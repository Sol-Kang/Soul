function [estimate_final,b_out] = Kalman(b, measurement_associate, estimate_prev)
persistent A H P a

for l = 1:numel(estimate_prev(1,:))
  if(estimate_prev(5,l) == measurement_associate(10,1))
    estimate = estimate_prev(:,l);
    B = l;
  end
end
if(estimate(7,1) == 0)
  P{estimate(5,1)} = 8.0*eye(4);
end
% initialization
if (b == 1)
  A = [1 0 0.025 0 ; 0 1 0 0.025 ; 0 0 1 0 ; 0 0 0 1];
  H = 1.0*eye(4);
  P{estimate(5,1)} = 8.0*eye(4);
  estimate(8,1) = 0;
  b = b+1;
  
end

%% Time Update
  predicted = A*estimate(1:4,:);
% predicted = estimate(1:2,:) + [mean(measurement_associate(3,:))*0.025;mean(measurement_associate(4,:))*0.025];


Ppre = A*P{estimate(5,1)}*A';

%% Measurement Update
% Make kalman gain using Time updated data

%% %%%%%%%%%%%%%%%%%%%%%%%%%
%% model 1
for k = 1:numel(measurement_associate(1,:))
  Rk = [measurement_associate(5,k) 0 0 0; 0 measurement_associate(6,k) 0 0;0 0 measurement_associate(7,k) 0 ; 0 0 0 measurement_associate(8,k) ]; %Covariance matrix
  K{k} = Ppre/(Ppre + (Rk*Rk')); 
  
  
end

% Make estimation data using kalman gain
est = predicted;
temp_2 = 0;
for k = 1:numel(measurement_associate(1,:))
  temp_2 = temp_2+K{k}*(measurement_associate(1:4,k) - predicted(1:4,:));
end
est = predicted+temp_2/ numel(measurement_associate(1,:));

estimate_prev(:,B) = [est;estimate_prev(5:7,B)];
estimate_final = estimate_prev;
b_out = b;
temp_1 = 0;
for k = 1:numel(measurement_associate(1,:))
  temp_1 = temp_1 + K{k};
%   Rk = [measurement_associate(5,k) 0 0 0; 0 measurement_associate(6,k) 0 0;0 0 measurement_associate(7,k) 0 ; 0 0 0 measurement_associate(8,k) ]; %Covariance matrix
%   temp_1 = temp_1 + (Rk*Rk');
end
P{estimate(5,1)} = Ppre-temp_1*Ppre/numel(measurement_associate(1,:));%inv(inv(Ppre) + temp_1);
end
