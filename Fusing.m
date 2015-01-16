close all; clear all; clc;
rawData = open('./Scenarios_3.mat');
sensor{1} = rawData.SC.signals.values';
sensor{2} = rawData.LRR.signals.values';
sensor{3} = rawData.CR1_FL.signals.values';
sensor{4} = rawData.CR1_FR.signals.values';
GT_x = rawData.R_x;
GT_y = rawData.R_y;
nSensor = numel(sensor);
endTime = numel(sensor{1}(1,:));

dev(1) = 2;
dev(2) = 2;
dev(3) = 2;
dev(4) = 2;

vel_dev(1) = 4;
vel_dev(2) = 4;
vel_dev(3) = 4;
vel_dev(4) = 4;

for i = 1:nSensor
  for k = 1:endTime
    sensor_time{i}{k} = sensor{i}(:,k);
  end
end
b = 0;
estimate_prev = zeros(2);

%% Setup
th0 = [0 0 -60 60];          % Setup angle
range = [-200 200 -200 300];          % Plot range
pauseTime = 0.1;
dataColor = [1 0 0; 0 1 0; 0 0 1; 1 1 0 ; 0 1 1 ; 1 0 1];
offset0_x = [0 0 -0.8 0.8];  % Setup Offset X
offset0_y = [1.5 3.6 3.24 3.24]; % Setup Offset Y
lim = [22.5 45 75 75];

hFigure1 = figure(1);
axis equal; axis(range);
box on; hold on;
hTrue = gobjects(nSensor,1);
hestimate = gobjects(nSensor,1);

Plot(nSensor, th0, lim, offset0_x, offset0_y, dataColor);

hFigure2 = figure(2);

axis auto;
box on;


% [f_pos_x,f_pos_y,f_vel_x,f_vel_y] = error_model;
for i = 1:endTime
  
  for j = 1:nSensor
    r = sensor_time{j}{i}(1,:);
    th = sensor_time{j}{i}(5,:);
    x = repmat(r,2,1) .* [sind(th+th0(j)); cosd(th+th0(j))]; % Translate coordinate to XY coordinate
    y = [offset0_x(j);offset0_y(j)];
    
    real_mNoise_x{j} = random('norm', 0, dev(j), 1 , numel(x(1,:)));
    real_mNoise_y{j} = random('norm', 0, dev(j), 1 , numel(x(1,:)));
    vel_Noise_x{j} = random('norm', 0, vel_dev(j), 1 , numel(x(1,:)));
    vel_Noise_y{j} = random('norm', 0, vel_dev(j), 1 , numel(x(1,:)));
    z = x + repmat(y,1,numel(x(1,:)))+[real_mNoise_x{j};real_mNoise_y{j}];  % Reflect Offset
    %% z 값에서 오차의 평균값 빼준다. SD는 실험을 통한 구한 값을 쓴다.
    %% 거리에 따라 오차가 다르다. 따라서 z값 기준으로 제해준다.
    v = sensor_time{j}{i}(3:4,:);
%         v = [cosd(th0(j)) -sind(th0(j)) ; sind(th0(j)) cosd(th0(j))]*sensor_time{j}{i}(3:4,:) 
%     v = [cosd(-th0(j)) -sind(-th0(j)) ; sind(-th0(j)) cosd(-th0(j))]*sensor_time{j}{i}(3:4,:) 
% v = [cosd(90) -sind(90) ; sind(90) cosd(90)]*[rawData.Rdot_x(1);rawData.Rdot_y(1)]+[vel_Noise_x{j};vel_Noise_y{j}];
%     v_rot = repmat(v,2,1) * [cosd(th0(j)) -sind(th0(j)) ; sind(th0(j)) cosd(th0(j))];
    %% 속도도 마찬가지로 오차의 평균 빼준다.
    measurement{j} = [z;v];
    for k = 1:numel(measurement{j}(1,:))
      if(measurement{j}(1,k)<0)
        measurement{j}(3,k) = -measurement{j}(3,k);
      end
%       if(measurement{j}(2,k)<0)
%         measurement{j}(4,k) = -measurement{j}(4,k);
%       end
    end
        
    
    dev_x{j} = ones(1, numel(sensor_time{j}{i}(1,:)))*dev(j);
    dev_y{j} = ones(1, numel(sensor_time{j}{i}(1,:)))*dev(j);
    vel_dev_x{j} = ones(1, numel(sensor_time{j}{i}(1,:)))*vel_dev(j);
    vel_dev_y{j} = ones(1, numel(sensor_time{j}{i}(1,:)))*vel_dev(j);
    sensor_num = j*ones(1,numel(sensor_time{j}{i}(1,:)));
    measurement{j} = [measurement{j};dev_x{j};dev_y{j};vel_dev_x{j};vel_dev_y{j};0;0;sensor_num];
    
    set(0,'CurrentFigure',hFigure1);
    hTrue(j) = line(z(1,:), z(2,:));
    
    set(hTrue(j),'linestyle','none','color',dataColor(j,:),'marker','.','markersize',10);
    
  end
  
  [measurement_associate] = Associate(measurement);
  
  c = 1;
  %% Initialization
  if (b == 0)
    estimate_prev = zeros(8,1);
    for l = 1:numel(measurement_associate)
      sum_dev_x = 0; sum_dev_y = 0;
      sum_vel_dev_x = 0; sum_vel_dev_y = 0;
      for m = 1:numel(measurement_associate{l}(1,:))
        sum_dev_x = sum_dev_x + inv(measurement_associate{l}(5,m)^2);
        sum_dev_y = sum_dev_y + inv(measurement_associate{l}(6,m)^2);
        sum_vel_dev_x = sum_vel_dev_x + inv(measurement_associate{l}(7,m)^2); 
        sum_vel_dev_y = sum_vel_dev_y + inv(measurement_associate{l}(8,m)^2); 
      end
      es_x = 0, es_y = 0;
      es_vel_x = 0;, es_vel_y = 0;
      for m = 1:numel(measurement_associate{l}(1,:))
        es_x = es_x + measurement_associate{l}(1,m)*inv(measurement_associate{l}(5,m)^2)/sum_dev_x;
        es_y = es_y + measurement_associate{l}(2,m)*inv(measurement_associate{l}(6,m)^2)/sum_dev_y;
        es_vel_x = es_vel_x + measurement_associate{l}(3,m)*inv(measurement_associate{l}(7,m)^2)/sum_vel_dev_x;
        es_vel_y = es_vel_y + measurement_associate{l}(4,m)*inv(measurement_associate{l}(8,m)^2)/sum_vel_dev_y;
      end
      
      
      if(b == 0)
        estimate_prev = [es_x;es_y;es_vel_x;es_vel_y;c;0;0];
        c = c + 1;
        b = 1;
      else
        estimate_prev = [estimate_prev [es_x;es_y;es_vel_x;es_vel_y;c;0;0]];
        c = c + 1;
      end
    end

    fuse_info = gobjects(numel(estimate_prev(1,:)),1);
    fuse_ID = gobjects(numel(estimate_prev(1,:)),1);
  end
  %   estimate_prev(1:2,:)
  %% Estimate, Associate Data matching
  [estimate_mat, measurement_mat] = Matching(estimate_prev, measurement_associate);
  measurement_associate = measurement_mat;
  estimate_prev = estimate_mat; % ID : Successive ID for Associated data
  %% Kalman filter
  for k= 1:numel(estimate_prev(1,:))
    
    [estimate,b_out] = Kalman(b, measurement_associate{k}, estimate_prev);
    estimate_prev = estimate;
    
  end
  b = b_out;
  delete(fuse_info);
  delete(fuse_ID);
  for k = 1:numel(estimate_prev(1,:))
    fuse_info(k) = text(-180,-200+k*20,sprintf('ID = %d #ofSensor = %d',estimate_prev(5,k),estimate_prev(6,k)));
    fuse_ID(k) = text(estimate_prev(1,k)+10,estimate_prev(2,k)+10,sprintf('ID = %d',estimate_prev(5,k)));
  end
  %%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  est{i} = estimate_prev;
  %   delete(hestimate);
  hestimate = line(estimate_prev(1,:), estimate_prev(2,:));
  set(hestimate,'linestyle','none','color','r','marker','x','markersize',10);
%   legend([hTrue(1) hTrue(2) hTrue(3) hTrue(4) hestimate],[{'SC' 'LRR' 'FL' 'FR' 'estimate'}],'Location','northwest');
%   legend('boxoff');
  for k= 1:numel(estimate_prev(1,:))
    diff = [-GT_y(i);GT_x(i)] - estimate_prev(1:2,k);
    distance = sqrt(diff'*diff);
    if(distance < 10)
      for l = 1:numel(measurement_associate)
        if(estimate_prev(5,k) == measurement_associate{l}(10,1))
          [error_SC(i), error_LRR(i), error_FL(i), error_FR(i), error_fusion(i)]  = error_model(estimate_prev(:,k), measurement_associate{l}, -GT_y(i), GT_x(i));
          
          set(0,'CurrentFigure',hFigure2);
          plot(error_fusion,'Marker','*','Color','black');
          plot(error_SC,'Color','r');
          plot(error_FL,'Color','b');
          plot(error_LRR,'Color','g');
          plot(error_FR,'Color','y');
          hold on;
          
        end
      end
    end
  end
%   
  drawnow;
  pause(pauseTime);
end
