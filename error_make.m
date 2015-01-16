% function = error_make
raw_error = open('./LRR_error.mat');
long_dist = raw_error.LRR_error(:,1);
pos_x_err = raw_error.LRR_error(:,2);
pos_y_err = raw_error.LRR_error(:,3);
vel_x_err = raw_error.LRR_error(:,4);
vel_y_err = raw_error.LRR_error(:,5);
dev_pos_x = raw_error.LRR_error(:,6);
dev_pos_y = raw_error.LRR_error(:,7);
dev_vel_x = raw_error.LRR_error(:,8);
dev_vel_y = raw_error.LRR_error(:,9);

LRR_f_pos_x = fit(long_dist,pos_x_err,'poly2');
LRR_f_pos_y = fit(long_dist,pos_y_err,'poly2');
LRR_f_vel_x = fit(long_dist,vel_x_err,'poly2');
LRR_f_vel_y = fit(long_dist,vel_y_err,'poly2');
LRR_f_dev_pos_x = fit(long_dist,dev_pos_x,'poly2');
LRR_f_dev_pos_y = fit(long_dist,dev_pos_y,'poly2');
LRR_f_dev_vel_x = fit(long_dist,dev_vel_x,'poly2');
LRR_f_dev_vel_y = fit(long_dist,dev_vel_y,'poly2');
% end
