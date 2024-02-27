
%% getting debugging data
addpath(genpath('Functions'));

filePath = '/home/savanna-runner/log_data/yr/oldfoot.mat';

allData = load(filePath);


dt = 0.0005;
T=8;

Time = dt*double(allData.SAVE_cnt_global);

time_length = size(Time,1);
StatusWord = allData.elmo_data(1:12,:);
ControlWord = allData.elmo_data(13:24,:);
ErrorCode = allData.elmo_data(25:36,:);

Gyrometer = allData.sensor_data(1:3,:);
Accelerometer = allData.sensor_data(4:6,:);
Encoder_Position = allData.sensor_data(7:18,:);
Encoder_Velocity = allData.sensor_data(19:30,:);
Real_Torque = allData.sensor_data(31:42,:);

FootPosition = allData.cartesian(1:12,:);
FootVelocity= allData.cartesian(13:24,:);
FootPosition_desired = allData.cartesian_desired(1:12,:);
FootVelocity_desired = allData.cartesian_desired(13:24,:);
FSM = allData.fsm_controller_output_FSM;
Contact = allData.x_estimated_contact;
TrueContact = allData.x_true_trueContact;
WKC = allData.wkc;


X = allData.x_estimated_X;
Xd = allData.x_desired_X_d;
X_True = allData.x_true_X;
% R, Rd, R_true is from X, Xd, X_True.
% X, Xd, X_True is from x_estimated_X, x_desired_X_d, x_true_X.
% X: estimated rotation matrix(with vector form)
% Xd: desired rotation matrix(with vector form)
% X_True: rotation matrix from Raisim(with vector form)


Torque_Command_PD = allData.torque_command(1:12,:);
Torque_Command_Controller = allData.torque_command(13:24,:);
Torque_Command_NMPC = allData.torque_command(25:36,:);
Torque_Command_RL = allData.torque_command(37:48,:);
action_rl = allData.action_stair;

Force_Command_PD = allData.force_command(1:12,:);
Force_Command_NMPC = allData.force_command(13:24,:);

joint_position_desired = allData.joint_desired(1:12,:);
joint_velocity_desired = allData.joint_desired(13:24,:);

GPS_raw = allData.gps_log(1:3,:);
GPS_init = allData.gps_log(4:6,:);
GPS_cov_sqrt = allData.gps_log(7:9,:);
GPS_in = allData.gps_log(10,:);

%barrier_estimator = allData.gj_estimator_output;

thread_name = ["EtherCAT","Raisim Simulation","Raisim Visualization","Main Controller","FSM Controller", "estimation", "imu", "ROS", "Dummy", "Camera", "Mapping"];

thread_time = allData.thread_time;


R_vec = X(1:9,:);
Rd_vec = Xd(1:9,:);
R_true_vec = X_True(1:9,:);
% R, Rd, R_true is from X, Xd, X_True.

for i=1:size(R_vec,2)
    
    R(1,1,i) = R_vec(1,i);
    R(2,1,i) = R_vec(2,i);
    R(3,1,i) = R_vec(3,i);
    R(1,2,i) = R_vec(4,i);
    R(2,2,i) = R_vec(5,i);
    R(3,2,i) = R_vec(6,i);
    R(1,3,i) = R_vec(7,i);
    R(2,3,i) = R_vec(8,i);
    R(3,3,i) = R_vec(9,i);
    
    
    Rd(1,1,i) = Rd_vec(1,i);
    Rd(2,1,i) = Rd_vec(2,i);
    Rd(3,1,i) = Rd_vec(3,i);
    Rd(1,2,i) = Rd_vec(4,i);
    Rd(2,2,i) = Rd_vec(5,i);
    Rd(3,2,i) = Rd_vec(6,i);
    Rd(1,3,i) = Rd_vec(7,i);
    Rd(2,3,i) = Rd_vec(8,i);
    Rd(3,3,i) = Rd_vec(9,i);
    
    R_True(1,1,i) = R_true_vec(1,i);
    R_True(2,1,i) = R_true_vec(2,i);
    R_True(3,1,i) = R_true_vec(3,i);
    R_True(1,2,i) = R_true_vec(4,i);
    R_True(2,2,i) = R_true_vec(5,i);
    R_True(3,2,i) = R_true_vec(6,i);
    R_True(1,3,i) = R_true_vec(7,i);
    R_True(2,3,i) = R_true_vec(8,i);
    R_True(3,3,i) = R_true_vec(9,i);

end
%

index = 1:size(Time,2);

% time_start = 14;
% initial_time = Time(1);
% time_duration = 1.0;
% 
% index = ((time_start-initial_time)/0.0005 : (time_start-initial_time)/0.0005 + time_duration/0.0005);


%% caculate data.

figure(1);
plot(Time, FootVelocity(1,:)');

qtn = zeros(4,length(Time));
q = zeros(19,length(Time));
q_dot = zeros(18,length(Time));

R = zeros(3,3,length(Time));
for i = 1:length(Time)
    R(1,1,i) = R_vec(1,i);
    R(2,1,i) = R_vec(2,i);
    R(3,1,i) = R_vec(3,i);
    R(1,2,i) = R_vec(4,i);
    R(2,2,i) = R_vec(5,i);
    R(3,2,i) = R_vec(6,i);
    R(1,3,i) = R_vec(7,i);
    R(2,3,i) = R_vec(8,i);
    R(3,3,i) = R_vec(9,i);
end

% rotation matrix to quarternion.
for i = 1:length(Time)
    qtn(:,i) = (rotm2quat(R(:,:,i)))';
end

% Save generalized coordinate.
q(4:7,:) = qtn;
q(8:19,:) = Encoder_Position;

Q = zeros(12,length(Time));
q_lpf = zeros(12,length(Time));

fs = 2000;
f_cut = 500;
w = (fs/length(Time))*(1:length(Time));
cut_index = round(f_cut/fs*numel(Time));

for i = 1:12
    Q(i,:) = fft(Encoder_Position(i,:));
    Q(i,cut_index+2:end-cut_index) = 0;
    q_lpf(i,:) = ifft(Q(i,:));
end

% figure(); hold on;
% plot(Encoder_Position(3,:),'k');
% plot(q_lpf(3,:),'r');

Joint_Velocity = zeros(12,length(Time));

for i = 2:length(Time)
    Joint_Velocity(:,i) = (q_lpf(:,i)-q_lpf(:,i-1))/dt;
end

% Save generalized velocity.
q_dot(4:6,:) = Gyrometer;
q_dot(7:18,:) = Encoder_Velocity;

% figure(); hold on;
% plot(Encoder_Velocity(2,:),'k');
% plot(Joint_Velocity(2,:),'r');

% figure(3); hold on;
% plot(Gyrometer(1,:),'r');
% plot(Gyrometer(2,:),'g');
% plot(Gyrometer(3,:),'b');
% 
% plot(gyro_lpf(1,:),'r');
% plot(gyro_lpf(2,:),'g');
% plot(gyro_lpf(3,:),'b');


% q_ddot vector.
% Linear accleration.

acc = Accelerometer;
acc(3,:) = acc(3,:) - 9.8;

% Angular acceleration.
angular_vel = Gyrometer;
angular_acc = zeros(3,length(Time));

for i = 2:length(Time)
    angular_acc(:,i) = (angular_vel(:,i)-angular_vel(:,i-1))/dt;
end


q_ddot(1:3,:) = acc;
q_ddot(4:6,:) = angular_acc;

fs = 2000;
f_cut = 500;
%w = (fs/length(Time))*(1:length(Time));
cut_index = round(f_cut/fs*numel(Time));

ACC = zeros(6,length(Time));
acc_lpf = zeros(6,length(Time));

for i = 1:6
    ACC(i,:) = fft(q_ddot(i,:));
    ACC(i,cut_index+2:end-cut_index) = 0;
    acc_lpf(i,:) = ifft(ACC(i,:));
end


q_ddot(1:6,:) = acc_lpf;
%q_ddot(1:6,:) = 0;


% Save joint acceleration.
for i = 2:length(Time)
    q_ddot(7:18,i) = (Encoder_Velocity(:,i)-Encoder_Velocity(:,i-1))/dt;
end

fs = 2000;
f_cut = 500;
w = (fs/length(Time))*(1:length(Time));
cut_index = round(f_cut/fs*numel(Time));

ACC = zeros(12,length(Time));
acc_lpf = zeros(12,length(Time));

for i = 1:12
    ACC(i,:) = fft(q_ddot(i+6,:));
    ACC(i,cut_index+2:end-cut_index) = 0;
    acc_lpf(i,:) = ifft(ACC(i,:));
end

q_ddot(7:18,:) = acc_lpf;

%% save data for raisim as txt.
q_output = q';

q_dot_output = q_dot';

torque_output = Real_Torque';
torque_output = [zeros(length(Time),6), torque_output];

q_ddot_output = q_ddot';

writematrix(q_output, 'generalized_position.txt', 'Delimiter', '\t');
writematrix(q_dot_output, 'generalized_velocity.txt', 'Delimiter', '\t');
writematrix(q_ddot_output, 'generalized_acceleration.txt', 'Delimiter', '\t');
writematrix(torque_output, 'generalized_torque.txt', 'Delimiter', '\t');


%%
idx_fig = [3 8];
figure(); 
subplot(2,1,1);
plot(Time, Accelerometer(3,:)');
xlim(idx_fig);

subplot(2,1,2);
plot(Time, FootVelocity(1,:)');
xlim(idx_fig);
