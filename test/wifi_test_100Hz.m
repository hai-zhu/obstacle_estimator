%% test of the obstacle estimator, when using wifi connection
clear 
clc


%% load data
bag = rosbag('wifi_100Hz.bag');
obs_bag_raw = select(bag, 'Topic', '/Target1/pose');
obs_msgs_raw= readMessages(obs_bag_raw);
obs_bag_100Hz = select(bag, 'Topic', '/Target1/obstacleStateEstimation');
obs_msgs_100Hz= readMessages(obs_bag_100Hz);
sampleN_raw = size(obs_msgs_raw,1);
sampleN_100Hz = size(obs_msgs_100Hz,1);

%% time stamp
start_raw = obs_msgs_raw{1}.Header.Stamp.seconds;
end_raw   = obs_msgs_raw{sampleN_raw}.Header.Stamp.seconds;
duration_raw = end_raw - start_raw;
start_100Hz = obs_msgs_100Hz{1}.Header.Stamp.seconds;
end_100Hz   = obs_msgs_100Hz{sampleN_100Hz}.Header.Stamp.seconds;
duration_100Hz = end_100Hz - start_100Hz;
start_min = min(start_raw, start_100Hz);

%% select and read the raw obstacle information
pos_raw = zeros(sampleN_raw, 5);            % time stamp, x, y, z, norm
for i = 1 : sampleN_raw
    pos_raw(i, 1) = obs_msgs_raw{i}.Header.Stamp.seconds - start_min;
    pos_raw(i, 2) = obs_msgs_raw{i}.Pose.Position.X;
    pos_raw(i, 3) = obs_msgs_raw{i}.Pose.Position.Y;
    pos_raw(i, 4) = obs_msgs_raw{i}.Pose.Position.Z;
    pos_raw(i, 5) = norm(pos_raw(i, 2:4));
end


%% get velocity information using simple difference
vel_raw = zeros(sampleN_raw, 5);            % time stamp, vx, vy, vz, norm
for i = 1 : sampleN_raw-1
    vel_raw(i, 1) = pos_raw(i, 1);
    dt_temp  = pos_raw(i+1, 1) - pos_raw(i, 1);
    vel_raw(i, 2:4) = (pos_raw(i+1, 2:4) - pos_raw(i, 2:4))/dt_temp;
    vel_raw(i, 5) = norm(vel_raw(i, 2:4));
end
vel_raw(sampleN_raw, :) = vel_raw(sampleN_raw-1, :);


%% plot raw position data
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('position [m]');
axis([0 duration_raw -5.0 5.0]);
plot(pos_raw(:,1), pos_raw(:, 2), '-b');
plot(pos_raw(:,1), pos_raw(:, 3), '-k');
plot(pos_raw(:,1), pos_raw(:, 4), '-g');
plot(pos_raw(:,1), pos_raw(:, 5), '-r');
legend('x','y','z','dis');


%% plot raw velocity data
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('velocity [m/s]');
axis([0 duration_raw -5.0 5.0]);
plot(vel_raw(:,1), vel_raw(:, 2), '-b');
plot(vel_raw(:,1), vel_raw(:, 3), '-k');
plot(vel_raw(:,1), vel_raw(:, 4), '-g');
plot(vel_raw(:,1), vel_raw(:, 5), '-r');
legend('vx','vy','vz','vel');


%% select and read the 100Hz obstacle information
pos_100Hz = zeros(sampleN_100Hz, 5);            % time stamp, x, y, z, norm
for i = 1 : sampleN_100Hz
    pos_100Hz(i, 1) = obs_msgs_100Hz{i}.Header.Stamp.seconds - start_min;
    pos_100Hz(i, 2) = obs_msgs_100Hz{i}.Pose.Pose.Position.X;
    pos_100Hz(i, 3) = obs_msgs_100Hz{i}.Pose.Pose.Position.Y;
    pos_100Hz(i, 4) = obs_msgs_100Hz{i}.Pose.Pose.Position.Z;
    pos_100Hz(i, 5) = norm(pos_100Hz(i, 2:4));
end
vel_100Hz = zeros(sampleN_100Hz, 5);            % time stamp, x, y, z, norm
for i = 1 : sampleN_100Hz
    vel_100Hz(i, 1) = obs_msgs_100Hz{i}.Header.Stamp.seconds - start_min;
    vel_100Hz(i, 2) = obs_msgs_100Hz{i}.Twist.Twist.Linear.X;
    vel_100Hz(i, 3) = obs_msgs_100Hz{i}.Twist.Twist.Linear.Y;
    vel_100Hz(i, 4) = obs_msgs_100Hz{i}.Twist.Twist.Linear.Z;
    vel_100Hz(i, 5) = norm(vel_100Hz(i, 2:4));
end

 
%% Compare raw and 100Hz position
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('position [m]');
axis([0 max(duration_raw, duration_100Hz) -5.0 5.0]);
plot(pos_raw(:,1), pos_raw(:, 2), '-b');
plot(pos_100Hz(:,1), pos_100Hz(:, 2), '-r');
plot(pos_raw(:,1), pos_raw(:, 3), '-b');
plot(pos_100Hz(:,1), pos_100Hz(:, 3), '-r');
plot(pos_raw(:,1), pos_raw(:, 4), '-b');
plot(pos_100Hz(:,1), pos_100Hz(:, 4), '-r');
legend('raw', '100Hz');


%% Compare raw and 100Hz position
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('velocity [m]');
axis([0 max(duration_raw, duration_100Hz) -5.0 5.0]);
% plot(vel_raw(:,1), vel_raw(:, 2), '-b');
plot(vel_100Hz(:,1), vel_100Hz(:, 2), '-r');
% plot(vel_raw(:,1), vel_raw(:, 3), '-b');
plot(vel_100Hz(:,1), vel_100Hz(:, 3), '-r');
% plot(vel_raw(:,1), vel_raw(:, 4), '-b');
plot(vel_100Hz(:,1), vel_100Hz(:, 4), '-r');
% plot(vel_raw(:,1), vel_raw(:, 5), '-b');
plot(vel_100Hz(:,1), vel_100Hz(:, 5), '-r');
legend('raw', '100Hz');

