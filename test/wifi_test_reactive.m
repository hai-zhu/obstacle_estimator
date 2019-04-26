%% test of the obstacle estimator, when using wifi connection
clear 
clc


%% load data
bag = rosbag('wifi_reactive.bag');
obs_bag_raw = select(bag, 'Topic', '/Target1/pose');
obs_msgs_raw= readMessages(obs_bag_raw);
obs_bag_reactive = select(bag, 'Topic', '/Target1/obstacleStateEstimation');
obs_msgs_reactive= readMessages(obs_bag_reactive);
sampleN_raw = size(obs_msgs_raw,1);
sampleN_reactive = size(obs_msgs_reactive,1);

%% time stamp
start_raw = obs_msgs_raw{1}.Header.Stamp.seconds;
end_raw   = obs_msgs_raw{sampleN_raw}.Header.Stamp.seconds;
duration_raw = end_raw - start_raw;
start_reactive = obs_msgs_reactive{1}.Header.Stamp.seconds;
end_reactive   = obs_msgs_reactive{sampleN_reactive}.Header.Stamp.seconds;
duration_reactive = end_reactive - start_reactive;
start_min = min(start_raw, start_reactive);

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


%% select and read the reactive obstacle information
pos_reactive = zeros(sampleN_reactive, 5);            % time stamp, x, y, z, norm
for i = 1 : sampleN_reactive
    pos_reactive(i, 1) = obs_msgs_reactive{i}.Header.Stamp.seconds - start_min;
    pos_reactive(i, 2) = obs_msgs_reactive{i}.Pose.Pose.Position.X;
    pos_reactive(i, 3) = obs_msgs_reactive{i}.Pose.Pose.Position.Y;
    pos_reactive(i, 4) = obs_msgs_reactive{i}.Pose.Pose.Position.Z;
    pos_reactive(i, 5) = norm(pos_reactive(i, 2:4));
end
vel_reactive = zeros(sampleN_reactive, 5);            % time stamp, x, y, z, norm
for i = 1 : sampleN_reactive
    vel_reactive(i, 1) = obs_msgs_reactive{i}.Header.Stamp.seconds - start_min;
    vel_reactive(i, 2) = obs_msgs_reactive{i}.Twist.Twist.Linear.X;
    vel_reactive(i, 3) = obs_msgs_reactive{i}.Twist.Twist.Linear.Y;
    vel_reactive(i, 4) = obs_msgs_reactive{i}.Twist.Twist.Linear.Z;
    vel_reactive(i, 5) = norm(vel_reactive(i, 2:4));
end

 
%% Compare raw and reactive position
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('position [m]');
axis([0 max(duration_raw, duration_reactive) -5.0 5.0]);
plot(pos_raw(:,1), pos_raw(:, 2), '-b');
plot(pos_reactive(:,1), pos_reactive(:, 2), '-r');
plot(pos_raw(:,1), pos_raw(:, 3), '-b');
plot(pos_reactive(:,1), pos_reactive(:, 3), '-r');
plot(pos_raw(:,1), pos_raw(:, 4), '-b');
plot(pos_reactive(:,1), pos_reactive(:, 4), '-r');
legend('raw', '100Hz');


%% Compare raw and reactive position
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('velocity [m]');
axis([0 max(duration_raw, duration_reactive) -5.0 5.0]);
plot(vel_raw(:,1), vel_raw(:, 2), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 2), '-r');
plot(vel_raw(:,1), vel_raw(:, 3), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 3), '-r');
plot(vel_raw(:,1), vel_raw(:, 4), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 4), '-r');
plot(vel_raw(:,1), vel_raw(:, 5), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 5), '-r');
legend('raw', '100Hz');

