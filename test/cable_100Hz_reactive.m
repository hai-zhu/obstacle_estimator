%% test of the obstacle estimator, when using cable connection
clear 
clc


%% load data
bag = rosbag('cable_reactive.bag');
obs_bag_reactive = select(bag, 'Topic', '/Target1/obstacleStateEstimation');
obs_msgs_reactive= readMessages(obs_bag_reactive);
bag_100Hz = rosbag('cable_100Hz.bag');
obs_bag_100Hz = select(bag_100Hz, 'Topic', '/Target1/obstacleStateEstimation');
obs_msgs_100Hz= readMessages(obs_bag_100Hz);
sampleN_reactive = size(obs_msgs_reactive,1);
sampleN_100Hz = size(obs_msgs_100Hz,1);

%% time stamp
start_reactive = obs_msgs_reactive{1}.Header.Stamp.seconds;
end_reactive   = obs_msgs_reactive{sampleN_reactive}.Header.Stamp.seconds;
duration_reactive = end_reactive - start_reactive;
start_100Hz = obs_msgs_100Hz{1}.Header.Stamp.seconds;
end_100Hz   = obs_msgs_100Hz{sampleN_100Hz}.Header.Stamp.seconds;
duration_100Hz = end_100Hz - start_100Hz;
start_min = min(start_reactive, start_100Hz);


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

 
%% Compare 100Hz and reactive position
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('position [m]');
axis([0 max(duration_100Hz, duration_reactive) -5.0 5.0]);
plot(pos_100Hz(:,1), pos_100Hz(:, 2), '-b');
plot(pos_reactive(:,1), pos_reactive(:, 2), '-r');
plot(pos_100Hz(:,1), pos_100Hz(:, 3), '-b');
plot(pos_reactive(:,1), pos_reactive(:, 3), '-r');
plot(pos_100Hz(:,1), pos_100Hz(:, 4), '-b');
plot(pos_reactive(:,1), pos_reactive(:, 4), '-r');
legend('raw', '100Hz');


%% Compare 100Hz and reactive position
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('velocity [m]');
axis([0 max(duration_100Hz, duration_reactive) -5.0 5.0]);
plot(vel_100Hz(:,1), vel_100Hz(:, 2), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 2), '-r');
plot(vel_100Hz(:,1), vel_100Hz(:, 3), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 3), '-r');
plot(vel_100Hz(:,1), vel_100Hz(:, 4), '-b');
plot(vel_reactive(:,1), vel_reactive(:, 4), '-r');
legend('raw', '100Hz');

