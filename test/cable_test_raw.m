%% test of the obstacle estimator, when using cable connection
clear 
clc


%% load raw data
bag = rosbag('cable_bag.bag');
startTime = bag.StartTime;              % s
endTime   = bag.EndTime;                % s
duration  = endTime - startTime;        % s


%% choose a test duration
testStart = startTime;
testEnd   = endTime;
testDuration = testEnd - testStart;


%% select and read the raw obstacle information
obs_bag = select(bag, 'Topic', '/Target1/pose');
obs_msgs= readMessages(obs_bag);
sampleN = size(obs_msgs,1);
pos_raw = zeros(sampleN, 5);            % time stamp, x, y, z, norm
for i = 1 : sampleN
    pos_raw(i, 1) = obs_msgs{i}.Header.Stamp.seconds - testStart;
    pos_raw(i, 2) = obs_msgs{i}.Pose.Position.X;
    pos_raw(i, 3) = obs_msgs{i}.Pose.Position.Y;
    pos_raw(i, 4) = obs_msgs{i}.Pose.Position.Z;
    pos_raw(i, 5) = norm(pos_raw(i, 2:4));
end


%% get velocity information using simple difference
vel_raw = zeros(sampleN, 5);            % time stamp, vx, vy, vz, norm
for i = 1 : sampleN-1
    vel_raw(i, 1) = pos_raw(i, 1);
    dt_temp  = pos_raw(i+1, 1) - pos_raw(i, 1);
    vel_raw(i, 2:4) = (pos_raw(i+1, 2:4) - pos_raw(i, 2:4))/dt_temp;
    vel_raw(i, 5) = norm(vel_raw(i, 2:4));
end
vel_raw(sampleN, :) = vel_raw(sampleN-1, :);


%% plot raw position data
figure;
hold on;
grid on;
box on;
xlabel('time stamp [s]');
ylabel('position [m]');
axis([0 testDuration -5.0 5.0]);
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
axis([0 testDuration -5.0 5.0]);
plot(vel_raw(:,1), vel_raw(:, 2), '-b');
plot(vel_raw(:,1), vel_raw(:, 3), '-k');
plot(vel_raw(:,1), vel_raw(:, 4), '-g');
plot(vel_raw(:,1), vel_raw(:, 5), '-r');
legend('vx','vy','vz','vel');

