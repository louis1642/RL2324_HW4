close all
clear
clc

t_end = plotErrorAndTorques("hw4/goals" + ".bag");



function t_end = plotErrorAndTorques(bagname)
    % load ROS bag
    bag = rosbag(bagname);
    pose = bag.select("Topic", "/fra2mo/pose");
    poseTs = timeseries(pose);
    t = poseTs.Time - poseTs.Time(1);
    x = poseTs.Data(:, 4);
    y = poseTs.Data(:, 5);
    % z = poseTs.Data(:, 6);
    yaw = quat2eul([poseTs.Data(:, 10), poseTs.Data(:, 7:9)]);
    yaw = yaw(:, 1);

    % position
    figure(Name="Position over time")
    plot(t, x, "DisplayName", "x(t)")
    hold on
    plot(t, y, "DisplayName", "y(t)")
    legend("location","best")
    grid on
    title("Position over time")
    xlabel("Time [s]")


    % trajectory
    idxs = 1:round(length(x)/50):length(x);
    ql = 0.5;
    qx = x(idxs);
    qy = y(idxs);
    qu = cos(yaw(idxs));
    qv = sin(yaw(idxs));
    figure(Name="Trajectory")
    im = imread("gazebo_world.png");
    plot(x, y)
    hold on
    quiver(qx, qy, qu, qv, ql)
    impl = image([-22.6, 2.4], [12.4, -3.2], im);
    impl.AlphaData = 0.5;
    uistack(impl, "bottom")
    grid on
    title("Trajectory")
    axis equal

    % yaw
    figure(Name="Yaw over time")
    plot(t, yaw, "DisplayName", "Yaw")
    xlabel("Time [s]")
    grid on
    legend("Location","ne")
    title("Yaw over time")
    
    t_end = poseTs.Time(end);
end