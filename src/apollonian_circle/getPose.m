function Pose = getPose(posedata)

    % [posedata,~,~] = receive(posesub,5);
    Pose(1:2,1) = [posedata.Pose.Position.X;posedata.Pose.Position.Y];
        angles = quat2eul(...
        [posedata.Pose.Orientation.W, posedata.Pose.Orientation.X, ...
        posedata.Pose.Orientation.Y, posedata.Pose.Orientation.Z]);
    Pose(3,1) = angles(1) + pi/2;
    if Pose(3,1) > pi
        Pose(3,1) = Pose(3,1) - 2*pi;
    end
end