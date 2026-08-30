function egoPose = egoBusToScenarioPose(actor_pose)
egoPose.ActorID = uint32(1);
egoPose.Position = [actor_pose.x, actor_pose.y, 0];
egoPose.Velocity = [actor_pose.velocity, 0, 0];
egoPose.Roll = 0;
egoPose.Pitch = 0;
egoPose.Yaw = actor_pose.yaw * 180/pi;
egoPose.AngularVelocity = [0, 0, 0];
end