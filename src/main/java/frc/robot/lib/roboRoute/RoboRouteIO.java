package frc.robot.lib.roboRoute;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface RoboRouteIO {
    @AutoLog
    public static class RobotRouteIOInputs {
        // Triggers
        public boolean runPath = false;
        public boolean runPose = false;
        public boolean runRoute = false;

        // Action data
        public String path = "";
        public Pose2d pose = new Pose2d();
        public Pose2d[] route = new Pose2d[0];
    }

    public default void updateInputs(RobotRouteIOInputs inputs) {}
}