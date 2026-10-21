package frc.robot.lib.roboRoute;

import org.littletonrobotics.junction.AutoLog;

public interface RoboRouteIO {
    @AutoLog
    public static class RobotRouteIOInputs {
        public boolean run = false;
        public String plan = "";
    }

    public default void updateInputs(RobotRouteIOInputs inputs) {}
}