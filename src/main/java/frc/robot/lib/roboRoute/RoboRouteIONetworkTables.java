package frc.robot.lib.roboRoute;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;

public class RoboRouteIONetworkTables implements RoboRouteIO {
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("RoboRoute");

    private final StringSubscriber planSubscriber;

    public RoboRouteIONetworkTables() {
        planSubscriber = table
        .getStringTopic("Plan").subscribe("");
    }

    public void updateInputs(RobotRouteIOInputs inputs) {
        TimestampedString[] updates = planSubscriber.readQueue();

        inputs.run = updates.length > 0;

        inputs.plan = planSubscriber.get();
    }
}
