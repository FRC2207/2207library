package frc.robot.lib.roboRoute;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;

public class RoboRouteIONetworkTables implements RoboRouteIO {
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("RoboRoute");

    private final BooleanSubscriber runPathSubscriber;
    private final BooleanSubscriber runPoseSubscriber;
    private final BooleanSubscriber runRouteSubscriber;

    private final StringSubscriber pathSubscriber;
    private final StructSubscriber<Pose2d> poseSubscriber;
    private final StructArraySubscriber<Pose2d> routeSubscriber;

    public RoboRouteIONetworkTables() {
        // Triggers
        runPathSubscriber = table.getBooleanTopic("RunPath").subscribe(false);
        runPoseSubscriber = table.getBooleanTopic("RunPose").subscribe(false);
        runRouteSubscriber = table.getBooleanTopic("RunRoute").subscribe(false);

        // Action data
        pathSubscriber = table.getStringTopic("Path").subscribe("");
        poseSubscriber = table.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d());
        routeSubscriber = table.getStructArrayTopic("Route", Pose2d.struct).subscribe(new Pose2d[0]);
    }

    public void updateInputs(RobotRouteIOInputs inputs) {
        inputs.runPath = runPathSubscriber.get();
        inputs.runPose = runPoseSubscriber.get();
        inputs.runRoute = runRouteSubscriber.get();

        inputs.path = pathSubscriber.get();
        inputs.pose = poseSubscriber.get();
        inputs.route = routeSubscriber.get();
    }
}
