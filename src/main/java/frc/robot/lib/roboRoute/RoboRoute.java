package frc.robot.lib.roboRoute;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;

public class RoboRoute extends SubsystemBase{
    public static boolean isRouting;
    private RoboRouteIO io;
    private final RobotRouteIOInputsAutoLogged inputs = new RobotRouteIOInputsAutoLogged();

    private static PathConstraints constraints;

    static {
        constraints = new PathConstraints(
                DriveConstants.maxSpeedMetersPerSec * .75, 3.0,
                Math.PI * 2, Units.degreesToRadians(720));
    }

    public RoboRoute(RoboRouteIO io){
        this.io = io;

        Trigger pathTrigger = new Trigger(() -> inputs.runPath);
        Trigger poseTrigger = new Trigger(() -> inputs.runPose);
        Trigger routeTrigger = new Trigger(() -> inputs.runRoute);

        pathTrigger.onTrue(runPath(inputs.path));
        poseTrigger.onTrue(pathfindToPose(inputs.pose));
        routeTrigger.onTrue(runRoute(inputs.route));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.runEveryN(5, (Runnable) () -> Logger.processInputs("RoboRoute", inputs));
    }

    public static Command runPath(String pathName){
        isRouting = true;
        try {
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(pathName), constraints).finallyDo(() -> isRouting = false);
        } catch (FileVersionException | IOException | ParseException e) {
            // Log and return a no-op command on error
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static Command pathfindToPose(Pose2d pose){
        isRouting = true;
        return AutoBuilder.pathfindToPose(pose, constraints).finallyDo(() -> isRouting = false);
    }

    public static Command runRoute(Pose2d[] route){
        if (route.length < 2){
            return Commands.none();
        }
        isRouting = true;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(route);

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, null);
        path.preventFlipping = true;

        return AutoBuilder.pathfindThenFollowPath(path, constraints).finallyDo(() -> isRouting = false);
    }
}
