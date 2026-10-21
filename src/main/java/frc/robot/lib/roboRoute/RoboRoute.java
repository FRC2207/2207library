package frc.robot.lib.roboRoute;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
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

public class RoboRoute extends SubsystemBase {
    public static boolean isRouting;
    private RoboRouteIO io;
    private final RobotRouteIOInputsAutoLogged inputs = new RobotRouteIOInputsAutoLogged();

    private static PathConstraints constraints;

    static {
        constraints = new PathConstraints(
                DriveConstants.maxSpeedMetersPerSec * .75, 3.0,
                Math.PI * 2, Units.degreesToRadians(720));
    }

    public RoboRoute(RoboRouteIO io) {
        this.io = io;
        Trigger trigger = new Trigger(() -> inputs.run);

        trigger.onTrue(Commands.deferredProxy(() -> {
            try {
                return processStepCommand(RoboRouteCodec.decode(inputs.plan))
                .beforeStarting(() -> { isRouting = true; })
                .finallyDo(() -> { isRouting = false; });
            } catch (IOException e) {
                // Log and return a no-op command on error
                e.printStackTrace();
                return Commands.none();
            }
        }));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RoboRoute", inputs);
    }

    /**
     * 
     * @param step A RoboRoute step to be processed
     * @return A step as a Wpilib command
     */
    public Command processStepCommand(RoboRouteStep step) {
        switch (step.getStepType()) {
            case PATH:
                return runPath(step.getPathName());

            case POSE:
                return pathfindToPose(step.getPose());

            case ROUTE:
                return runRoute(step.getRoutePoses());

            case ACTION:
                return runCustomAction(step.getActionId());

            case SEQUENCE:
                List<Command> sequence = new ArrayList<>();
                step.getChildren().forEach(child -> sequence.add(processStepCommand(child)));

                return Commands.sequence(sequence.toArray(Command[]::new));

            case PARALLEL:
                List<Command> parallelList = new ArrayList<>();
                step.getChildren().forEach(child -> parallelList.add(processStepCommand(child)));

                return Commands.parallel(parallelList.toArray(Command[]::new));

            default:
                return Commands.none();
        }
    }

    public static Command runPath(String pathName) {
        isRouting = true;
        try {
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(pathName), constraints)
                    .finallyDo(() -> isRouting = false);
        } catch (FileVersionException | IOException | ParseException e) {
            // Log and return a no-op command on error
            e.printStackTrace();
            return Commands.none();
        }
    }

    public static Command pathfindToPose(Pose2d pose) {
        isRouting = true;
        return AutoBuilder.pathfindToPose(pose, constraints).finallyDo(() -> isRouting = false);
    }

    public static Command runRoute(Pose2d[] route) {
        if (route.length < 2) {
            return Commands.none();
        }
        isRouting = true;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(route);

        GoalEndState goalEndState = new GoalEndState(0.0, route[route.length - 1].getRotation());
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, goalEndState);
        path.preventFlipping = true;

        return AutoBuilder.pathfindThenFollowPath(path, constraints).finallyDo(() -> isRouting = false);
    }

    public static Command runCustomAction(int actionId) {
        switch (actionId) {
            default:
                return Commands.none();
        }
    }
}
