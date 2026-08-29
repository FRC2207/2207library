package frc.robot.lib.ObjectVision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.subsystems.swerveDrive.Drive;
import frc.robot.current.subsystems.swerveDrive.DriveConstants;

public class ObjectVision extends SubsystemBase {

    private final Drive swerve;
    private final ObjectVisionIO io;
    private final ObjectVisionIOInputsAutoLogged inputs = new ObjectVisionIOInputsAutoLogged();

    private static final double MIN_BALL_DISTANCE_M = 0.3;
    private static final int MAX_BALLS = 300;

    private static final double FIELD_LENGTH_M = 16.46;
    private static final double FIELD_WIDTH_M = 8.23;
    private static final double FIELD_MARGIN_M = 0.3;

    private static final long VISION_MAX_AGE_MS = 500;
    private long lastVisionUpdateMs = 0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            DriveConstants.maxSpeedMetersPerSec,
            3.0,
            Math.PI * 2,
            Math.toRadians(720));

    private final Pose3d[] ballPosesBuf = new Pose3d[MAX_BALLS];

    public ObjectVision(Drive drive, ObjectVisionIO io) {
        this.swerve = drive;
        this.io = io;

        for (int i = 0; i < MAX_BALLS; i++) {
            ballPosesBuf[i] = new Pose3d();
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectVision", inputs);

        double[] rx = inputs.objectX;
        double[] ry = inputs.objectY;

        if (rx == null || ry == null || rx.length == 0) {
            Logger.recordOutput("ObjectVision/Balls", new Pose3d[0]);
            return;
        }

        lastVisionUpdateMs = System.currentTimeMillis();

        int n = Math.min(Math.min(rx.length, ry.length), MAX_BALLS);

        for (int i = 0; i < n; i++) {
            ballPosesBuf[i] = new Pose3d(
                    rx[i],
                    ry[i],
                    0.102,
                    new Rotation3d());
        }

        Logger.recordOutput(
                "ObjectVision/Balls",
                java.util.Arrays.copyOf(ballPosesBuf, n));
    }

    private List<Translation2d> getValidBalls() {

        // Ignore stale vision
        if (System.currentTimeMillis() - lastVisionUpdateMs > VISION_MAX_AGE_MS) {
            return List.of();
        }

        double[] rx = inputs.objectX;
        double[] ry = inputs.objectY;

        if (rx == null || ry == null) {
            return List.of();
        }

        Translation2d robotPos = swerve.getPose().getTranslation();

        double minDistanceSquared = MIN_BALL_DISTANCE_M * MIN_BALL_DISTANCE_M;

        List<Translation2d> balls = new ArrayList<>();

        int n = Math.min(rx.length, ry.length);

        for (int i = 0; i < n; i++) {

            double x = rx[i];
            double y = ry[i];

            // Ignore balls outside the field
            if (x < FIELD_MARGIN_M
                    || x > FIELD_LENGTH_M - FIELD_MARGIN_M
                    || y < FIELD_MARGIN_M
                    || y > FIELD_WIDTH_M - FIELD_MARGIN_M) {
                continue;
            }

            // Ignore balls too close to the robot
            double dx = x - robotPos.getX();
            double dy = y - robotPos.getY();

            if (dx * dx + dy * dy < minDistanceSquared) {
                continue;
            }

            balls.add(new Translation2d(x, y));
        }

        return balls;
    }

    private Command buildDriveToClosestBallCommand() {

        List<Translation2d> balls = getValidBalls();

        if (balls.isEmpty()) {
            return null;
        }

        Translation2d robotPos = swerve.getPose().getTranslation();

        Translation2d closestBall = balls.stream()
                .min((a, b) -> Double.compare(
                        robotPos.getDistance(a),
                        robotPos.getDistance(b)))
                .orElse(null);

        if (closestBall == null) {
            return null;
        }

        Logger.recordOutput(
                "ObjectVision/ClosestBall",
                new Pose3d(
                        closestBall.getX(),
                        closestBall.getY(),
                        0.0,
                        new Rotation3d()));

        Pose2d target = new Pose2d(
                closestBall,
                closestBall.minus(robotPos).getAngle());

        return AutoBuilder.pathfindToPose(
                target,
                CONSTRAINTS,
                0.0);
    }

    public Command driveToClosestBall() {
        return Commands.deferredProxy(() -> {

            Command command = buildDriveToClosestBallCommand();

            return command != null
                    ? command
                    : Commands.none();
        });
    }

    public Command kindleCommand() {
        return driveToClosestBall();
    }
}