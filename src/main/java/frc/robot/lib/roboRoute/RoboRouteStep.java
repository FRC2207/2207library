package frc.robot.lib.roboRoute;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public final class RoboRouteStep {
    public enum RoboRouteStepType {
        PATH,
        POSE,
        ROUTE,
        ACTION,
        SEQUENCE,
        PARALLEL,
    }

    private RoboRouteStepType stepType;
    private String pathName;
    private Pose2d pose;
    private Pose2d[] routePoses;
    private int actionId;
    private ArrayList<RoboRouteStep> children;

    /**
     * Creates a RoboRoute step for following a predefined PathPlanner path.
     * @param pathName The name of the PathPlanner path
     */
    public RoboRouteStep(String pathName){
        this.stepType = RoboRouteStepType.PATH;
        this.pathName = pathName;
    }

    /**
     * Creates a RoboRoute step for moving towards a target position.
     * @param pose The target postion to navigate to
     */
    public RoboRouteStep(Pose2d pose){
        this.stepType = RoboRouteStepType.POSE;
        this.pose = pose;
    }

    /**
     * Creates a RoboRoute step for following a path made of waypoints.
     * @param routePoses A list of waypoints to be followed
     */
    public RoboRouteStep(Pose2d[] routePoses){
        this.stepType = RoboRouteStepType.ROUTE;
        this.routePoses = routePoses;
    }

    /**
     * Creates a RoboRoute step for running a custom action.
     * @param actionId The id of a custom action
     */
    public RoboRouteStep(int actionId){
        this.stepType = RoboRouteStepType.ACTION;
        this.actionId = actionId;
    }

    /**
     * Creates a RoboRoute step for grouping steps together for a Sequence or Parallel Group.
     * @param type Either {@link frc.robot.lib.roboRoute.RoboRouteStep.RoboRouteStepType#SEQUENCE RoboRouteStepType.SEQUENCE} or {@link frc.robot.lib.roboRoute.RoboRouteStep.RoboRouteStepType#PARALLEL RoboRouteStepType.PARALLEL}
     * @param children A list of sub steps to run either sequentially or in parallel
     */
    public RoboRouteStep(RoboRouteStepType type, ArrayList<RoboRouteStep> children){
        this.stepType = type;
        this.children = children;
    }

    public RoboRouteStepType getStepType(){
        return this.stepType;
    }

    public String getPathName(){
        return this.pathName;
    }

    public Pose2d getPose(){
        return this.pose;
    }

    public Pose2d[] getRoutePoses(){
        return this.routePoses;
    }

    public int getActionId(){
        return this.actionId;
    }

    public List<RoboRouteStep> getChildren(){
        return Collections.unmodifiableList(this.children);
    }

}