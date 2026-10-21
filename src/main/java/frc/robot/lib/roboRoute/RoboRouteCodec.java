package frc.robot.lib.roboRoute;

import java.io.IOException;
import java.util.ArrayList;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.roboRoute.RoboRouteStep.RoboRouteStepType;

public final class RoboRouteCodec {
    private static final ObjectMapper mapper = new ObjectMapper();

    public static RoboRouteStep decode(String json) throws IOException {
        return parseStep(mapper.readTree(json));
    }

    private static RoboRouteStep parseStep(JsonNode node) {
        var type = RoboRouteStepType.valueOf(node.required("type").asText());

        switch (type){
            case PATH:
                return new RoboRouteStep(node.required("pathName").asText());
            case POSE:
                return new RoboRouteStep(parsePose(node.required("pose")));
            case ROUTE:
                var poseNodes = node.required("routePoses");
                var poses = new Pose2d[poseNodes.size()];

                for (int i = 0; i < poses.length; i++) {
                    poses[i] = parsePose(poseNodes.required(i));
                }
                return new RoboRouteStep(poses);
            case ACTION:
                return new RoboRouteStep(node.required("actionId").asInt());
            case SEQUENCE, PARALLEL:
                ArrayList<RoboRouteStep> children = new ArrayList<>();
                for (var child : node.required("children")) {
                    children.add(parseStep(child));
                }

                return new RoboRouteStep(type, children);
            default:
                return new RoboRouteStep(RoboRouteStepType.SEQUENCE, new ArrayList<RoboRouteStep>());
        }
    }

    private static Pose2d parsePose(JsonNode node) {
        return new Pose2d(
            node.required("x").asDouble(),
            node.required("y").asDouble(),
            Rotation2d.fromRadians(
                node.required("rotation").asDouble()
            )
        );
    }

}