package frc.robot.lib.ObjectVision;
import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.AutoLog;
import java.util.function.Consumer;

public interface ObjectVisionIO {
  @AutoLog
  public static class ObjectVisionIOInputs {
    public double[] objectX = new double[]{};
    public double[] objectY = new double[]{};
    public double[] objectZ = new double[]{};
    public double[] objectPitch = new double[]{};
    public double[] objectYaw = new double[]{};
    public double[] objectRoll = new double[]{};
  }

  public default void updateInputs(ObjectVisionIOInputs inputs) {}
}