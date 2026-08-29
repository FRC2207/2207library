package frc.robot.lib.ObjectVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;

public class ObjectVisionIODetection implements ObjectVisionIO {

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("VisionData");   
    private StructArraySubscriber<ObjectStruct> objectSub;

    public ObjectVisionIODetection() {

        // objectSub = table.getStructArrayTopic("object_data", objectStruct.struct).subscribe(new objectStruct[]);

        objectSub = table.getStructArrayTopic("vision_data", ObjectStruct.struct).subscribe(new ObjectStruct[0]);
    }
    
    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        ObjectStruct[] objects = objectSub.get();
        double[] objectXPoints = new double[objects.length];
        double[] objectYPoints = new double[objects.length];
        double[] objectZPoints = new double[objects.length];
        double[] objectPitchPoints = new double[objects.length];
        double[] objectRollPoints = new double[objects.length];
        double[] objectYawPoints = new double[objects.length];
        for (int i = 0; i < objects.length; i++) {
            objectXPoints[i] = objects[i].x;
            objectYPoints[i] = objects[i].y;
            objectZPoints[i] = objects[i].z;
            objectRollPoints[i] = objects[i].roll;
            objectPitchPoints[i] = objects[i].pitch;
            objectYawPoints[i] = objects[i].yaw;
        }
        
        inputs.objectX = objectXPoints;
        inputs.objectY = objectYPoints;
        inputs.objectZ = objectZPoints;
        inputs.objectPitch = objectPitchPoints;
        inputs.objectRoll = objectRollPoints;
        inputs.objectYaw = objectYawPoints;
        
    }
}