package frc.robot.lib.ObjectVision;
import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class ObjectStruct implements StructSerializable {
    public float x;
    public float y;
    public float z;
    public float roll;
    public float pitch;
    public float yaw;

    public static final ObjectStructDef struct = new ObjectStructDef();

    public ObjectStruct(float x, float y, float z, float roll, float pitch, float yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public ObjectStruct() {}

    public static class ObjectStructDef implements Struct<ObjectStruct> {
        @Override public Class<ObjectStruct> getTypeClass() { return ObjectStruct.class; }
        @Override public String getTypeString() { return "struct:Object"; }
        @Override public int getSize()    { return Float.BYTES * 6; }
        @Override public String getSchema() { return "float x;float y;float z;float roll;float pitch;float yaw;"; }
        @Override public String getTypeName() { return "object"; }

        @Override
        public ObjectStruct unpack(ByteBuffer bb) {
            ObjectStruct f = new ObjectStruct();
            f.x = bb.getFloat();
            f.y = bb.getFloat();
            f.z = bb.getFloat();
            f.roll = bb.getFloat();
            f.pitch = bb.getFloat();
            f.yaw = bb.getFloat();
            return f;
        }

        @Override
        public void pack(ByteBuffer bb, ObjectStruct val) {
            bb.putFloat(val.x);
            bb.putFloat(val.y);
            bb.putFloat(val.z);
            bb.putFloat(val.roll);
            bb.putFloat(val.pitch);
            bb.putFloat(val.yaw);
        }
    }
}