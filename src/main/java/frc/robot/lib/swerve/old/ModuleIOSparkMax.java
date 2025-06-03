package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.CleanSparkMaxValue;
import frc.robot.util.SparkMaxPeriodicFrameConfig;

public class ModuleIOSparkMax implements ModuleIO {
    private SparkMax driveSparkMax;
    private SparkMax turnSparkMax;
    private SparkMaxConfig driveConfig = new SparkMaxConfig();
    private SparkMaxConfig turnConfig = new SparkMaxConfig();

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    
    // I've opted to also add our CANCoders in here for the time being. These can be moved if we see a need, although I don't see us mounting anything else on these modules right now
    private CANcoder turnAbsoluteEncoder;

    // SDS MK4i L3 Gearing
    private final double driveAfterEncoderReduction = Constants.SwerveConstants.driveReduction;
    private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOSparkMax(int index) {
        System.out.println("[Init] Creating ModuleIOSparkMax" + Integer.toString(index));

        switch (index) {
            case 0:
                driveSparkMax = new SparkMax(Constants.SwerveConstants.Modules.frontLeftDrive, MotorType.kBrushless);
                turnSparkMax = new SparkMax(Constants.SwerveConstants.Modules.frontLeftTurn, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(Constants.SwerveConstants.Modules.frontLeftEncoder);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.Modules.frontLeftEncoderOffset);
                break;
            case 1:
                driveSparkMax = new SparkMax(Constants.SwerveConstants.Modules.frontRightDrive, MotorType.kBrushless);
                turnSparkMax = new SparkMax(Constants.SwerveConstants.Modules.frontRightTurn, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(Constants.SwerveConstants.Modules.frontRightEncoder);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.Modules.frontRightEncoderOffset);
                break;
            case 2:
                driveSparkMax = new SparkMax(Constants.SwerveConstants.Modules.backLeftDrive, MotorType.kBrushless);
                turnSparkMax = new SparkMax(Constants.SwerveConstants.Modules.backLeftTurn, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(Constants.SwerveConstants.Modules.backLeftEncoder);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.Modules.backLeftEncoderOffset);
                break;
            case 3:
                driveSparkMax = new SparkMax(Constants.SwerveConstants.Modules.backRightDrive, MotorType.kBrushless);
                turnSparkMax = new SparkMax(Constants.SwerveConstants.Modules.backRightTurn, MotorType.kBrushless);
                turnAbsoluteEncoder = new CANcoder(Constants.SwerveConstants.Modules.backRightEncoder);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.Modules.backRightEncoderOffset);
                break;
            default:
                throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }

        // Insert burn logic here...

        driveSparkMax.setCANTimeout(500);
        turnSparkMax.setCANTimeout(500);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        // Spark Max Config
        turnConfig.inverted(isTurnMotorInverted);
        
        driveConfig.smartCurrentLimit(40);
        turnConfig.smartCurrentLimit(30);

        driveConfig.voltageCompensation(12);
        turnConfig.voltageCompensation(12.0);
        // We'll set these values in memory. We aren't burning for now. This can be updated later...
        for (int i = 0; i < 2; i++) {
            SparkMaxPeriodicFrameConfig.configNotLeader(driveSparkMax, driveConfig);
            SparkMaxPeriodicFrameConfig.configNotLeader(turnSparkMax, turnConfig);
            driveConfig.signals.primaryEncoderPositionPeriodMs(10);
      
            // TODO: Look at encoder stuff - why can't they be together?
            driveEncoder.setPosition(0.0);
            driveConfig.absoluteEncoder.averageDepth(2);
            //driveConfig.alternateEncoder.measurementPeriod(10);
      
            turnRelativeEncoder.setPosition(0.0);
            //turnConfig.alternateEncoder.measurementPeriod(10);
            turnConfig.absoluteEncoder.averageDepth(2);
            //turnConfig.absoluteEncoder.inverted(true);

        }

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = 
            CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.drivePositionRad,
                Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction);
        inputs.driveVelocityRadPerSec = 
            CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.driveVelocityRadPerSec,
                Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / driveAfterEncoderReduction);
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
        //inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

        inputs.turnAbsolutePositionRad =
            MathUtil.angleModulus(
                new Rotation2d(
                    turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
                    * 2.0
                    * Math.PI)
                .minus(absoluteEncoderOffset)
                .getRadians());
        inputs.turnPositionRad =
            CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.turnPositionRad, 
                Units.rotationsToRadians(turnRelativeEncoder.getPosition())
                / turnAfterEncoderReduction);
        inputs.turnVelocityRadPerSec = 
            CleanSparkMaxValue.cleanSparkMaxValue(
                inputs.turnVelocityRadPerSec, 
                Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / turnAfterEncoderReduction);
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
        //inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
    }

    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    public void setDriveBrakeMode(boolean enable) {
        driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
