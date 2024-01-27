package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    /* We should use velocity control to ensure consistant performance.
     * An idle mode for default will help with faster acceleration.
     * A low speed is also needed for AMP.
     */
    private final CANSparkMax top;
    private final CANSparkMax bottom;
    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;
    private final SparkPIDController topVelController;
    private final SparkPIDController bottomVelController;

    Shooter(int topRollerID, int bottomRollerID) {
        top = new CANSparkMax(topRollerID, MotorType.kBrushless);
        bottom = new CANSparkMax(bottomRollerID, MotorType.kBrushless);

        top.restoreFactoryDefaults();
        bottom.restoreFactoryDefaults();

        top.setSmartCurrentLimit(40);
        bottom.setSmartCurrentLimit(40);
        top.setIdleMode(IdleMode.kCoast);
        bottom.setIdleMode(IdleMode.kCoast);

        top.enableVoltageCompensation(12.6);
        bottom.enableVoltageCompensation(12.6);

        topEncoder = top.getEncoder();
        bottomEncoder = bottom.getEncoder();

        topVelController = top.getPIDController();
        bottomVelController = bottom.getPIDController();
        topVelController.setFeedbackDevice(topEncoder);
        bottomVelController.setFeedbackDevice(bottomEncoder);

        topVelController.setP(ShooterConstants.kP);
        topVelController.setI(ShooterConstants.kI);
        topVelController.setD(ShooterConstants.kD);
        topVelController.setIZone(ShooterConstants.kIz);
        topVelController.setFF(ShooterConstants.kFF);
        topVelController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

        bottomVelController.setP(ShooterConstants.kP);
        bottomVelController.setI(ShooterConstants.kI);
        bottomVelController.setD(ShooterConstants.kD);
        bottomVelController.setIZone(ShooterConstants.kIz);
        bottomVelController.setFF(ShooterConstants.kFF);
        bottomVelController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    }

    public void runAtSpeed(double target) {
        double setpoint = target * ShooterConstants.kMaxRPM;
        topVelController.setReference(setpoint, ControlType.kVelocity);
        bottomVelController.setReference(setpoint, ControlType.kVelocity);
    }

    public void stop() {
        top.set(0);
        bottom.set(0);
    }

    public double getTopVelocity() {
        return topEncoder.getVelocity();
    }

    public double getBottomVelocity() {
        return bottomEncoder.getVelocity();
    }
}
