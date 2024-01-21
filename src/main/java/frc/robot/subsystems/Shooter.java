package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
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


    }
}
