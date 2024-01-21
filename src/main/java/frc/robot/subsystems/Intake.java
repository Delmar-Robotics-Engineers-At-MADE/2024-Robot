package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    /*We need methods to intake and stop when note is detected, feed to shooter, reverse intake and feed manually.
     * Common wisdom says that the intake should run at 2x drive speed.
     */
    private final CANSparkMax intake;
    private final RelativeEncoder encoder;
    private final SparkPIDController intakePID;


    Intake(int intakeID) {
        intake = new CANSparkMax(intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        intake.setSmartCurrentLimit(20);
        encoder = intake.getEncoder();
        intakePID = intake.getPIDController();
    }

    public void hold() {
        intake.setSmartCurrentLimit(5);
        intakePID.setReference(encoder.getPosition(), ControlType.kPosition);
    }

    public void runAtVelocity(double setpoint) {
        intake.setSmartCurrentLimit(20);
        intakePID.setReference(setpoint, ControlType.kVelocity);
    }

    public void runOpenLoop(double supplier, boolean isOverride) {
        if(!isOverride) {
            hold();
        }
        else {
            intake.setSmartCurrentLimit(20);
            intake.set(supplier);
        }
    }

    
}
