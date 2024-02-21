package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    /*We need methods to intake and stop when note is detected, feed to shooter, reverse intake and feed manually.
     * Common wisdom says that the intake should run at 2x drive speed.
     */
    private final CANSparkMax intake;
    private final RelativeEncoder encoder;
    private final SparkPIDController intakePID;
    private final DigitalInput optical;


    public Intake(int intakeID, int sensorDIO) {
        intake = new CANSparkMax(intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        optical = new DigitalInput(sensorDIO);

        intake.setSmartCurrentLimit(20);
        encoder = intake.getEncoder();
        intakePID = intake.getPIDController();

        intakePID.setP(IntakeConstants.kP);
        intakePID.setI(IntakeConstants.kI);
        intakePID.setD(IntakeConstants.kD);
        intakePID.setIZone(IntakeConstants.kIz);
        intakePID.setFF(IntakeConstants.kFF);
        intakePID.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);      
    }

    public void hold(double pos) {
        intake.setSmartCurrentLimit(5);
        intakePID.setReference(pos, ControlType.kPosition);
    }

    public void runAtVelocity(double setpoint) {
        intake.setSmartCurrentLimit(20);
        intakePID.setReference(setpoint, ControlType.kVelocity);
    }

    public void runOpenLoop(double supplier) {
        intake.setSmartCurrentLimit(20);
        intake.set(supplier);
    }

    public void autoIntake() {
        if(!isNote()){
            intakePID.setReference(IntakeConstants.kIntakeSpeed, ControlType.kVelocity);
        }
        else {
            hold(encoder.getPosition());
        }
    }

    public boolean isNote() {
        return optical.get();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getOutputCurrent() {
        return intake.getOutputCurrent();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getTemp() {
        return intake.getMotorTemperature();
    }
}
