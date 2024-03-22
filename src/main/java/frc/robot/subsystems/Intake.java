package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Intake.HoldIntake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase{
    /*We need methods to intake and stop when note is detected, feed to shooter, reverse intake and feed manually.
     * Common wisdom says that the intake should run at 2x drive speed.
     */
    private final CANSparkMax intake;
    private final RelativeEncoder encoder;
    private final DigitalInput opticalOne;
    private final DigitalInput opticalTwo;
    private final PIDController velPID;
    private final PIDController posPID;
    private final SimpleMotorFeedforward ff;

    public Intake(int intakeID, int portSensorDIO, int starboardSensorDIO) {
        intake = new CANSparkMax(intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        opticalTwo = new DigitalInput(portSensorDIO);
        opticalOne = new DigitalInput(starboardSensorDIO);

        intake.setSmartCurrentLimit(25);
        encoder = intake.getEncoder();

        velPID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        velPID.setTolerance(0, IntakeConstants.kVelTolerance);

        posPID = new PIDController(IntakeConstants.kpP, IntakeConstants.kpI, IntakeConstants.kpD);

        ff = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV);

        this.setDefaultCommand(new HoldIntake(this));
    }

    public void hold(double pos) {
        intake.set(posPID.calculate(getPosition(), pos));
    }

    public void halt() {
        intake.set(0);
    }

    public void runAtVelocity(double setpoint) {
        intake.set(velPID.calculate(getVelocity(), setpoint) + ff.calculate(setpoint));
    }

    public void runOpenLoop(double supplier) {
        intake.set(supplier);
    }

    public void autoIntake() {
        if(!isNote()){
            if(!(intake.getOutputCurrent() >= 15)) {
                intake.set((velPID.calculate(getVelocity(), IntakeConstants.kIntakeSpeed) + ff.calculate(IntakeConstants.kIntakeSpeed))/11000);
                System.out.println(intake.getAppliedOutput());
            }
            else {
                intake.set((velPID.calculate(getVelocity(), IntakeConstants.kIntakeCaptureSpeed) + ff.calculate(IntakeConstants.kIntakeCaptureSpeed))/11000);           
            }
        }
        else {
            intake.set(0);
            System.out.println("capture");
        }
    }

    // public void autoIntake() {
    //     if(!isNote()) {
    //         intake.set(IntakeConstants.kReverseSpeed);
    //     }
    //     else {
    //         hold(getPosition());
    //     }
    // }

    public boolean isNote() {
        return opticalTwo.get();
        //return opticalOne.get();
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

    public boolean isSafeTemp() {
        if(getTemp() < ShooterConstants.kThermalLimit) {
            return true;
        }
        else{
            return false;
        }
    }

    public void hiCurrent() {
        intake.setSmartCurrentLimit(25);
    }

    public void loCurrent() {
        intake.setSmartCurrentLimit(5);
    }

    public boolean isLeft() {
        return opticalTwo.get();
    }

    public boolean isRight() {
        return opticalOne.get();
    }
}
