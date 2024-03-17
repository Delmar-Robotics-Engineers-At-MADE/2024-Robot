package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    /*We need methods to run the arm to postions using a rev throughbore encoder in absolute mode.
     * It is needed to read arm position and to move the arm.
     */
    private final CANSparkMax left;
    private final CANSparkMax right;
    private final AbsoluteEncoder encoder;
    private final SparkPIDController armPID;

    private final ArmFeedforward ff =
        new ArmFeedforward(
        ArmConstants.kSVolts, ArmConstants.kGVolts,
        ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    public Arm (int leftID, int rightID) {
        left = new CANSparkMax(leftID, MotorType.kBrushless);
        right = new CANSparkMax(rightID, MotorType.kBrushless);
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setSmartCurrentLimit(40);
        right.setSmartCurrentLimit(40);
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        left.setInverted(false);
        right.setInverted(true);

        encoder = left.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setInverted(true);
        right.follow(left, true);
        armPID = left.getPIDController();
        armPID.setFeedbackDevice(encoder);

        armPID.setP(ArmConstants.kP);
        armPID.setI(ArmConstants.kI);
        armPID.setD(ArmConstants.kD);
        armPID.setIZone(ArmConstants.kIz);
        armPID.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

        left.burnFlash();
        right.burnFlash();

    }

    public void runOpenLoop(double supplier) {
        if(getPos() >= ArmConstants.kUpperLimit) {
            left.set(supplier);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            left.set(supplier);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else {
            left.set(supplier);
        }
    }

  
    public void hold(TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        armPID.setReference(setpoint.position, ControlType.kPosition,0, feedforward);
    }

    public void runToPosition(TrapezoidProfile.State setpoint) {
        //these are included safety measures. not necessary, but useful
        if(getPos() >= ArmConstants.kUpperLimit) {
            left.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            left.set(0);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT! " + getPos());
        }
        else{
            double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
            armPID.setReference(setpoint.position, ControlType.kPosition, 0, feedforward);
        }
    }

    public double getPos() {
        return encoder.getPosition();
    }

    public double[] getTemp() {
       double result[] = {left.getMotorTemperature(), right.getMotorTemperature()};
       return result;
    }
    
    public double[] getCurrent() {
        double result[] = {left.getOutputCurrent(), right.getOutputCurrent()};
        return result;
    }
}
