package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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


    Arm (int leftID, int rightID) {
        left = new CANSparkMax(leftID, MotorType.kBrushless);
        right = new CANSparkMax(rightID, MotorType.kBrushless);
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setSmartCurrentLimit(40);
        right.setSmartCurrentLimit(40);
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        encoder = left.getAbsoluteEncoder(Type.kDutyCycle);
        armPID = left.getPIDController();
        armPID.setFeedbackDevice(encoder);
        right.follow(left);

        left.burnFlash();
        right.burnFlash();
    }

    public void runOpenLoop(double supplier) {
        if(getPos() >= ArmConstants.kUpperLimit) {
            hold();
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            hold();
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else {
            left.set(supplier);
        }
    }

    public void hold() {
        armPID.setReference(encoder.getPosition(), ControlType.kPosition);
    }

    public void runToPosition(double setpoint) {
        //these are included safety measures. not necessary, but useful
        if(getPos() >= ArmConstants.kUpperLimit) {
            hold();
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            hold();
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else{
            armPID.setReference(setpoint, ControlType.kPosition);
        }
    }

    public double getPos() {
        return encoder.getPosition();
    }
    
}
