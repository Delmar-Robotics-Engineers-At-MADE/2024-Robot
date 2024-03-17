package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController pid;
    private DigitalInput limitSwitch;
    private boolean homed;

    public Climber(int ID, int DIO, boolean invert) {

        motor = new CANSparkMax(ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        limitSwitch = new DigitalInput(DIO);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(12.6);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(invert);
        pid.setFeedbackDevice(encoder);

        pid.setP(ClimberConstants.kP);
        pid.setI(ClimberConstants.kI);
        pid.setD(ClimberConstants.kD);
        pid.setIZone(ClimberConstants.kIz);
        pid.setFF(ClimberConstants.kFF);
        pid.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

        homed = false;
    }

    public boolean isHomed() {
        return homed;
    }

    public void home() {
        if(limitSwitch.get()) {
            motor.set(ClimberConstants.kHomeSpeed);
            homed = false;
        }
        else {
            motor.set(0);
            encoder.setPosition(0);
            homed = true;
        }
    }

    public void stop() {
        motor.set(0);
    }

    public void runOpenLoop(boolean up) {
        motor.set(up ? ClimberConstants.kHomeSpeed : -ClimberConstants.kHomeSpeed);
        System.out.println("¡DIRECT CTRL! ¡OVEREXTEND POSSIBLE!");
    }

    public void runInDirection(boolean up) {
        if(!isHomed()) {
            motor.set(up ? ClimberConstants.kHomeSpeed : -ClimberConstants.kHomeSpeed);
            System.out.println("¡NOT HOMED! ¡OVEREXTEND POSSIBLE!");
        }
        else if (!limitSwitch.get() && !up) {
            motor.set(0);
        }
        else if (isHomed()) {
            if(getPos() >= ClimberConstants.kUpperLimit) {
                motor.set(0);
                System.out.println("¡CLIMBER TOO HIGH! ¡OVEREXTEND! ¡OVEREXTEND!");
            }
            else {
                pid.setReference(up ? ClimberConstants.kUpperLimit : ClimberConstants.kLowerLimit, ControlType.kPosition);
            }
        }
    }    

    public void runToPos(double setpoint) {
        if(!isHomed()) {
            motor.set(0);
            System.out.println("¡NOT HOMED! ¡HOMING IMPORTANT!");
        }
        else if(setpoint >= ClimberConstants.kLowerLimit && setpoint <= ClimberConstants.kUpperLimit) {
            pid.setReference(setpoint, ControlType.kPosition);
        }
    }

    public void hold(double pos) {
        pid.setReference(pos, ControlType.kPosition);
    }

    public double getPos() {
        return encoder.getPosition();
    }
}
