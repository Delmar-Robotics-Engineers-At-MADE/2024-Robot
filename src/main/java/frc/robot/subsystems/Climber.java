package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

    Climber(int ID, int DIO) {

        motor = new CANSparkMax(ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        limitSwitch = new DigitalInput(DIO);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(12.6);
        motor.setIdleMode(IdleMode.kBrake);

        pid.setFeedbackDevice(encoder);

        pid.setP(ClimberConstants.kP);
        pid.setI(ClimberConstants.kI);
        pid.setD(ClimberConstants.kD);
        pid.setIZone(ClimberConstants.kIz);
        pid.setFF(ClimberConstants.kFF);
        pid.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

        homed = false;
    }

    @Override
    public void periodic() {
        super.periodic();
        isHomed();
    }
    public boolean isHomed() {
        return homed;
    }

    public void home() {
        if(!limitSwitch.get()) {
            motor.set(ClimberConstants.kHomeSpeed);
            homed = false;
        }
        else {
            motor.set(0);
            encoder.setPosition(0);
            homed = true;
        }
    }

    public void runOpenLoop(double speed) {
        if(!isHomed()) {
            motor.set(speed);
            System.out.println("¡NOT HOMED! ¡OVEREXTEND POSSIBLE!");
        }
        else if (isHomed() && speed < 0) {
            motor.set(0);
        }
        else if (isHomed()) {
            if(getPos() >= ClimberConstants.kUpperLimit) {
                motor.set(0);
                System.out.println("¡CLIMBER TOO HIGH! ¡OVEREXTEND! ¡OVEREXTEND!");
            }
            else {
                motor.set(speed);
            }
        }
    }

    public double getPos() {
        return encoder.getPosition();
    }
    
}
