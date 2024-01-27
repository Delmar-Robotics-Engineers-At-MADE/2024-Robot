package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController pid;
    DigitalInput limitSwitch;

    Climber(int ID, int DIO) {

        motor = new CANSparkMax(ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        limitSwitch = new DigitalInput(DIO);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(12.6);


        pid.setFeedbackDevice(encoder);

        pid.setP(ClimberConstants.kP);
        pid.setI(ClimberConstants.kI);
        pid.setD(ClimberConstants.kD);
        pid.setIZone(ClimberConstants.kIz);
        pid.setFF(ClimberConstants.kFF);
        pid.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    }
    
}
