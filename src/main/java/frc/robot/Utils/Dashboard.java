package frc.robot.Utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Dashboard extends SubsystemBase{
    // intend to use elastic
    /* only use accessor methods here except for diagnostics controls
        Diagnostics controls go in subsystem tabs NOT in match tab.
        The match tab should provide a clean end user experiance that delivers only match critical information.
    */

    public Dashboard(
     DriveSubsystem drivetrain,
     Arm arm,
     Intake intake, 
     Shooter shooter, 
     Climber port, 
     Climber starboard,
     Sendable autochooser) {

        Shuffleboard.getTab("drivetrain");
        Shuffleboard.getTab("arm");
        Shuffleboard.getTab("intake");
        Shuffleboard.getTab("shooter");
        Shuffleboard.getTab("climbers");

        Shuffleboard.getTab("match").addBoolean("capture", () -> intake.isNote());
        Shuffleboard.getTab("match").addDouble("heading", () -> -drivetrain.getHeading()).withWidget("Gyro");
        Shuffleboard.getTab("match").addBoolean("port homed", () -> port.isHomed());
        Shuffleboard.getTab("match").addBoolean("starboard homed", () -> starboard.isHomed());
        Shuffleboard.getTab("match").addBoolean("overheat", () -> !intake.isSafeTemp() || !shooter.isSafeTemp());


        Shuffleboard.getTab("arm").addDouble("arm pos", () -> arm.getPos());
        Shuffleboard.getTab("arm").addDouble("left temp", () -> arm.getTemp()[0]);
        Shuffleboard.getTab("arm").addDouble("right temp", () -> arm.getTemp()[1]);
        Shuffleboard.getTab("arm").addDouble("left current", () -> arm.getCurrent()[0]);
        Shuffleboard.getTab("arm").addDouble("right current", () -> arm.getCurrent()[1]);
        
        Shuffleboard.getTab("intake").addBoolean("capture", () -> intake.isNote());
        Shuffleboard.getTab("intake").addDouble("current", () -> intake.getOutputCurrent());
        Shuffleboard.getTab("intake").addDouble("velocity", () -> intake.getVelocity());
        Shuffleboard.getTab("intake").addDouble("temp", () -> intake.getTemp());
        Shuffleboard.getTab("intake").addBoolean("thermal safe", () -> intake.isSafeTemp());
        Shuffleboard.getTab("intake").addBoolean("left", ()-> intake.isLeft());
        Shuffleboard.getTab("intake").addBoolean("right", ()-> intake.isRight());
        
        Shuffleboard.getTab("shooter").addDouble("top speed", () -> shooter.getTopVelocity());
        Shuffleboard.getTab("shooter").addDouble("bottom Speed", () -> shooter.getBottomVelocity());
        Shuffleboard.getTab("shooter").addDouble("avg speed", () -> shooter.getAvgVelocity());
        Shuffleboard.getTab("shooter").addDouble("bottom current", () -> shooter.getOutputCurrent()[1]);
        Shuffleboard.getTab("shooter").addDouble("top current", () -> shooter.getOutputCurrent()[0]);
        Shuffleboard.getTab("shooter").addDouble("top temp", () -> shooter.getTemp()[0]);
        Shuffleboard.getTab("shooter").addDouble("bottom temp", () -> shooter.getTemp()[1]);
        Shuffleboard.getTab("shooter").addBoolean("thermal safe", () -> shooter.isSafeTemp());



        Shuffleboard.getTab("climbers").addBoolean("port homed", () -> port.isHomed());
        Shuffleboard.getTab("climbers").addBoolean("starboard homed", () -> starboard.isHomed());
        Shuffleboard.getTab("climbers").addDouble("port pos", () -> port.getPos());
        Shuffleboard.getTab("climbers").addDouble("starboard pos", () -> starboard.getPos());
    }
}
