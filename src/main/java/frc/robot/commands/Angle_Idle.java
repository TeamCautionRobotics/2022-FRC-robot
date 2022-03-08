package frc.robot.commands;

import frc.robot.subsystems.ClimbAngle;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Angle_Idle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem; 

  private boolean commandDone = false;

  public Angle_Idle(ClimbAngle angleSubsystem) {

    this.angleSubsystem = angleSubsystem;
    
    addRequirements(angleSubsystem);

  }

  @Override
  public void initialize() {

    commandDone = false;  // wpilib bug workaround

  }

  @Override
  public void execute() {

    angleSubsystem.enablePID(false);  // disable angle pid
    angleSubsystem.setPower(0);  // kill power to angle

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
