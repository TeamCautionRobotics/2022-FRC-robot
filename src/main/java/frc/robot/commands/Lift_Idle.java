package frc.robot.commands;

import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Lift_Idle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;

  public Lift_Idle(ClimbLift liftSubsystem) {

    this.liftSubsystem = liftSubsystem;
    
    addRequirements(liftSubsystem);

  }

  @Override
  public void initialize() {

    commandDone = false;  // wpilib bug workaround

  }

  @Override
  public void execute() {

    liftSubsystem.enablePID(false);  // disable the lift pid
    liftSubsystem.setPower(0.0);  // zero the power

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
