package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intakeSubsystem;
  private final Conveyor conveyorSubsystem;

  public GrabBall(Intake intakeSubsystem, Conveyor conveyorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;

    addRequirements(intakeSubsystem, conveyorSubsystem);
  }

  @Override
  public void initialize() {
    conveyorSubsystem.setGate(true);  // close the gate
    intakeSubsystem.setDeploy(true);  // drop intake
  }

  @Override
  public void execute() {
    intakeSubsystem.runMotor(1.0);  // run the intake
    conveyorSubsystem.runMotor(1.0);  // run the conveyor
  }

  @Override
  public void end(boolean interrupted) {
    // intakeSubsystem.setDeploy(false);  // lift intake
    intakeSubsystem.runMotor(0.0);  // stop the intake
    conveyorSubsystem.runMotor(0.0);  // stop the conveyor
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
