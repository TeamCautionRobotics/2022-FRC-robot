package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EjectBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intakeSubsystem;
  private final Conveyor conveyorSubsystem;

  public EjectBall(Intake intakeSubsystem, Conveyor conveyorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;

    addRequirements(intakeSubsystem, conveyorSubsystem);
  }

  @Override
  public void initialize() {
    conveyorSubsystem.setGate(true);  // close the gate
    intakeSubsystem.setDeploy(false);  // raise intake
  }

  @Override
  public void execute() {
    conveyorSubsystem.runMotor(-1.0);  // run the conveyor
  }

  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.runMotor(0.0);  // stop the conveyor
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
