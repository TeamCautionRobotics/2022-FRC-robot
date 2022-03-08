package frc.robot.commands;
import frc.robot.subsystems.ClimbHook;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Hook_Idle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbHook hookSubsystem; 

  private Timer timer;
  private boolean commandDone = false;

  public Hook_Idle(ClimbHook hookSubsystem) {

    this.hookSubsystem = hookSubsystem;
    
    addRequirements(hookSubsystem);

  }

  @Override
  public void initialize() {

    commandDone = false;  // wpilib bug workaround
    
    // reset and start timer
    timer.reset(); 
    timer.start();

  }

  @Override
  public void execute() {

    if (timer.get() > 1.0) {  // every second

      hookSubsystem.set(true);  // extend hooks

      // full reset timer
      timer.stop();
      timer.reset();
      timer.start();
    }

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
