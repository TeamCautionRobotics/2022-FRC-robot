package frc.robot.commands;

import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_FirstBar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;
  private int climbStep = 0;
  private Timer timer0;


  /**
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_FirstBar(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // retract the static hooks
    hookSubsystem.set(false);

    // start on step 0
    climbStep = 0;

    commandDone = false;  // wpilib bug workaround

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // pull lift down

      commandDone = false;
        if ((liftSubsystem.getLeftLiftPosition() < 0.2) &&
           (liftSubsystem.getRightLiftPosition() < 0.2)) {

            climbStep = 1;

        } else {

            angleSubsystem.enablePID(false);  // disable the angle pid 
            angleSubsystem.setPower(0);  // zero the power to the angle

            liftSubsystem.enablePID(true);  // enable lift pid
            liftSubsystem.setPosition(0.0);  // set setpoint
        }
        break;

      case 1:  // latch

        hookSubsystem.set(true);  // extend hooks
        timer0.reset();  // reset timer
        timer0.start();  // start timer
        climbStep = 2;  // move to next step
        break;

      case 2:  // kill power to winch
        
        if (timer0.get() > 0.5) {  // wait half a second
          timer0.stop();  // stop the timer
          liftSubsystem.enablePID(false);  // disable the winch pid
          liftSubsystem.setPower(0);  // zero the power to winch motors
          climbStep = 3;  // move to next step
        }
        break;

      case 3:

        commandDone = true;  // we're done
        break;

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
