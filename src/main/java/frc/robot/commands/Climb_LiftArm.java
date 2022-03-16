package frc.robot.commands;

import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_LiftArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;
  private int climbStep = 10;


  /**
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_LiftArm(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    if (!angleSubsystem.getCalibrated() || !liftSubsystem.getCalibrated()) {
      climbStep = 12;
      System.out.println("ERROR: Climb not calibrated!");
    } else {

      // retract the static hooks
      hookSubsystem.set(false);

      // start on step 0
      climbStep = 10;

      commandDone = false;
    }

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 1:  // do nothing
        ;
        break;

      case 10:  // extend the lift 20 in

      commandDone = false;
        if ((liftSubsystem.getLeftEncoderDistance() > 16.8) &&
           (liftSubsystem.getRightEncoderDistance() > 16.8)) {

            climbStep = 11;

        } else {
             liftSubsystem.enablePID(true);  // enable pid
             liftSubsystem.setPosition(17.0);  // set setpoint
        }
        break;

      case 11:  // angle the arms 110 deg

        if ((angleSubsystem.getLeftEncoderDistance() > 100) &&
           (angleSubsystem.getRightEncoderDistance() > 100)) {

            climbStep = 12;

        } else {

          angleSubsystem.enablePID(true);  // enable pid
          angleSubsystem.setPosition(110);  // set setpoint

        }
        break;

      case 12:

        commandDone = true;  // we're done
        climbStep = 1;  // make loop do nothing
        break;

    }

  }

  @Override
  public void end(boolean interrupted) {

    if (interrupted) {
      System.out.println("command terminated");
    }

  }

  // TODO: FIX THIS
  @Override
  public boolean isFinished() {
    // return commandDone;
    return false;  // make command never-ending for testing
  }
}
