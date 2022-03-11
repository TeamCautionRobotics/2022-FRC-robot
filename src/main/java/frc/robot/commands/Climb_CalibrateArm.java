package frc.robot.commands;

import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_CalibrateArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;

  private int climbStep = 10;
  private Timer failTimer = new Timer();


  /**
   * creates a new Climb_LiftAndLatch commnad.
   * pulls down and hooks onto the first bar, lifts and latches using the static hooks.
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_CalibrateArm(ClimbAngle angleSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // start on step 0 for calibration
    climbStep = 10;

    commandDone = false;

    failTimer.stop();
    failTimer.reset();
    failTimer.start();

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 1:  // do nothing
        ;
        break;

      case 10:

        commandDone = false;
        if (angleSubsystem.getLeftArmAtZeroSwitch() && angleSubsystem.getRightArmAtZeroSwitch()) {
          angleSubsystem.setPower(0.0);
          angleSubsystem.setEncoderPosition(0.0);
          failTimer.reset();
          failTimer.start();
          climbStep = 11;
        } else {
          angleSubsystem.enablePID(false);
          angleSubsystem.setPower(-0.25);
        }
        break;

      case 11:  // Calibrate the lift encoders

        if (liftSubsystem.getLeftArmFullyDownSwitch() && liftSubsystem.getRightArmFullyDownSwitch()) {
          liftSubsystem.setPower(0.0);  // stop the motors
          liftSubsystem.setEncoderPosition(0.0);  // zero the encoders
          failTimer.reset();
          failTimer.start();
          climbStep = 12;  // go to next step
        } else {
          liftSubsystem.enablePID(false);  // disable the PID
          liftSubsystem.setPower(-0.1);  // pull down
        }  
        break;

      case 12:

        failTimer.stop();
        commandDone = true;  // we're done
        climbStep = 1;  // do nothing for rest of loop
        break;

    }

    if (failTimer.get() > 2) {  // if we're doing a single step for more than 2 seconds, fail out
      System.out.println("CALIBRATION ERROR: Timed out (Check limit switches!)");

      // kill power to stuff
      liftSubsystem.enablePID(false);
      liftSubsystem.setPower(0);
      angleSubsystem.enablePID(false);
      liftSubsystem.setPower(0);
      
      // exit
      climbStep = 12;

    }

  }

  @Override
  public void end(boolean interrupted) {

  }
  
  @Override
  public boolean isFinished() {
    return commandDone;
    // return false;  // force never-ending for testing
  }
}
