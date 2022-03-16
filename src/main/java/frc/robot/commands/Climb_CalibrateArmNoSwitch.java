package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_CalibrateArmNoSwitch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;

  private int climbStep = 10;
  private Timer calibrationFailTimer = new Timer();
  private Timer sectionPassTimer = new Timer();


  /**
   * creates a new Climb_CalibrateArmNoSwitch commnad.
   * Calibrates the climb arms without using limit switches
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_CalibrateArmNoSwitch(ClimbAngle angleSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    climbStep = 10;  // start at step 10
    commandDone = false;  // wpilib bug workaround

    // init timers
    sectionPassTimer.reset();
    sectionPassTimer.start();
    calibrationFailTimer.reset();
    calibrationFailTimer.start();

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 1:  // do nothing
        break;

      case 10:  // calibrate angle motors

        angleSubsystem.enablePID(false);  // disable the pid
        angleSubsystem.setPower(-0.25);  // run that back

        if (angleSubsystem.getLeftEncoderRate() > -0.1 && angleSubsystem.getRightEncoderRate() > -0.1) {  // if we've stopped
          if (sectionPassTimer.get() > Constants.Climb.misc.calibrationSuccessDelay) {  // if we've waited long enough

            angleSubsystem.setPower(0.0);  // stop the motors
            angleSubsystem.setEncoderPosition(0.0);  // zero the encoders

            calibrationFailTimer.reset();  // reset the failiure timer
            calibrationFailTimer.start();  // start the failiure timer
            sectionPassTimer.reset();  // reset the section pass timer
            sectionPassTimer.start();  // start the section pass timer
            climbStep = 11;  // move on

          }
        } else {  // if we haven't stopped

          sectionPassTimer.reset();  // reset the section pass timer
          sectionPassTimer.start();  // start the section pass timer

        }
        break;

      case 11:  // Calibrate lift motors

        liftSubsystem.enablePID(false);  // disable the pid
        liftSubsystem.setPower(-0.1);  // run that back

        if (liftSubsystem.getLeftEncoderRate() > -0.1 && liftSubsystem.getRightEncoderRate() > -0.1) {  // if we've stopped
          if (sectionPassTimer.get() > Constants.Climb.misc.calibrationSuccessDelay) {  // if we've waited long enough

            liftSubsystem.setPower(0.0);  // stop the motors
            liftSubsystem.setEncoderPosition(0.0);  // zero the encoders
            
            calibrationFailTimer.reset();  // reset the failiure timer
            calibrationFailTimer.start();  // start the failiure timer
            sectionPassTimer.reset();  // reset the section pass timer
            sectionPassTimer.start();  // start the section pass timer
            climbStep = 12;  // go to next step

          }
        } else {  // if we haven't stopped

          sectionPassTimer.reset();  // reset the section pass timer
          sectionPassTimer.start();  // start the section pass timer

        }
        break;

      case 12:  // exit with success

        angleSubsystem.setCalibrated(true);
        liftSubsystem.setCalibrated(true);

        sectionPassTimer.stop();
        calibrationFailTimer.stop();
        commandDone = true;  // we're done
        climbStep = 1;  // do nothing for rest of loop
        break;

      case 13:  // exit with fail and kill power

        angleSubsystem.setCalibrated(false);
        liftSubsystem.setCalibrated(false);

        angleSubsystem.enablePID(false);
        angleSubsystem.setPower(0);
        liftSubsystem.enablePID(false);
        liftSubsystem.setPower(0);

        sectionPassTimer.stop();
        calibrationFailTimer.stop();
        commandDone = true;
        climbStep = 1;
        break;

    }

    if (calibrationFailTimer.get() > Constants.Climb.misc.calibrationOverallFailDelay) {  // if we're doing a single step for too long, fail out
      
      System.out.println("ERROR: Calibration timed out!");
      
      climbStep = 13;  // exit with failure

    }

  }

  @Override
  public void end(boolean interrupted) {

    if (angleSubsystem.getCalibrated() && liftSubsystem.getCalibrated()) {
      System.out.println("WARNING: Calibration Success!");
    }

  }
  
  @Override
  public boolean isFinished() {
    return commandDone;
    // return false;  // force never-ending for testing
  }
}
