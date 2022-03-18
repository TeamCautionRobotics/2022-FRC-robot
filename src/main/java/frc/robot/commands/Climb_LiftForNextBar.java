package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.nio.file.ClosedFileSystemException;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

public class Climb_LiftForNextBar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;
  private boolean hooksSet = false;
  private int climbStep = 0;
  private Timer currentLimitTimeout = new Timer();
  private Timer timer0 = new Timer();


  /**
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_LiftForNextBar(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // wpilib bug workaround
    commandDone = false;
    hooksSet = false;

    angleSubsystem.setNeutralMode(NeutralMode.Coast);  // coast the arm motors
    liftSubsystem.setIdleMode(IdleMode.kBrake);  // brake the lift motors

    climbStep = 9;  // start on step 9

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // SAFE MODE

        System.out.println("ERROR: Winch exceeded current limit! Activating safe mode...");

        hookSubsystem.set(true);  // extend the hooks

        // brake all motors
        angleSubsystem.setNeutralMode(NeutralMode.Brake);
        liftSubsystem.setIdleMode(IdleMode.kBrake);

        // cut all power to angle
        angleSubsystem.enablePID(false);
        angleSubsystem.setPower(0);
        angleSubsystem.stop();

        // cut all power to lift
        liftSubsystem.enablePID(false);
        liftSubsystem.setPower(0);
        liftSubsystem.stop();

        commandDone = true;  // we're done
        climbStep = 1;  // do nothing for remainder of command
        break;

      case 1:
        ;  // do nothing
        break;

      case 9:  // run winch a little to give us some slack

        if (liftSubsystem.getLeftEncoderDistance() > 5 &&
            liftSubsystem.getRightEncoderDistance() > 5) {

              liftSubsystem.setPower(0);
              climbStep = 10;

        } else {

          liftSubsystem.enablePID(false);
          liftSubsystem.setPower(0.1);

        }
        break;

      case 10:  // run angle to get us off the bar

        // if we're at the setpoint
        if (angleSubsystem.getLeftEncoderDistance() > 60 &&
            angleSubsystem.getRightEncoderDistance() > 60) {

              climbStep = 11;

        } else {  // if we're not there yet

          angleSubsystem.enablePID(true);
          angleSubsystem.setPosition(78);

        }
        break;

      case 11:
        if (liftSubsystem.getLeftEncoderDistance() > 9 &&
        liftSubsystem.getRightEncoderDistance() > 9) {

          liftSubsystem.setPower(0);
          climbStep = 12;

        } else {

          liftSubsystem.enablePID(false);
          liftSubsystem.setPower(0.1);

        }
        break;
        

      case 12:  // winch out while avoiding the bar

        // if we're at the setpoint
        if (liftSubsystem.getLeftEncoderDistance() > 32.8 &&
            liftSubsystem.getRightEncoderDistance() > 32.8) {

              liftSubsystem.setPower(0);

          if (timer0.get() > 0.5) {
            climbStep = 13;
            timer0.stop();
          }

        } else {  // if we're not there yet

          // go 33 inches out
          liftSubsystem.enablePID(false);
          liftSubsystem.setPower(0.2);

          timer0.reset();
          timer0.start();

          // if we're off the bars, angle down
          if (liftSubsystem.getLeftEncoderDistance() > 10 &&
          liftSubsystem.getRightEncoderDistance() > 10) {

            angleSubsystem.enablePID(true);
            angleSubsystem.setPosition(30);

          }
        }

        // TODO: THIS IS NOT TRIGGERING
        // if we're off the bars, angle down
        if (liftSubsystem.getLeftEncoderDistance() > 12.5 &&
        liftSubsystem.getRightEncoderDistance() > 12.5) {

          System.out.println("DO 50");
          angleSubsystem.enablePID(true);
          angleSubsystem.setPosition(50);

        }
        break;

      case 13:

        // if we're past the bar, angle back up
        if (angleSubsystem.getLeftEncoderDistance() > 95 &&
            angleSubsystem.getRightEncoderDistance() > 95) {

            climbStep = 14;

        } else {

          angleSubsystem.enablePID(true);
          angleSubsystem.setPosition(110);

        }
        break;

      case 14:  // finish
        
        commandDone = true;
        climbStep = 1;  // make loop do nothing
        break;
        
    }

    // safe mode trigger
    // if we're exceeding current limit:
    if(liftSubsystem.getLeftMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold || 
      liftSubsystem.getRightMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold) {

        if (currentLimitTimeout.get() > Constants.Climb.lift.maxCurrentTimeout) {  // if the timeout has expired while the current is exceeded
          currentLimitTimeout.stop(); // stop the timeout timer
          climbStep = 0; // go to safe mode!
        }

    } else {  // if the current has not exceeded threshold
      currentLimitTimeout.reset();  // full reset the timer
      currentLimitTimeout.start();
    }

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    //return commandDone;
    return false;  // force never-ending for testing
  }
}
