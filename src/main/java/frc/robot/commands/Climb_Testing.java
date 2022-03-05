package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;

public class Climb_Testing extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem;

  // angle vars
  private boolean anglePidEnabled = true;
  private double angleSetpoint = 0;
  private double angleManualPower = 0;
  private double angle_kP = Constants.Climb.angler.kP;
  private double angle_kI = Constants.Climb.angler.kI;
  private double angle_kD = Constants.Climb.angler.kD;

  private boolean anglePidEnabled_last = true;
  private double angleSetpoint_last = 0;
  private double angleManualPower_last = 0;
  private double angle_kP_last = Constants.Climb.angler.kP;
  private double angle_kI_last = Constants.Climb.angler.kI;
  private double angle_kD_last = Constants.Climb.angler.kD;

  // lift vars
  private boolean liftPidEnabled = true;
  private double liftSetpoint = 0;
  private double liftManualPower = 0;
  private double lift_kP = Constants.Climb.lift.kP;
  private double lift_kI = Constants.Climb.lift.kI;
  private double lift_kD = Constants.Climb.lift.kI;

  private boolean liftPidEnabled_last = true;
  private double liftSetpoint_last = 0;
  private double liftManualPower_last = 0;
  private double lift_kP_last = Constants.Climb.lift.kP;
  private double lift_kI_last = Constants.Climb.lift.kI;
  private double lift_kD_last = Constants.Climb.lift.kD;

  // hook vars
  private boolean hookEnabled = false;
  private boolean hookEnabled_last = false;


  public Climb_Testing(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // put numbers on the smart dash
    // angle vars
    SmartDashboard.putBoolean("Angle PID Enable", anglePidEnabled);
    SmartDashboard.putNumber("Angle Setpoint", angleSetpoint);
    SmartDashboard.putNumber("Angle P", angle_kP);
    SmartDashboard.putNumber("Angle I", angle_kI);
    SmartDashboard.putNumber("Angle D", angle_kD);
    SmartDashboard.putNumber("Angle Manual Pwr", angleManualPower);

    // lift vars
    SmartDashboard.putBoolean("Lift PID Enable", liftPidEnabled);
    SmartDashboard.putNumber("Lift Setpoint", liftSetpoint);
    SmartDashboard.putNumber("Lift P", lift_kP);
    SmartDashboard.putNumber("Lift I", lift_kI);
    SmartDashboard.putNumber("Lift D", lift_kD);
    SmartDashboard.putNumber("Lift Manual Pwr", liftManualPower);

    // hook vars
    SmartDashboard.putBoolean("Hook Deployed", hookEnabled);

  }

  @Override
  public void execute() {

    // print current angle vars
    SmartDashboard.putNumber("Angle Left Distance", angleSubsystem.getLeftAngleEncoderDistance());
    SmartDashboard.putNumber("Angle Right Distance", angleSubsystem.getRightAngleEncoderDistance());

    SmartDashboard.putBoolean("Angle Left Sw", angleSubsystem.getLeftArmAtZeroSwitch());
    SmartDashboard.putBoolean("Angle Right Sw", angleSubsystem.getRightArmAtZeroSwitch());
  
    // update angle vars
    anglePidEnabled =  SmartDashboard.getBoolean("Angle PID Enable", false);
    angleSetpoint = SmartDashboard.getNumber("Angle Setpoint", 0.0);
    angle_kP = SmartDashboard.getNumber("Angle P", 0.0);
    angle_kI = SmartDashboard.getNumber("Angle I", 0.0);
    angle_kD = SmartDashboard.getNumber("Angle D", 0.0);
    angleManualPower = SmartDashboard.getNumber("Angle Manual Pwr", 0.0);

    if (anglePidEnabled != anglePidEnabled_last) {
      anglePidEnabled_last = anglePidEnabled;
      angleSubsystem.enableAnglePid(anglePidEnabled);
      System.out.println("updated angle PID enable state");
    }

    if (angleSetpoint != angleSetpoint_last) {
      angleSetpoint_last = angleSetpoint;
      angleSubsystem.setAngleReference(angleSetpoint);
      System.out.println("updated angle setpoint");
    }

    if ((angle_kP != angle_kP_last) ||
        (angle_kI != angle_kI_last) ||
        (angle_kD != angle_kD_last)) {
          angle_kP_last = angle_kP;
          angle_kI_last = angle_kI;
          angle_kD_last = angle_kD;
          angleSubsystem.setPidVars(angle_kP, angle_kI, angle_kD);
          System.out.println("updated angle PID vars");
        }

    if (angleManualPower != angleManualPower_last) {
      angleManualPower_last = angleManualPower;
      angleSubsystem.setAnglePower(angleManualPower);
      System.out.println("updated angle manual power");
    }

    // print current lift vars
    SmartDashboard.putNumber("Lift Left Distance", liftSubsystem.getLeftLiftPosition());
    SmartDashboard.putNumber("Lift Right Distance", liftSubsystem.getRightLiftPosition());

    SmartDashboard.putNumber("Lift Left Current Draw", liftSubsystem.getLeftMotorCurrent());
    SmartDashboard.putNumber("Lift Right Current Draw", liftSubsystem.getRightMotorCurrent());

    SmartDashboard.putBoolean("Lift Left Sw", liftSubsystem.getLeftArmFullyDownSwitch());
    SmartDashboard.putBoolean("Lift Right Sw", liftSubsystem.getRightArmFullyDownSwitch());
    
    // update lift vars
    liftPidEnabled = SmartDashboard.getBoolean("Lift PID Enable", true);
    liftSetpoint = SmartDashboard.getNumber("Lift Setpoint", 0.0);
    lift_kP = SmartDashboard.getNumber("Lift P", 0.0);
    lift_kI = SmartDashboard.getNumber("Lift I", 0.0);
    lift_kD = SmartDashboard.getNumber("Lift D", 0.0);
    liftManualPower = SmartDashboard.getNumber("Lift Manual Pwr", 0.0);

    if (liftPidEnabled != liftPidEnabled_last) {
      liftPidEnabled_last = liftPidEnabled;
      liftSubsystem.enablePID(liftPidEnabled);
      System.out.println("updated lift pid enable state");
    }

    if (liftSetpoint != liftSetpoint_last) {
      liftSetpoint_last = liftSetpoint;
      liftSubsystem.setPosition(liftSetpoint);
      System.out.println("updated lift setpoint");
    }

    if (lift_kP != lift_kP_last) {
      lift_kP_last = lift_kP;
      liftSubsystem.setP(lift_kP);
      System.out.println("updated lift kP");
    }

    if (lift_kI != lift_kI_last) {
      lift_kI_last = lift_kI;
      liftSubsystem.setI(lift_kI);
      System.out.println("updated lift kI");
    }

    if (lift_kD != lift_kD_last) {
      lift_kD_last = lift_kD;
      liftSubsystem.setD(lift_kD);
      System.out.println("updated lift kD");
    }

    if (liftManualPower != liftManualPower_last) {
      liftManualPower_last = liftManualPower;
      liftSubsystem.setPower(liftManualPower);
      System.out.println("updated lift manual power");
    }

    // update hook vars
    hookEnabled = SmartDashboard.getBoolean("Hook Deployed", false);

    if (hookEnabled != hookEnabled_last) {
      hookEnabled_last = hookEnabled;
      hookSubsystem.setHook(hookEnabled);
      System.out.println("updated hook state");
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
