package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;

public class AlignReef extends Command {
  private final Drive drive;
  private final Vision vision;
  private final LED led;
  private final Elevator elevator;

  private final PIDController xController = new PIDController(0.5, 0, 0);
  private final PIDController yController = new PIDController(0.5, 0, 0);
  private final PIDController thetaController = new PIDController(0.05, 0, 0);
  private double xGoalMeters = 0.48;
  private double yGoalMeters = Units.inchesToMeters(-7);
  private double thetaGoalDegrees = 0; // Janky only work for one angle now

  private LEDPattern progressPattern;
  private double initialDistance = 2;
  private double currentDistance = 2;

  private double theta = 0;
  private double xdist = 0;
  private double ydist = 0;

  // boolean for if bot should align left or right
  private boolean alignLeft = true;

  public AlignReef(Drive drive, Vision vision, LED led, Elevator elevator, boolean alignLeft) {
    this.drive = drive;
    this.vision = vision;
    this.led = led;
    this.elevator = elevator;

    this.alignLeft = alignLeft;

    progressPattern =
        LEDPattern.progressMaskLayer(() -> ((initialDistance - currentDistance) / initialDistance));

    addRequirements(drive, led);
  }

  @Override
  public void initialize() {
    // Default setpoint: L4 left side(i think)
    if (alignLeft) yGoalMeters = Units.inchesToMeters(-7);
    else yGoalMeters = Units.inchesToMeters(7);
    xGoalMeters = 0.48;
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);
    double driveAngle = drive.getRotation().getDegrees();
    this.thetaGoalDegrees = AlignReef.calculateGoalAngle(driveAngle);

    thetaController.setSetpoint(thetaGoalDegrees);
    thetaController.enableContinuousInput(-180, 180);
    drive.stop();
    this.led.setPattern(progressPattern);
  }

  @Override
  public void execute() {
    // change setpoints if elevator setpoints have changed
    // if elevator septoint is for L2 or L3
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kLevel3Meters) {
      xGoalMeters = 0.45;
      if (alignLeft) yGoalMeters = Units.inchesToMeters(-7);
      else yGoalMeters = Units.inchesToMeters(7);
      // if elevator setpoint is at L4, stay a little further back
    } else if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel4Meters) {
      xGoalMeters = 0.48;
      if (alignLeft) yGoalMeters = Units.inchesToMeters(-7);
      else yGoalMeters = Units.inchesToMeters(7);
      // if elevator setpoint is at an algae level, stay a little further out and in the center
    } else if (elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) {
      xGoalMeters = 0.65;
      yGoalMeters = 0;
    }
    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);
    Pose3d tag = null; // TODO: handle seeing more than one reef tag
    for (var target : vision.getTagPoses(0)) { // TODO: Handle more than one camera
      // if(target.id() )
      if (target.id()
          == AlignReef.calculateGoalTargetID(
              thetaGoalDegrees)) { // TODO: janky only work for one tag for now
        tag = target.pose();
      }
    }

    if (tag == null) { // Janky way to use second camera :todo enable after basic testing
      for (var target : vision.getTagPoses(1)) { // TODO: Handle more than one camera
        if (target.id()
            == AlignReef.calculateGoalTargetID(
                thetaGoalDegrees)) { // TODO: janky only work for one tag for now
          tag = target.pose();
        }
      }
    }

    System.out.println(tag);
    if (tag != null) {
      theta = -Math.toDegrees(Math.atan2(tag.getY(), tag.getX()));
      xdist = tag.getX();
      ydist = tag.getY();
      currentDistance = Math.sqrt(xdist * xdist + ydist * ydist);
      var x = -xController.calculate(xdist);
      var y = -yController.calculate(ydist);
      var thetaOut =
          thetaController.calculate(drive.getPose().getRotation().getDegrees())
              * 0.1; // Janky clamping todo remove
      var speed =
          new ChassisSpeeds(
              x * drive.getMaxLinearSpeedMetersPerSec(),
              y * drive.getMaxLinearSpeedMetersPerSec(),
              thetaOut * drive.getMaxAngularSpeedRadPerSec());
      drive.runVelocity(speed);
    } else {
      drive.stop();
    }
  }

  public static double calculateGoalAngle(double driveAngle) {
    // double driveAngle = drive.getRotation().getDegrees();
    if (Math.abs(driveAngle) <= 30) return 0;
    else if (Math.abs(driveAngle) >= 150) return 180;
    else if (driveAngle >= 30 && driveAngle <= 90) return 60;
    else if (driveAngle >= 60) return 120;
    else if (driveAngle <= -30 && driveAngle >= -90) return -60;
    else return -120;
  }

  /**
   * Determines the target april tag ID based on the goal angle and alliance color. Ignores opposite
   * team's tags
   *
   * @return April tag ID
   */
  public static int calculateGoalTargetID(double angle_deg) {
    // if current alliance is blue, use the following april tags
    // default team is blue
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (angle_deg == 0) return 18;
      else if (angle_deg == 60) return 17;
      else if (angle_deg == 120) return 22;
      else if (angle_deg == 180) return 21;
      else if (angle_deg == -60) return 19;
      else return 20;
    }
    // if current alliance is red, use the following april tags
    if (angle_deg == 0) return 7;
    else if (angle_deg == 60) return 8;
    else if (angle_deg == 120) return 9;
    else if (angle_deg == 180) return 10;
    else if (angle_deg == -60) return 6;
    else return 11;
  }

  @Override
  public boolean isFinished() {
    System.out.println(
        "X OFFSET: "
            + Math.abs(xdist - xGoalMeters)
            + " | Y OFFSET: "
            + Math.abs(ydist - yGoalMeters)
            + " | THETA OFFSET: "
            + Math.abs(drive.getPose().getRotation().getDegrees() - thetaGoalDegrees));
    return Math.abs(xdist - xGoalMeters) < Units.inchesToMeters(1.5)
        && Math.abs(ydist - yGoalMeters) < Units.inchesToMeters(1.5)
        && Math.abs(drive.getPose().getRotation().getDegrees() - thetaGoalDegrees) < 3;
  }
}
