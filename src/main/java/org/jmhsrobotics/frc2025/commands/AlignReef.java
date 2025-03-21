package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class AlignReef extends Command {
  private final Drive drive;
  private final Vision vision;
  private final LED led;
  private final Elevator elevator;

  private final PIDController xController = new PIDController(0.6, 0, 0);
  private final PIDController yController = new PIDController(0.6, 0, 0);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);
  private double xGoalMeters = 0.48;
  private double yGoalMeters = Units.inchesToMeters(-7.375);
  private double thetaGoalDegrees = 0; // Janky only work for one angle now

  private LEDPattern progressPattern;
  private double initialDistance = 2;
  private double currentDistance = 2;

  private Pose3d lastTagPose = null;

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
    if (alignLeft) yGoalMeters = Units.inchesToMeters(-7.375);
    else yGoalMeters = Units.inchesToMeters(7.375);
    xGoalMeters = 0.48;
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);
    double driveAngle = drive.getRotation().getDegrees();
    this.thetaGoalDegrees = AutoAlign.calculateGoalAngle(driveAngle);

    thetaController.setSetpoint(thetaGoalDegrees);
    thetaController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    // New Reef Auto Align should have:
    // calculate translation relative to tag
    // calculate target tag
    // calculate target angle
    // calculate target pose relative to bot
    // calculate chassis speeds based on pose and goal pose
    // calculate theta speeds


    // change setpoints if elevator setpoints have changed
    // if elevator septoint is for L2 or L3
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kLevel3Meters) {
      xGoalMeters = 0.45;
      if (alignLeft) yGoalMeters = Units.inchesToMeters(-7.375);
      else yGoalMeters = Units.inchesToMeters(7.375);
      // if elevator setpoint is at L4, stay a little further back
    } else if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel4Meters) {
      xGoalMeters = 0.50;
      if (alignLeft) yGoalMeters = Units.inchesToMeters(-7.375);
      else yGoalMeters = Units.inchesToMeters(7.375);
      // if elevator setpoint is at an algae level, stay a little further out and in the center
    } else if (elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) {
      xGoalMeters = 0.7;
      yGoalMeters = 0;
    }
    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);
    int targetId = AutoAlign.calculateGoalTargetID(thetaGoalDegrees);
    Pose3d tag = null; // TODO: handle seeing more than one reef tag
    for (var target : vision.getTagPoses(0)) { // TODO: Handle more than one camera
      // if(target.id() )
      if (target.id() == targetId) { // TODO: janky only work for one tag for now
        tag = target.pose();
      }
    }
    if (tag == null) { // Janky way to use second camera :todo enable after basic testing
      for (var target : vision.getTagPoses(1)) { // TODO: Handle more than one camera
        if (target.id() == targetId) { // TODO: janky only work for one tag for now
          tag = target.pose();
        }
      }
    }
    Logger.recordOutput("Align/Target Tag ID: ", AutoAlign.calculateGoalTargetID(thetaGoalDegrees));
    Logger.recordOutput("Align/Drive Angle: ", drive.getPose().getRotation().getDegrees());
    System.out.println(tag);

    if (tag == null && lastTagPose != null) {
      Transform3d transform = new Pose3d(drive.getPose()).minus(lastTagPose);
      tag = new Pose3d(transform.getTranslation(), transform.getRotation());
    }
    Logger.recordOutput("testpos", tag);

    if (tag == null) {
      Pose3d defaultTagPose =
          VisionConstants.aprilTagLayout.getTagPose(targetId).orElse(new Pose3d());
      var tagTransform = defaultTagPose.minus(new Pose3d(drive.getPose()));
      // var tagTransform = new Pose3d(drive.getPose()).minus(defaultTagPose);
      tag = new Pose3d(tagTransform.getTranslation(), tagTransform.getRotation());
    }

    Logger.recordOutput("AutoAlignReef/Goal Position", tag);
    if (tag != null) {
      lastTagPose =
          new Pose3d(drive.getPose())
              .plus(new Transform3d(tag.getTranslation(), tag.getRotation()));
      theta = -Math.toDegrees(Math.atan2(tag.getY(), tag.getX()));
      xdist = tag.getX();
      ydist = tag.getY();
      currentDistance = Math.sqrt(xdist * xdist + ydist * ydist);
      var x = -xController.calculate(xdist);
      var y = -yController.calculate(ydist);
      var thetaOut =
          thetaController.calculate(
              drive.getPose().getRotation().getDegrees()); // Janky clamping todo remove
      var speed =
          new ChassisSpeeds(
              x * drive.getMaxLinearSpeedMetersPerSec(),
              y * drive.getMaxLinearSpeedMetersPerSec(),
              thetaOut * drive.getMaxAngularSpeedRadPerSec());
      drive.runVelocity(speed);
    } else {
      // drive.stop();
    }
    Logger.recordOutput("Align/Last Tag Pose", lastTagPose);
    // if (tag == null && lastTagPose == null) {
    //   drive.runVelocity(new ChassisSpeeds(0.2, 0, 0));
    // }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(xdist - xGoalMeters) < Units.inchesToMeters(1)
        && Math.abs(ydist - yGoalMeters) < Units.inchesToMeters(1)
        && Math.abs(drive.getPose().getRotation().getDegrees() - thetaGoalDegrees) < 3;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
