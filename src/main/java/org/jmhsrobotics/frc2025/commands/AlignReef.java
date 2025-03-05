package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;

public class AlignReef extends Command {
  private final Drive drive;
  private final Vision vision;

  private final PIDController xController = new PIDController(1, 0, 0);
  private final PIDController yController = new PIDController(1, 0, 0);
  private final PIDController thetaController = new PIDController(0.05, 0, 0);
  private double xGoalMeters = 0.5;
  private double yGoalMeters = 0;
  private double thetaGoalDegrees = 0; // Janky only work for one angle now

  public AlignReef(Drive drive, Vision vision) {
    addRequirements(drive);
    this.drive = drive;
    this.vision = vision;
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(xGoalMeters);
    yController.setSetpoint(yGoalMeters);

    this.thetaGoalDegrees = this.calculateGoalAngle();

    thetaController.setSetpoint(thetaGoalDegrees);
    thetaController.enableContinuousInput(-180, 180);
    drive.stop();
  }

  @Override
  public void execute() {
    Pose3d tag = null; // TODO: handle seeing more than one reef tag
    int targetTag =  this.calculateGoalTargetID();
    for (var target : vision.getTagPoses(0)) { // TODO: Handle more than one camera
      if (target.id()
          == targetTag) { // TODO: janky only work for one tag for now
        tag = target.pose();
      }
    }
    // if(tag == null) { // Janky way to use second camera :todo enable after basic testing
    //   for (var target : vision.getTagPoses(1)) { // TODO: Handle more than one camera
    //     if (target.id()
    //         == targetTag) { // TODO: janky only work for one tag for now
    //       tag = target.pose();
    //     }
    //   }
    // }
    System.out.println(tag);
    if (tag != null) {
      double theta = -Math.toDegrees(Math.atan2(tag.getY(), tag.getX()));
      double xdist = tag.getX();
      double ydist = tag.getY();
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

  private double calculateGoalAngle() {
    double driveAngle = drive.getRotation().getDegrees();
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
  private int calculateGoalTargetID() {
    // if current alliance is blue, use the following april tags
    // default team is blue
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (thetaGoalDegrees == 0) return 18;
      else if (thetaGoalDegrees == 60) return 17;
      else if (thetaGoalDegrees == 120) return 22;
      else if (thetaGoalDegrees == 180) return 21;
      else if (thetaGoalDegrees == -60) return 19;
      else return 20;
    }
    // if current alliance is red, use the following april tags
    if (thetaGoalDegrees == 0) return 7;
    else if (thetaGoalDegrees == 60) return 8;
    else if (thetaGoalDegrees == 120) return 9;
    else if (thetaGoalDegrees == 180) return 10;
    else if (thetaGoalDegrees == -60) return 6;
    else return 11;
  }
}
