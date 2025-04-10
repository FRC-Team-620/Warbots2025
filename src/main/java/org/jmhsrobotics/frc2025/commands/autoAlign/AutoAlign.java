package org.jmhsrobotics.frc2025.commands.autoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.drive.DriveConstants;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;

public class AutoAlign {
  /**
   * Determines the target april tag ID based on the goal angle and alliance color. Ignores opposite
   * team's tags
   *
   * @return April tag ID
   */
  public static int calculateGoalTargetID(double thetaGoalDegrees) {
    // red side tags
    // back - 10
    // front 7
    // blue side tags
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
    int invert = 180;
    if (thetaGoalDegrees == 0 + invert) return 7;
    else if (thetaGoalDegrees == -120) return 8;
    else if (thetaGoalDegrees == 0) return 10;
    else if (thetaGoalDegrees == 120) return 6;
    else if (thetaGoalDegrees == -60) return 9;
    else return 11;
  }

  /**
   * Determines the goal angle based on the current angle by choosing the closest one
   *
   * @param driveAngle
   * @return
   */
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
   * Calculates and Returns a chassis speed with rotational speed for auto align. This method is
   * separate from the translation auto align calculation to allow it to work even without an april
   * tag being seen
   *
   * @return
   */
  public static ChassisSpeeds getAutoAlignThetaSpeeds(
      PIDController thetaController, double thetaGoalDegrees, Rotation2d currentRotation) {
    ChassisSpeeds thetaSpeed = new ChassisSpeeds();
    // thetaGoalDegrees = AutoAlign.calculateGoalAngle(currentRotation.getDegrees()); //FIXME:
    // WARNING THIS MIGHT BrEAK AUTO ALIGN

    thetaController.setSetpoint(thetaGoalDegrees);
    var thetaOut = thetaController.calculate(currentRotation.getDegrees());

    thetaSpeed =
        thetaSpeed.plus(
            new ChassisSpeeds(
                0, 0, thetaOut * DriveConstants.thriftyConstants.maxAngularSpeedRadPerSec));
    return thetaSpeed;
  }

  /**
   * calculates chassis speeds output for source auto align based on a setpoint and PID controllers
   * passed in
   *
   * @param drive
   * @param setpoint
   * @param xController
   * @param yController
   * @param thetaController
   * @return ChassisSpeeds object with x, y, and angular speeds
   */
  public static ChassisSpeeds getDriveToPoseSpeeds(
      Drive drive,
      Pose2d setpoint,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {

    double xOutput = xController.calculate(drive.getPose().getX(), setpoint.getX());
    double yOutput = yController.calculate(drive.getPose().getY(), setpoint.getY());
    double thetaOutput =
        thetaController.calculate(
            drive.getPose().getRotation().getDegrees(), setpoint.getRotation().getDegrees());

    ChassisSpeeds outputSpeeds =
        new ChassisSpeeds(
            xOutput * drive.getMaxLinearSpeedMetersPerSec(),
            yOutput * drive.getMaxLinearSpeedMetersPerSec(),
            thetaOutput * drive.getMaxAngularSpeedRadPerSec());

    return ChassisSpeeds.fromFieldRelativeSpeeds(outputSpeeds, drive.getRotation());
  }

  /**
   * Calculates the proper reef transformtion form the april tag based on elevator setpoint and if
   * aligning left or right
   *
   * @param elevatorSetpoint
   * @param isLeft
   * @return Transform2d goal from the tag position
   */
  public static Transform2d calculateReefTransform(double elevatorSetpoint, boolean isLeft) {
    double yTransform = 0;
    double xTransform = 0;

    if (elevatorSetpoint == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevatorSetpoint == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) {
      xTransform = 0.7;
    } else {
      if (isLeft)
        yTransform =
            (elevatorSetpoint == Constants.ElevatorConstants.kLevel1Meters)
                ? Units.inchesToMeters(-19)
                : Units.inchesToMeters(-8.3);
      else
        yTransform =
            (elevatorSetpoint == Constants.ElevatorConstants.kLevel1Meters)
                ? Units.inchesToMeters(18)
                : Units.inchesToMeters(6);
      if (elevatorSetpoint == Constants.ElevatorConstants.kLevel2Meters
          || elevatorSetpoint == Constants.ElevatorConstants.kLevel3Meters) xTransform = 0.47;
      else if (elevatorSetpoint == Constants.ElevatorConstants.kLevel1Meters) xTransform = 0.5;
      else {
        if (isLeft) xTransform = 0.46;
        else xTransform = 0.45;
      }
    }
    return new Transform2d(xTransform, yTransform, new Rotation2d());
  }

  public static Pose3d getTagPoseRobotRelative(
      int targetId, Vision vision, Pose3d lastPose, Pose2d drivePose) {
    Pose3d tagPose = null;
    lastPose = null;
    // If no tag is seen, estimates tag pos using odometry
    if (tagPose == null) {
      Pose3d defaultTagPose =
          VisionConstants.aprilTagLayout.getTagPose(targetId).orElse(new Pose3d());
      Transform3d tagTransform = defaultTagPose.minus(new Pose3d(drivePose));
      // var tagTransform = new Pose3d(drive.getPose()).minus(defaultTagPose);
      tagPose = new Pose3d(tagTransform.getTranslation(), tagTransform.getRotation());
    }
    return tagPose;
  }

  public static ChassisSpeeds getReefAlignSpeeds(
      Pose3d tagPose,
      Transform2d goalTransform,
      PIDController xController,
      PIDController yController) {
    double xOutput = -xController.calculate(tagPose.getX(), goalTransform.getX());
    double yOutput = -yController.calculate(tagPose.getY(), goalTransform.getY());
    return new ChassisSpeeds(
        xOutput * DriveConstants.maxSpeedMetersPerSec,
        yOutput * DriveConstants.maxSpeedMetersPerSec,
        0);
  }

  public static int adjustTagID(int targetTagID) {
    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      switch (targetTagID) {
        case 17:
          return 8;
        case 18:
          return 7;
        case 19:
          return 6;
        case 20:
          return 11;
        case 21:
          return 10;
        case 22:
          return 9;
      }
    }
    return targetTagID;
  }

  public static double calculateGoalAngleFromId(int targetTagId) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (targetTagId == 18) return 0;
      else if (targetTagId == 17) return 60;
      else if (targetTagId == 22) return 120;
      else if (targetTagId == 21) return 180;
      else if (targetTagId == 19) return -60;
      else return -120;
    }

    // if current alliance is red, use the following angles
    if (targetTagId == 7) return 180;
    else if (targetTagId == 8) return -120;
    else if (targetTagId == 10) return 0;
    else if (targetTagId == 6) return 120;
    else if (targetTagId == 9) return -60;
    else return 60;
  }
}
