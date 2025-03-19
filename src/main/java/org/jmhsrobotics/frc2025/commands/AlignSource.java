package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class AlignSource extends Command {
  private Drive drive;
  private Pose2d goalPose;
  private boolean alignCloseToStation;

  private final PIDController xController = new PIDController(0.6, 0, 0);
  private final PIDController yController = new PIDController(0.6, 0, 0);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  public AlignSource(Drive drive, boolean alignCloseToStation) {
    this.drive = drive;
    this.alignCloseToStation = alignCloseToStation;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.goalPose = calculateSetpoints(drive, alignCloseToStation);
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(this.goalPose.getX());
    yController.setSetpoint(this.goalPose.getY());
    thetaController.setSetpoint(this.goalPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    this.goalPose = calculateSetpoints(drive, alignCloseToStation);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            calculateSourceAutoAlignSpeeds(
                this.drive,
                this.goalPose,
                this.xController,
                this.yController,
                this.thetaController),
            this.drive.getRotation()));
    // calculateSourceAutoAlignSpeeds(
    //     this.drive, this.goalPose, this.xController, this.yController, this.thetaController));
    Logger.recordOutput("Align Source/Goal Pose", this.goalPose);
  }

  @Override
  public boolean isFinished() {
    Transform2d distance = drive.getPose().minus(this.goalPose);
    // return distance.getX() < Units.inchesToMeters(0.5)
    //     && distance.getY() < Units.inchesToMeters(1)
    //     && distance.getRotation().getDegrees() < 3;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  /**
   * calculates the coordinate setpoints for source align based drivetrain position and if the auto
   * align should be on the end further or closer to the drivers
   *
   * @param drive
   * @param alignCloseToSource
   * @return double array with index 0 being the x coordinate setpoint and index 1 being the y
   *     coordinate setpoint
   */
  public static Pose2d calculateSetpoints(Drive drive, boolean alignCloseToSource) {
    Pose2d targetPose = new Pose2d();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      if (drive.getPose().getY() > 4) {
        Pose2d targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(2).orElse(new Pose3d()).toPose2d();
        if (alignCloseToSource) {
          targetPose = targetTagPose.plus(new Transform2d(0.4, 0.5, new Rotation2d()));
        } else {
          targetPose = targetTagPose.plus(new Transform2d(0.4, -0.5, new Rotation2d()));
        }
      } else {
        Pose2d targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(1).orElse(new Pose3d()).toPose2d();
      }
    } else {
      if (drive.getPose().getY() > 4) {
        Pose2d targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(13).orElse(new Pose3d()).toPose2d();
      } else {
        Pose2d targetTagPose =
            VisionConstants.aprilTagLayout.getTagPose(12).orElse(new Pose3d()).toPose2d();
      }
    }
    // if (alignCloseToSource)
    //   return targetTagPose.plus(new Transform2d(0.15, 0.61, new Rotation2d()));
    return targetPose;
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
  public static ChassisSpeeds calculateSourceAutoAlignSpeeds(
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

    return outputSpeeds;
  }
}
