package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class DriveMeToTheMoon extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;

  private final PIDController xController = new PIDController(0.5, 0, 0);
  private final PIDController yController = new PIDController(0.5, 0, 0);
  private final PIDController thetaController = new PIDController(0.1, 0, 0);
  private double xGoalMeters = 0.48;
  private double yGoalMeters = Units.inchesToMeters(-7.375);
  private double thetaGoalDegrees = 0;

  private Pose3d lastTagPose = null;

  private static final double DEADBAND = 0.05;

  private DoubleSupplier xSupplier, ySupplier, omegaSupplier, leftTriggerValue, rightTriggerValue;

  // boolean for if bot should align left or right

  public DriveMeToTheMoon(
      Drive drive,
      Vision vision,
      Elevator elevator,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier leftTriggerValue,
      DoubleSupplier rightTriggerValue) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.leftTriggerValue = leftTriggerValue;
    this.rightTriggerValue = rightTriggerValue;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();

    double driveAngle = drive.getRotation().getDegrees();
    this.thetaGoalDegrees = AlignReef.calculateGoalAngle(driveAngle);

    thetaController.enableContinuousInput(-180, 180);
    drive.stop();
  }

  @Override
  public void execute() {
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            Math.copySign(
                xSupplier.getAsDouble() * xSupplier.getAsDouble(), xSupplier.getAsDouble()),
            Math.copySign(
                ySupplier.getAsDouble() * ySupplier.getAsDouble(), ySupplier.getAsDouble()));

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * 0.6, DEADBAND);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            (MathUtil.clamp(linearVelocity.getX(), -1, 1) * drive.getMaxLinearSpeedMetersPerSec()),
            (MathUtil.clamp(linearVelocity.getY(), -1, 1) * drive.getMaxLinearSpeedMetersPerSec()),
            (MathUtil.clamp(omega, -1, 1) * drive.getMaxAngularSpeedRadPerSec()));

    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    // TODO: prevent speed from surpassing maximum
    speeds = speeds.plus(calculateAutoAlignThetaSpeeds());
    speeds = speeds.plus(calculateAutoAlignTranslationSpeeds());
    drive.runVelocity(speeds);

    Logger.recordOutput("X speed", speeds.vxMetersPerSecond);
    Logger.recordOutput("Y Speed", speeds.vyMetersPerSecond);
    Logger.recordOutput("Align/Target Tag ID: ", AlignReef.calculateGoalTargetID(thetaGoalDegrees));
    Logger.recordOutput("Align/Drive Angle: ", drive.getPose().getRotation().getDegrees());
    Logger.recordOutput("Align/Last Tag Pose", lastTagPose);
  }

  /**
   * Calculates and returns a chassis speed with the translation output needed to auto align with a
   * part of the field
   *
   * @return ChassisSpeed Object
   */
  private ChassisSpeeds calculateAutoAlignTranslationSpeeds() {
    if (rightTriggerValue.getAsDouble() > 0.5 || leftTriggerValue.getAsDouble() > 0.5) {
      double xGoalMeters = this.getXSetpoint();
      double yGoalMeters = this.getYSetpoint();
      // System.out.println("X Setpoint: " + xSetpoint + " | Y Setpoint: " + ySetpoint);
      xController.setSetpoint(xGoalMeters);
      yController.setSetpoint(yGoalMeters);

      double xdist = 0;
      double ydist = 0;

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
      double xOutput = 0.0;
      double yOutput = 0.0;
      if (tag == null && lastTagPose != null) {
        Transform3d transform = new Pose3d(drive.getPose()).minus(lastTagPose);
        tag = new Pose3d(transform.getTranslation(), transform.getRotation());
      }
      Logger.recordOutput("testpos", tag);
      if (tag != null) {
        lastTagPose =
            new Pose3d(drive.getPose())
                .plus(new Transform3d(tag.getTranslation(), tag.getRotation()));
        xdist = tag.getX();
        ydist = tag.getY();
        System.out.println("X Distance: " + xdist + " | Y Distance: " + ydist);
        xOutput = -xController.calculate(xdist);
        yOutput = -yController.calculate(ydist);

        ChassisSpeeds translationSpeeds =
            new ChassisSpeeds(
                xOutput * drive.getMaxLinearSpeedMetersPerSec(),
                yOutput * drive.getMaxLinearSpeedMetersPerSec(),
                0);

        drive.setAutoAlignComplete(
            Math.abs(xdist - xGoalMeters) < Units.inchesToMeters(1.25)
                && Math.abs(ydist - yGoalMeters) < Units.inchesToMeters(1.25)
                && Math.abs(drive.getPose().getRotation().getDegrees() - thetaGoalDegrees) < 3);

        return translationSpeeds;
      } else {
        drive.setAutoAlignComplete(false);
        // drive.stop();
      }
    }
    drive.setAutoAlignComplete(false);
    this.lastTagPose = null;
    return new ChassisSpeeds();
  }

  /**
   * Calculates and Returns a chassis speed with rotational speed for auto align. This method is
   * separate from the translation auto align calculation to allow it to work even without an april
   * tag being seen
   *
   * @return
   */
  private ChassisSpeeds calculateAutoAlignThetaSpeeds() {
    if (rightTriggerValue.getAsDouble() > 0.5 || leftTriggerValue.getAsDouble() > 0.5) {
      ChassisSpeeds thetaSpeed = new ChassisSpeeds();
      this.thetaGoalDegrees = AlignReef.calculateGoalAngle(drive.getRotation().getDegrees());

      thetaController.setSetpoint(thetaGoalDegrees);

      if (rightTriggerValue.getAsDouble() > 0.5 || leftTriggerValue.getAsDouble() > 0.5) {
        thetaSpeed =
            thetaSpeed.plus(
                new ChassisSpeeds(
                    0,
                    0,
                    thetaController.calculate(drive.getPose().getRotation().getDegrees()) * 0.5));
      }
      return thetaSpeed;
    }
    return new ChassisSpeeds();
  }

  /**
   * Returns the X setpoint for auto align based on elevator setpoint
   *
   * @return
   */
  private double getXSetpoint() {
    // Algae Setpoint
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) return 0.7;
    else {
      // L1, 2, and 3 setpoint
      if (elevator.getSetpoint() == Constants.ElevatorConstants.kLevel1Meters
          || elevator.getSetpoint() == Constants.ElevatorConstants.kLevel2Meters
          || elevator.getSetpoint() == Constants.ElevatorConstants.kLevel3Meters) return 0.43;
      // L4 Setpoint
      return 0.5;
    }
  }

  /**
   * Returns the Y setpoint for auto align based on elevator setpoints and trigger values
   *
   * @return
   */
  private double getYSetpoint() {
    // Centered for algae pickup
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) return 0;
    else {
      // positive for right side of april tag, negative for left side
      if (rightTriggerValue.getAsDouble() >= leftTriggerValue.getAsDouble())
        return Units.inchesToMeters(7.375);
      return Units.inchesToMeters(-7.375);
    }
  }
}
