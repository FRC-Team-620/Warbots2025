package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignSource;
import org.jmhsrobotics.frc2025.commands.autoAlign.AutoAlign;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.drive.DriveConstants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class DriveMeToTheMoon extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Intake intake;
  private final Wrist wrist;
  private final Indexer indexer;

  private final PIDController reefXController = new PIDController(0.5, 0, 0.01);
  private final PIDController reefYController = new PIDController(0.5, 0, 0.01);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);

  private final PIDController sourceXController = new PIDController(0.9, 0, 0.01);
  private final PIDController sourceYController = new PIDController(0.9, 0, 0.01);

  private Trigger autoIntakeAlgae;
  private int targetId;

  Pose3d tagPose = new Pose3d();
  Transform2d goalTransform = new Transform2d();
  private Transform2d algaeIntakeTransform = new Transform2d(0.29, 0, new Rotation2d());
  private Transform2d algaeLineupTransform = new Transform2d(0.7, 0, new Rotation2d());
  private Transform2d algaeInIntakeTransform = new Transform2d(1.0, 0, new Rotation2d());

  private Pose3d lastTagPose = null;

  private static final double DEADBAND = 0.05;

  private DoubleSupplier xSupplier, ySupplier, omegaSupplier, leftTriggerValue, rightTriggerValue;

  // boolean for if bot should align left or right

  public DriveMeToTheMoon(
      Drive drive,
      Vision vision,
      Elevator elevator,
      Intake intake,
      Wrist wrist,
      Indexer indexer,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier leftTriggerValue,
      DoubleSupplier rightTriggerValue,
      Trigger autoIntakeAlgae) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;
    this.wrist = wrist;
    this.indexer = indexer;

    this.autoIntakeAlgae = autoIntakeAlgae;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.leftTriggerValue = leftTriggerValue;
    this.rightTriggerValue = rightTriggerValue;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    reefXController.reset();
    reefYController.reset();
    sourceXController.reset();
    sourceYController.reset();

    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);

    drive.stop();
  }

  @Override
  public void execute() {
    double xValue, yValue, omega;

    if (drive.getTurboMode()) {
      xValue = xSupplier.getAsDouble() * DriveConstants.turboCoefficient;
      yValue = ySupplier.getAsDouble() * DriveConstants.turboCoefficient;
      omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * 0.8, DEADBAND);

    } else {
      xValue = xSupplier.getAsDouble() * DriveConstants.nonTurboCoefficient;
      yValue = ySupplier.getAsDouble() * DriveConstants.nonTurboCoefficient;
      omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * 0.6, DEADBAND);
    }

    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            Math.copySign(xValue * xValue, xValue), Math.copySign(yValue * yValue, yValue));

    // Apply rotation deadband

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

    // if triggers elevator is at bottom and no coral in intake, default for triggers is source auto
    // align
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kCoralIntakeMeters
        && intake.getMode() == 2
        && !autoIntakeAlgae.getAsBoolean()
        && !indexer.hasCoral()) {
      if (rightTriggerValue.getAsDouble() > 0.5 || leftTriggerValue.getAsDouble() > 0.5) {
        // calculates the field relative setpoint position
        // TODO: Needs to be cleaned up
        Pose2d setpoint;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
          setpoint =
              AlignSource.calculateSetpoints(
                  drive,
                  (rightTriggerValue.getAsDouble() > 0.5 && drive.getPose().getY() > 4)
                      || (leftTriggerValue.getAsDouble() > 0.5 && drive.getPose().getY() < 4));
        } else {
          setpoint =
              AlignSource.calculateSetpoints(
                  drive,
                  (!((rightTriggerValue.getAsDouble() > 0.5 && drive.getPose().getY() > 4)
                      || (leftTriggerValue.getAsDouble() > 0.5 && drive.getPose().getY() < 4))));
        }
        // calculates the source auto align speed and adds it to speeds
        speeds =
            speeds.plus(
                AutoAlign.getDriveToPoseSpeeds(
                    drive, setpoint, sourceXController, sourceYController, thetaController));
      } else drive.setAutoAlignComplete(false);
    } else {
      // reef auto align
      if (rightTriggerValue.getAsDouble() > 0.5
          || leftTriggerValue.getAsDouble() > 0.5
          || autoIntakeAlgae.getAsBoolean()) {
        // calculate angle goal, target tag ID, goal transform from tag and tag position
        double thetaGoalDegrees = AutoAlign.calculateGoalAngle(drive.getRotation().getDegrees());
        targetId = AutoAlign.calculateGoalTargetID(thetaGoalDegrees);

        if (autoIntakeAlgae.getAsBoolean()) {
          if (Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(5.5)
              && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(5.5)
              && Math.abs(drive.getRotation().getDegrees() - thetaGoalDegrees) < 3
              && goalTransform != algaeInIntakeTransform) {
            if (goalTransform == algaeIntakeTransform) goalTransform = algaeInIntakeTransform;
            else if (goalTransform == algaeLineupTransform
                && (elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
                    || elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL3Meters))
              goalTransform = algaeIntakeTransform;
            else if (goalTransform != algaeIntakeTransform
                && goalTransform != algaeLineupTransform
                && goalTransform != algaeInIntakeTransform) goalTransform = algaeLineupTransform;
          }
        } else {
          goalTransform =
              AutoAlign.calculateReefTransform(
                  elevator.getSetpoint(),
                  leftTriggerValue.getAsDouble() > rightTriggerValue.getAsDouble());
        }

        tagPose = AutoAlign.getTagPoseRobotRelative(targetId, vision, lastTagPose, drive.getPose());

        if (tagPose != null) {
          // sets last tag position
          lastTagPose =
              new Pose3d(drive.getPose())
                  .plus(new Transform3d(tagPose.getTranslation(), tagPose.getRotation()));

          // gets reef align translation speeds and theta speeds separately, then adds them together
          ChassisSpeeds reefAlignSpeeds =
              AutoAlign.getReefAlignSpeeds(
                  tagPose, goalTransform, reefXController, reefYController);
          reefAlignSpeeds =
              reefAlignSpeeds.plus(
                  AutoAlign.getAutoAlignThetaSpeeds(
                      thetaController, thetaGoalDegrees, drive.getRotation()));
          speeds = speeds.plus(reefAlignSpeeds);

          // For LED driver feedback
          Logger.recordOutput("Align/X Distance", Math.abs(tagPose.getX() - goalTransform.getX()));
          Logger.recordOutput("Align/Y Distance", Math.abs(tagPose.getY() - goalTransform.getY()));
          Logger.recordOutput(
              "Align/Theta Distance",
              Math.abs(drive.getRotation().getDegrees() - thetaGoalDegrees));

          drive.setAutoAlignComplete(
              Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(1)
                  && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(1)
                  && Math.abs(drive.getRotation().getDegrees() - thetaGoalDegrees) < 3);
        }

      } else {
        lastTagPose = null;
        drive.setAutoAlignComplete(false);
      }
    }
    // really weird way of stoping auto algae intake from starting at the intake setpoint instead of
    // lineup in some extraneous cases. sorry this is getting very spaghetti-like
    if (!autoIntakeAlgae.getAsBoolean()
        && (!(rightTriggerValue.getAsDouble() > 0.5) || !(leftTriggerValue.getAsDouble() > 0.5)))
      goalTransform = algaeLineupTransform;

    drive.runVelocity(speeds);

    Logger.recordOutput("X speed", speeds.vxMetersPerSecond);
    Logger.recordOutput("Y Speed", speeds.vyMetersPerSecond);
    Logger.recordOutput("Align/Last Tag Pose", lastTagPose);

    Pose3d defaultTagPose =
        VisionConstants.aprilTagLayout
            .getTagPose(targetId)
            .orElse(new Pose3d()); // TODO: handle null tag pose
    Logger.recordOutput("Align/targetPos", defaultTagPose.plus(new Transform3d(goalTransform)));

    if (Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2))
            < 0.01
        && Math.abs(speeds.omegaRadiansPerSecond) < 0.01
        && wrist.getSetpoint() == Constants.WristConstants.kRotationIntakeCoralDegrees) {
      drive.stopWithX();
    }
  }
}
