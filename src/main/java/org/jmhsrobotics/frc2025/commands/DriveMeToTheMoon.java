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

  private Trigger autoIntakeAlgae;

  private final PIDController xController = new PIDController(0.6, 0, 0.005);
  private final PIDController yController = new PIDController(0.6, 0, 0.005);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);
  private int targetId;
  Transform2d goalTransform = new Transform2d();

  private Pose3d lastTagPose = null;
  private Pose3d tagPose = new Pose3d();
  private final Transform2d algaeLineupTransform = new Transform2d(0.7, 0, new Rotation2d());
  private final Transform2d algaeIntakeTransform = new Transform2d(0.45, 0, new Rotation2d());

  private static final double DEADBAND = 0.05;

  private DoubleSupplier xSupplier, ySupplier, omegaSupplier, leftTriggerValue, rightTriggerValue;

  // boolean for if bot should align left or right

  public DriveMeToTheMoon(
      Drive drive,
      Vision vision,
      Elevator elevator,
      Intake intake,
      Wrist wrist,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier leftTriggerValue,
      DoubleSupplier rightTriggerValue,
      Trigger autoIntakeAlge) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;
    this.wrist = wrist;
    this.autoIntakeAlgae = autoIntakeAlge;

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
    thetaController.enableContinuousInput(-180, 180);

    drive.stop();
  }

  @Override
  public void execute() {
    double xValue, yValue;

    if (drive.getTurboMode()) {
      xValue = xSupplier.getAsDouble() * DriveConstants.turboCoefficient;
      yValue = ySupplier.getAsDouble() * DriveConstants.turboCoefficient;
    } else {
      xValue = xSupplier.getAsDouble() * DriveConstants.nonTurboCoefficient;
      yValue = ySupplier.getAsDouble() * DriveConstants.nonTurboCoefficient;
    }

    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            Math.copySign(xValue * xValue, xValue), Math.copySign(yValue * yValue, yValue));

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

    // if triggers elevator is at bottom and no coral in intake, default for triggers is source auto
    // align
    if (elevator.getSetpoint() == 0
        && !intake.isCoralInIntake()
        && !autoIntakeAlgae.getAsBoolean()) {
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
                AutoAlign.getSourceAlignSpeeds(
                    drive, setpoint, xController, yController, thetaController));
      } else drive.setAutoAlignComplete(false);
    } else {
      // reef auto align
      if (rightTriggerValue.getAsDouble() > 0.5
          || leftTriggerValue.getAsDouble() > 0.5
          || autoIntakeAlgae.getAsBoolean()) {
        // calculate angle goal, target tag ID, goal transform from tag and tag position
        double thetaGoalDegrees = AutoAlign.calculateGoalAngle(drive.getRotation().getDegrees());
        targetId = AutoAlign.calculateGoalTargetID(thetaGoalDegrees);

        // if driver is pressing the dedicated algae align button, transform is automatically set
        // correctly for algae
        if (autoIntakeAlgae.getAsBoolean()) {
          if (Math.abs(tagPose.getX() - goalTransform.getX()) < Units.inchesToMeters(1)
              && Math.abs(tagPose.getY() - goalTransform.getY()) < Units.inchesToMeters(1)
              && Math.abs(drive.getRotation().getDegrees() - thetaGoalDegrees) < 3) {
            // if goal was intaking, then backs out. if goal was lineup and elevator is at right
            // setpoint, then move up to intake
            if (goalTransform == algaeIntakeTransform) goalTransform = algaeLineupTransform;
            else if (goalTransform == algaeLineupTransform
                && (elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
                    || elevator.getSetpoint() == Constants.ElevatorConstants.kAlgaeIntakeL3Meters))
              goalTransform = algaeIntakeTransform;
          } else if (goalTransform != algaeIntakeTransform
              && goalTransform != algaeLineupTransform) {
            // sets goal to lineup if not already at lineup setting
            goalTransform = algaeLineupTransform;
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

          // gets reef align translation speeds and theta speeds separately, then adds them
          // together
          ChassisSpeeds reefAlignSpeeds =
              AutoAlign.getReefAlignSpeeds(tagPose, goalTransform, xController, yController);
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
    // Really weird way of stopping auto algae intake from starting at the intake setpoint in some
    // extraneous cases. sorry this is getting really spaghetti again
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
