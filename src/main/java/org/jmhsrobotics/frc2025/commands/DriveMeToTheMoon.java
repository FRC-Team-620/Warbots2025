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
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.drive.DriveConstants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class DriveMeToTheMoon extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Intake intake;

  private final PIDController xController = new PIDController(0.6, 0, 0);
  private final PIDController yController = new PIDController(0.6, 0, 0);
  private final PIDController thetaController = new PIDController(0.01, 0, 0);
  private double thetaGoalDegrees = 0;
  Transform2d goalTransform = new Transform2d();

  private Pose3d lastTagPose = null;

  private static final double DEADBAND = 0.05;

  private DoubleSupplier xSupplier, ySupplier, omegaSupplier, leftTriggerValue, rightTriggerValue;

  // boolean for if bot should align left or right

  public DriveMeToTheMoon(
      Drive drive,
      Vision vision,
      Elevator elevator,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier leftTriggerValue,
      DoubleSupplier rightTriggerValue) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;

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

    // TODO: prevent speed from surpassing maximum
    if (elevator.getSetpoint() == 0 && !intake.isCoralInIntake()) {
      System.out.println("Auto Aligning Source");
      if (rightTriggerValue.getAsDouble() > 0.5) {
        Pose2d sourceGoalPose = AlignSource.calculateSetpoints(drive, true);
        speeds =
            speeds.plus(
                AlignSource.calculateSourceAutoAlignSpeeds(
                    this.drive,
                    sourceGoalPose,
                    this.xController,
                    this.yController,
                    this.thetaController));
      }
      if (leftTriggerValue.getAsDouble() > 0.5) {
        Pose2d sourceGoalPose = AlignSource.calculateSetpoints(drive, false);
        speeds =
            speeds.plus(
                AlignSource.calculateSourceAutoAlignSpeeds(
                    this.drive,
                    sourceGoalPose,
                    this.xController,
                    this.yController,
                    this.thetaController));
      }
    } else {
      System.out.println("Auto Aligning Reef");
      speeds = speeds.plus(calculateAutoAlignReefTranslationSpeeds());
      speeds = speeds.plus(calculateAutoAlignThetaSpeeds());
    }

    drive.runVelocity(speeds);
    int targetId =
        AlignReef.calculateGoalTargetID(
            AlignReef.calculateGoalAngle(drive.getRotation().getDegrees()));

    Logger.recordOutput("X speed", speeds.vxMetersPerSecond);
    Logger.recordOutput("Y Speed", speeds.vyMetersPerSecond);
    Logger.recordOutput("Align/Target Tag ID: ", targetId);
    Logger.recordOutput("Align/Drive Angle: ", drive.getPose().getRotation().getDegrees());
    Logger.recordOutput("Align/Last Tag Pose", lastTagPose);

    Pose3d defaultTagPose =
        VisionConstants.aprilTagLayout
            .getTagPose(targetId)
            .orElse(new Pose3d()); // TODO: handle null tag pose
    boolean isRight = rightTriggerValue.getAsDouble() >= leftTriggerValue.getAsDouble();

    goalTransform = getReefOffset(elevator.getSetpoint(), isRight);
    xController.setSetpoint(goalTransform.getX());
    yController.setSetpoint(goalTransform.getY());

    Logger.recordOutput("Align/targetPos", defaultTagPose.plus(new Transform3d(goalTransform)));
  }

  /**
   * Calculates and returns a chassis speed with the translation output needed to auto align with a
   * part of the field
   *
   * @return ChassisSpeed Object
   */
  private ChassisSpeeds calculateAutoAlignReefTranslationSpeeds() {
    boolean isRight = rightTriggerValue.getAsDouble() >= leftTriggerValue.getAsDouble();
    Transform2d goalTransform =
        getReefOffset(elevator.getSetpoint(), isRight); // TODO: add offset for algae
    xController.setSetpoint(goalTransform.getX());
    yController.setSetpoint(goalTransform.getY());
    int targetId = AlignReef.calculateGoalTargetID(thetaGoalDegrees);
    // Logger.recordOutput("Align/Goal X", goalTransform.getX());
    // Does all calculations only if triggers are pressed
    if (rightTriggerValue.getAsDouble() > 0.5 || leftTriggerValue.getAsDouble() > 0.5) {
      // calculate and update PID loop setpoints relative to tag based on robot state

      Pose3d tag = null;
      // Looks through each cameras inputs and gets the tag position if it matches the target ID
      for (var target : vision.getTagPoses(0)) {
        // if(target.id() )
        if (target.id() == targetId) {
          tag = target.pose();
        }
      }

      if (tag == null) {
        for (var target : vision.getTagPoses(1)) {
          if (target.id() == targetId) {
            tag = target.pose();
          }
        }
      }

      double xOutput, yOutput, xdist, ydist = 0.0;
      // calculates the estimated tag position if a tag is not seen, but it was seen previously
      // allowing for smooth control
      if (tag == null && lastTagPose != null) {
        Transform3d transform = new Pose3d(drive.getPose()).minus(lastTagPose);
        tag = new Pose3d(transform.getTranslation(), transform.getRotation());
      }
      // Logger.recordOutput("testpos", tag);
      // If Tag is still Null Use Global ODOM to navigate to Tag
      if (tag == null) {
        Pose3d defaultTagPose =
            VisionConstants.aprilTagLayout.getTagPose(targetId).orElse(new Pose3d());
        var tagtransform = defaultTagPose.minus(new Pose3d(drive.getPose()));
        tag = new Pose3d(tagtransform.getTranslation(), tagtransform.getRotation());
      }

      // if there is a tag position, calculates the PID outputs
      if (tag != null) {
        lastTagPose =
            new Pose3d(drive.getPose())
                .plus(new Transform3d(tag.getTranslation(), tag.getRotation()));
        xdist = tag.getX();
        ydist = tag.getY();
        xOutput = -xController.calculate(xdist);
        yOutput = -yController.calculate(ydist);

        ChassisSpeeds translationSpeeds = new ChassisSpeeds();
        // Only applied translational auto align speeds if the tag is in front of the robot
        if (tag.getX() > 0.35) {
          translationSpeeds =
              new ChassisSpeeds(
                  xOutput * drive.getMaxLinearSpeedMetersPerSec(),
                  yOutput * drive.getMaxLinearSpeedMetersPerSec(),
                  0);
        } else {
          translationSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        // updates the status for auto align being complete in the drive subsystem - needed for LED
        // feedback
        drive.setAutoAlignComplete(
            Math.abs(xdist - goalTransform.getX()) < Units.inchesToMeters(1.25)
                && Math.abs(ydist - goalTransform.getY()) < Units.inchesToMeters(1.25)
                && Math.abs(drive.getPose().getRotation().getDegrees() - thetaGoalDegrees) < 3);

        return translationSpeeds;
      } else {
        // drive.stop();
      }
    }
    // sets last tag position to null, auto align completion to false, and returns an empty chassis
    // speeds object if auto align is not being attempted
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
  private double getXSetpoint(double elevatorSetpointMeters) {
    // Algae Setpoint
    if (elevatorSetpointMeters == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevatorSetpointMeters == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) return 0.7;
    else {
      // L1, 2, and 3 setpoint
      if (elevatorSetpointMeters == Constants.ElevatorConstants.kLevel1Meters
          || elevatorSetpointMeters == Constants.ElevatorConstants.kLevel2Meters
          || elevatorSetpointMeters == Constants.ElevatorConstants.kLevel3Meters) return 0.43;
      // L4 Setpoint
      return 0.5;
    }
  }

  // private static final Transform2d algaeOffset = new Transform2d(0.7,0,Rotation2d.kZero);
  private static final double coralOffsetY = 7.375;

  /**
   * Returns the Y setpoint for auto align based on elevator setpoints and trigger values
   *
   * @return
   */
  private double getYSetpoint(double elevatorSetpointMeters, boolean isRight) {
    // Centered for algae pickup
    if (elevatorSetpointMeters == Constants.ElevatorConstants.kAlgaeIntakeL2Meters
        || elevatorSetpointMeters == Constants.ElevatorConstants.kAlgaeIntakeL3Meters) return 0;
    else {
      // positive for right side of april tag, negative for left side
      if (isRight) return Units.inchesToMeters(coralOffsetY);
      return Units.inchesToMeters(-coralOffsetY);
    }
  }

  private Transform2d getReefOffset(double elevatorSetpointMeters, boolean isRight) {
    // if(el)
    return new Transform2d(
        this.getXSetpoint(elevatorSetpointMeters),
        this.getYSetpoint(elevatorSetpointMeters, isRight),
        Rotation2d.kZero);
  }
}
