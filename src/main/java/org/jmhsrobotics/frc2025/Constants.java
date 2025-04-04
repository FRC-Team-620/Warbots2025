// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import org.jmhsrobotics.frc2025.subsystems.drive.DriveConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final double ksimTimestep = 0.02;
  public static final double krealTimeStep = ksimTimestep;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final int[] kIgnoredAprilTags = new int[] {14, 15, 4, 5};
  public static final int[] kReefAprilTags = new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  public static final int[] kProcessorAprilTags = new int[] {12, 13, 2, 1};
  public static final double kReefAngleInc = 60.0;
  public static final double kProcessorAngle = 90.0;
  public static final double[] loadingStationAngles = new double[] {-54.0, 54.0}; // Placeholders

  public static class CAN {
    public static final int kElevatorMotorLeftID = 30;
    public static final int kElevatorMotorRightID = 31;

    public static final int kWristMotorID = 40;

    public static final int kIntakeMotorID = 50;
    public static final int kCoralSensorID = 51;
    public static final int kAlgaeSensorID = 52;

    public static final int kIndexerMotorID = 55;
    public static final int kClimberMotorID = 45;

    public static final int kCanAndGyroID = 60;

    public static final int kMitoCANdriaID = 57;

    public static final int kLinearActuatorMotorLeftID = 61;
    public static final int kLinearActuatorMotorRightID = 62;

    public static final Map<Integer, String> kCanDeviceMap =
        Map.ofEntries(
            Map.entry(kElevatorMotorLeftID, "ElevatorMotorLeft"),
            Map.entry(kElevatorMotorRightID, "ElevatorMotorRight"),
            Map.entry(kWristMotorID, "WristMotor"),
            Map.entry(kIntakeMotorID, "IntakeMotor"),
            Map.entry(kCoralSensorID, "CoralSensor"),
            Map.entry(kAlgaeSensorID, "AlgaeSensor"),
            Map.entry(kIndexerMotorID, "IndexerMotor"),
            Map.entry(kClimberMotorID, "ClimberMotor"),
            Map.entry(kCanAndGyroID, "CanAndGyro"),
            Map.entry(kMitoCANdriaID, "MitoCANdria"),
            Map.entry(kLinearActuatorMotorLeftID, "LinearActuatorMotorLeft"),
            Map.entry(kLinearActuatorMotorRightID, "LinearActuatorMotorRight"),
            Map.entry(DriveConstants.thriftyConstants.frontLeftDriveCanId, "FrontLeftDrive"),
            Map.entry(DriveConstants.thriftyConstants.backLeftDriveCanId, "BackLeftDrive"),
            Map.entry(DriveConstants.thriftyConstants.frontRightDriveCanId, "FrontRightDrive"),
            Map.entry(DriveConstants.thriftyConstants.backRightDriveCanId, "BackRightDrive"),
            Map.entry(DriveConstants.thriftyConstants.frontLeftTurnCanId, "FrontLeftTurn"),
            Map.entry(DriveConstants.thriftyConstants.backLeftTurnCanId, "BackLeftTurn"),
            Map.entry(DriveConstants.thriftyConstants.frontRightTurnCanId, "FrontRightTurn"),
            Map.entry(DriveConstants.thriftyConstants.backRightTurnCanId, "BackRightTurn"));
  }

  public static class ElevatorConstants {

    // new elevator max height is 1.80

    // converts motor rotations to elevator height in centimeters
    public static final double conversionFactor = (((1.0 / 3.0) / 12.0) * 100);

    public static final double kLevel1Meters = 0.47;
    public static final double kLevel2Meters = 0.30;
    public static final double kLevel3Meters = 0.77;
    // BARGE AND L4 SETPOINT CANNOT BE THE EXACT SAME
    public static final double kLevel4Meters = 1.56;
    public static final double kBargeMeters = 1.75;
    public static final double kProcesserMeters = .10;
    public static final double kIntermediateAlgaeSetpoint = 0.27;

    public static final double kAlgaeQTipMeters = .125;
    public static final double kCoralIntakeMeters = 0.0;
    public static final double kAlgaeIntakeL2Meters = 0.54;
    public static final double kAlgaeIntakeL3Meters = 1.03;

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0005;
    public static final double kF = 0.0002;
    public static final double kHeightTolerance = .10;
  }

  public static class WristConstants {

    // TODO: When absolute encoder conversion is fixed, remove  / 360, recalibrate PID and change
    // tolerance to 1 degree;
    public static final double kRotationIntakeCoralDegrees = 14;

    public static final double kLevel1Degrees = 180;
    public static final double kLevel2Degrees = 40.5;
    public static final double kLevel3Degrees = 40.5;
    public static final double kLevel4Degrees = 61.5;

    // algae rotation cannot be the exact same as kLevel1Degrees(stupid shit)
    public static final double kRotationAlgaeDegrees = 181;
    public static final double kRotationProcesserDegrees = 190;
    public static final double kRotationBargeDegrees = 70;

    public static final double kP = 0.0225;
    public static final double kI = 0.00;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double kAngleTolerance = 5;

    public static final double kSafeAngleDegrees = 40;
  }

  public static class IntakeConstants {

    public static final double kAlgaeDefaultCommandSpeed = -0.2;
    public static final double kCoralDefaultCommandSpeed = 0.1;

    public static final double kCoralIntakeSpeedDutyCycle = 0.4;
    public static final double kCoralExtakeSpeedDutyCycle = 0.2;

    public static final double kCoralIntakeIndexerSpeedDutyCycle = 0.23;
    public static final double kCoralIntakeIndexerSlowSpeedDutyCycle = 0.2;

    public static final double kAlgaeExtakeSpeedDutyCycle = 0.9;
    public static final double kAlgaeIntakeSpeedDutyCycle = 0.5;

    public static final int kCoralInIntakeDistanceMm = 30;
    public static final int kAlgaeInIntakeDistanceMm = 40;

    public static final double kCoralDebounceTime = 0.03;
    public static final double kAlgaeDebounceTime = 0.1;
  }

  public static final class IndexerConstants {
    public static final double kIndexerSpeedDutyCycle = 0.85;

    public static final double kIndexerSpeedRPM = 8500;
    public static final double kIndexerMaxAcceleration = 30000;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0000862;
  }

  public static final class ClimberConstants {
    // TODO: figure out what the real soft limits should be
    public static final double kSoftLimitTopDegrees = 7;
    public static final double kSoftLimitBottomDegrees = 70;
  }

  public static class LinearActuatorConstants {
    public static final double kExtendTimeout = 12;
    public static final double kRetractTimeout = 15;
  }

  public static class ModeConstants {
    public static final int kAlgae = 1;
    public static final int kSearch = 2;
    public static final int kCoral = 3;
  }

  public static final class LEDConstants {
    public static final int kPWMHeader = 5;
    // led length in pixels
    public static final int kLength = 150;
    // Density of the LED Strip - currently set at 120 LEDs per meter
    public static final Distance kSpacing = Meters.of(1.0 / 60);
    // number of times the flashcommand will change color per second
    public static final double kFlashFrequency = 10;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
