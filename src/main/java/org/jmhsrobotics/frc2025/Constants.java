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

  public static final int ledPWMHeader = 5;
  // led length in pixels
  public static final int ledLength = 10;
  // Density of the LED Strip - currently set at 120 LEDs per meter
  public static final Distance ledSpacing = Meters.of(1 / 120.0);

  public static class CAN {
    public static final int kElevatorMotorLeftID = 30;
    public static final int kElevatorMotorRightID = 31;

    public static final int kWristMotorID = 40;

    public static final int kIntakeMotorID = 50;
    public static final int kCoralSensorID = 51;
    public static final int kAlgaeSensorID = 52;

    public static final int kIndexerMotorID = 55;
    public static final int kClimberMotorID = 65;

    public static final int kCanAndGyroID = 60;

    public static final int kMitoCANdriaID = 57;
  }

  public static class ElevatorConstants {

    public static final double conversionFactor = 1.0 / 60.0;

    public static final double kLevel1Meters = 0.0;
    public static final double kLevel2Meters = 0.35;
    public static final double kLevel3Meters = 0.75;
    public static final double kLevel4Meters = 1.6;

    public static final double kProcesserMeters = .15;
    public static final double kBargeMeters = 1.8;

    public static final double kAlgaeQTipMeters = 0.125;
    public static final double kCoralIntakeMeters = 0;
    public static final double kAlgaeIntakeL2Meters = .55;
    public static final double kAlgaeIntakeL3Meters = 0.95;

    public static final double kP = 1.75;
    public static final double kI = .00;
    public static final double kD = .005;
    public static final double kHeightTolerance = 0.05;
  }

  public static class WristConstants {

    // TODO: When absolute encoder conversion is fixed, remove  / 360, recalibrate PID and change
    // tolerance to 1 degree;
    public static final double kRotationIntakeCoralDegrees = 20;

    public static final double kLevel1Degrees = 30.0;
    public static final double kLevel2Degrees = 44.0;
    public static final double kLevel3Degrees = 44.0;
    public static final double kLevel4Degrees = 95.0;

    public static final double kRotationAlgaeDegrees = 190;
    public static final double kRotationProcesserDegrees = 190;
    public static final double kRotationBargeDegrees = 90;
    public static final double kRotationQTipDegrees = 190;

    public static final double kP = 0.005;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kAngleTolerance = 2;

    public static final double kSafeAngleDegrees = 40;
  }

  public static class IntakeConstants {
    public static final double kIntakeSpeedDutyCycle = 0.25;
    public static final double kExtakeSpeedDutyCycle = -0.25;
  }

  public static final class IndexerConstants {
    public static final double kRotationUpDegrees = 180;
    public static final double kRotationDownDegrees = 0;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static class ModeConstants {
    public static final int kAlgae = 1;
    public static final int kSearch = 2;
    public static final int kCoral = 3;
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
