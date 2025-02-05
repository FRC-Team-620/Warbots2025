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

    public static final int kCanAndGyroID = 60;
  }

  public static class ElevatorConstants {

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

    public static final double kP = .01;
    public static final double kI = .00;
    public static final double kD = .00;
    public static final double kHeightTolerance = 0.1;
  }

  public static class WristConstants {

    public static final double kRotationIntakeCoralDegrees = 0;

    public static final double kLevel1Degrees = 10;
    public static final double kLevel2Degrees = 24;
    public static final double kLevel3Degrees = 24;
    public static final double kLevel4Degrees = 75;

    public static final double kRotationAlgaeDegrees = 180;
    public static final double kRotationProcesserDegrees = 170;
    public static final double kRotationBargeDegrees = 70;
    public static final double kRotationQTipDegrees = 175;

    public static final double kP = 0.01;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kAngleTolerance = 3;
  }

  public static class IntakeConstants {
    public static final double kIntakeSpeedDutyCycle = 0.25;
    public static final double kExtakeSpeedDutyCycle = -0.25;
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
