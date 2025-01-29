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

  public static final int canandgyroCanID = 62;

  public static final int ledPWMHeader = 5;
  // led length in pixels
  public static final int ledLength = 10;
  // Density of the LED Strip - currently set at 120 LEDs per meter
  public static final Distance ledSpacing = Meters.of(1 / 120.0);

  public static class ElevatatorConstants {
    public static final int elevatorMotorLeftID = 30;
    public static final int elevatorMotorRightID = 31;

    public static final double elevatorLevel1Height = 0.25;
    public static final double elevatorLevel2Height = 0.50;
    public static final double elevatorLevel3Height = 0.75;
    public static final double elevatorLevel4Height = 1.00;

    public static final double elevatorkP = .01;
    public static final double elevatorkI = .00;
    public static final double elevatorkD = .00;
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
