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

import org.jmhsrobotics.frc2025.util.CRT;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // RobotBase.startRobot(Robot::new);
    int R = 2;
    int G1 = 11;
    int G2 = 4;
    int encoderResolution = 320;

    int e1 = R * G1;
    double e2 = e1 / G2;
    int E1 = e1 % 1;
    double E2 = e2 % 1;

    E1 *= encoderResolution;
    E2 *= encoderResolution;

    CRT test = new CRT(G1, G2, E1, (int) E2);

    System.out.println(test.calcElevetorHeight());
  }
}
