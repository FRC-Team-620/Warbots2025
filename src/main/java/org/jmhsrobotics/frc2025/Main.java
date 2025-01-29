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

import java.math.BigInteger;
import java.util.ArrayList;
import org.jmhsrobotics.frc2025.util.CRT;
import org.jmhsrobotics.frc2025.util.oldcrt;

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
    int encoderResolution = 320;
    int R = (int) (1.5 * encoderResolution);
    int G1 = 11;
    int G2 = 4;
    int e1 = R * G1; // e1 & e2 are relative encoder resolution (multiturn value of the encoder)
    int e2 = e1 / G2; // computing gear ratio for encoder 2
    int E1 = e1 % encoderResolution; // E1 and E2 are forced down to absolute encoders by taking the
    System.out.println(E1);
    // relative value mod the resolution
    int E2 = e2 % encoderResolution;
    System.out.println(E2);
    ArrayList<BigInteger> A = new ArrayList<>();
    A.add(new BigInteger(E1 + ""));
    A.add(new BigInteger(E2 + ""));
    ArrayList<BigInteger> Q = new ArrayList<>();
    Q.add(new BigInteger(G1 + ""));
    Q.add(new BigInteger(G2 + ""));
    oldcrt test = new oldcrt(G1, G2, E1, E2);

    // System.out.println(test.calcElevetorHeight());
    System.out.println(CRT.chinese_remainder_theorem(A, Q, 2).toString());
    System.out.println(R);
  }
}
