package org.jmhsrobotics.frc2025.util;

public class CheckTolerance {
  public static boolean atGoalTolerance(double goal, double currentValue, double tolerance) {
    return Math.abs(currentValue - goal) <= tolerance;
  }
}
