package org.jmhsrobotics.frc2025.util;

public class GoalTolerance {
    public boolean atGoalTolerance(double goal, double currentValue, double tolerance){
        return Math.abs(currentValue - goal)
            < tolerance;
    }
}
