package org.jmhsrobotics.frc2025.util;

import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;
import org.jmhsrobotics.frc2025.Constants;

public class GoalTolerance {
    public boolean atElevatorGoal(Elevator elevator){
        return Math.abs(elevator.getHeight() - elevator.getSetpoint())
            < Constants.ElevatorConstants.kHeightTolerance;
    }
    public boolean atWristGoal(Wrist wrist){
        return Math.abs(wrist.getPositionDegrees() - wrist.getSetpoint())
            < Constants.WristConstants.kAngleTolerance;
    }
}
