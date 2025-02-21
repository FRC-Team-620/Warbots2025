package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator.ElevatorHeights;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO wristIO;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double setPointDegrees = Constants.WristConstants.kSafeAngleDegrees;
  public ElevatorHeights elevatorHeights;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(inputs);
    Logger.recordOutput("Wrist/AngleDegrees", inputs.positionDegrees);
    Logger.recordOutput("Wrist/OutputCurrent", inputs.motorAmps);
    Logger.recordOutput("Wrist/GoalAngle", setPointDegrees);

    double rawElevatorHeights =
        SmartDashboard.getNumber(
            "Elevator/Setpoint Value",
            0); // put in a value as the default when Elevator setpoint doesn't work
    if (rawElevatorHeights == ElevatorHeights.BOTTOM.getValue()) {
      this.elevatorHeights = ElevatorHeights.BOTTOM;
    } else if (rawElevatorHeights == ElevatorHeights.TOP.getValue()) {
      this.elevatorHeights = ElevatorHeights.TOP;
    }
    wristIO.setWristLimits(this.elevatorHeights);
  }

  public boolean atGoal() {
    return Math.abs(this.setPointDegrees - inputs.positionDegrees)
        < Constants.WristConstants.kAngleTolerance;
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean checkWristSafe() {
    return this.getPositionDegrees() > 20;
  }

  public void setSetpoint(double setPoint) {
    this.setPointDegrees = setPoint;
    wristIO.setPositionDegrees(setPoint);
  }

  public void setBrakeMode(boolean enable) {
    wristIO.setBrakeMode(enable);
  }

  public double getSetpoint() {
    return this.setPointDegrees;
  }
}
