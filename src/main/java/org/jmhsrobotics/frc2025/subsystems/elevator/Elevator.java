package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
  }

  public boolean atGoal() {
    return false;
  }

  public void setSetpoint(double setPoint) {
    elevatorIO.setPositionMeters(setPoint);
  }
}
