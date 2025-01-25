package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  private ElevatorIOInputs inputs = new ElevatorIOInputsAutoLogged();
  private double setPointMeters;

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    elevatorIO.updateInputs(inputs);
  }

  public boolean atGoal() {
    return false;
  }

  public void setSetpoint(double setPoint) {
    this.setPointMeters = setPoint;
  }
}
