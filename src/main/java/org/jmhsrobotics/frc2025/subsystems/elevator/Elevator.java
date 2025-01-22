package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.subsystems.elevator.ElevatorIO.ElevatorInputs;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  private ElevatorInputs inputs = new ElevatorInputsAutoLogged();
  private double setPointMeters;

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    elevatorIO.updateInputs(inputs);
  }
}
