package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private MechanismLigament2d stage1 = // TODO move to Robot Container
      new MechanismLigament2d("stage1", 3, 90, 10, new Color8Bit(0, 0, 168));
  private MechanismLigament2d carriage =
      new MechanismLigament2d("carriage", 3, 0, 5, new Color8Bit(255, 0, 0));
  Mechanism2d elevatorMech = new Mechanism2d(4, 4);
  private DigitalInput limitSwitch = new DigitalInput(2); // TODO figure out channel

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    var root = elevatorMech.getRoot("base", 1, 0);
    root.append(stage1).append(carriage);
    SmartDashboard.putData("Elevator", elevatorMech);
    setVoltage(0.001);
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    stage1.setLength(inputs.heightMeters / 2);
    carriage.setLength(inputs.heightMeters / 2);
    if (!limitSwitch.get()) {
      setVoltage(0);
      setZero();
    }
    Logger.recordOutput("Elevator Position:", inputs.heightMeters);
    Logger.recordOutput("Elevator Velocity: ", inputs.velocityMPS);
  }

  public boolean atGoal() {
    return Math.abs(inputs.heightMeters - elevatorIO.getSetpoint())
        < Constants.ElevatorConstants.kHeightTolerance;
  }

  public void setSetpoint(double setPoint) {
    elevatorIO.setPositionMeters(setPoint);
  }

  public void setVoltage(double voltage) {
    elevatorIO.setVoltage(voltage);
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public double getVelocity() {
    return inputs.velocityMPS;
  }

  public void setZero() {
    elevatorIO.setZero();
  }
}
