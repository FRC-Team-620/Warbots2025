package org.jmhsrobotics.frc2025.subsystems.elevator;

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
  private double setPointMeters;

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    var root = elevatorMech.getRoot("base", 1, 0);
    root.append(stage1).append(carriage);
    SmartDashboard.putData("Elevator", elevatorMech);
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    stage1.setLength(inputs.heightMeters / 2);
    carriage.setLength(inputs.heightMeters / 2);

    Logger.recordOutput("Elevator/Current", this.getCurrentAmps());
  }

  public boolean atGoal() {
    return Math.abs(inputs.heightMeters - setPointMeters)
        < Constants.ElevatorConstants.kHeightTolerance;
  }

  public void setSetpoint(double setPoint) {
    this.setPointMeters = setPoint;
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

  public double getCurrentAmps() {
    double totalAmps = 0;
    for (int i = 0; i < inputs.motorAmps.length; i++) {
      totalAmps += inputs.motorAmps[i];
    }
    return totalAmps;
  }

  public void setZero() {
    elevatorIO.setZero();
  }

  public void setBrakeMode(boolean enable) {
    elevatorIO.setBrakeMode(enable);
  }
}
