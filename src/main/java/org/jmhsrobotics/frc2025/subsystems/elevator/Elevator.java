package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.CheckTolerance;
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

  private final double ffEndHeight = 1.3;

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

    if (this.setPointMeters == Constants.ElevatorConstants.kLevel4Meters
        && inputs.heightMeters < this.ffEndHeight)
      elevatorIO.setPositionMeters(this.setPointMeters, 13);
    else elevatorIO.setPositionMeters(this.setPointMeters);

    Logger.recordOutput("Elevator/Current", this.getCurrentAmps());
    Logger.recordOutput("Elevator/Height", inputs.heightMeters);
    Logger.recordOutput("Elevator/Setpoint Value", setPointMeters);
    Logger.recordOutput("Elevator/Left Motor Temp", inputs.leftMotorTemp);
    Logger.recordOutput("Elevator/Right Motor Temp", inputs.rightMotorTemp);

    SmartDashboard.putNumber("Elevator/Raw Height Meters", inputs.heightMeters);
  }

  public void setSetpoint(double setPoint) {
    this.setPointMeters = setPoint;

    // Speeds up elevator travel to L4 by added ff until elevator is above a certain height
    if (this.setPointMeters == Constants.ElevatorConstants.kLevel4Meters
        && inputs.heightMeters < this.ffEndHeight) elevatorIO.setPositionMeters(setPoint, 13);
    else elevatorIO.setPositionMeters(setPoint);
  }

  public void setVoltage(double voltage) {
    elevatorIO.setVoltage(voltage);
  }

  public double getHeight() {
    return inputs.heightMeters;
  }

  public boolean atGoal() {
    return CheckTolerance.atGoalTolerance(
        setPointMeters, inputs.heightMeters, Constants.ElevatorConstants.kHeightTolerance);
  }

  public double getVelocity() {
    return inputs.velocityMPS;
  }

  public double getCurrentAmps() {
    double totalAmps = 0;
    if (inputs.motorAmps == null) {
      return 0;
    }
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

  public double getSetpoint() {
    return setPointMeters;
  }
}
