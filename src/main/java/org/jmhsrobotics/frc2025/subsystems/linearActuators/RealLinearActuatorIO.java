package org.jmhsrobotics.frc2025.subsystems.linearActuators;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class RealLinearActuatorIO implements LinearActuatorIO {
  private SparkMax leftMotor =
      new SparkMax(Constants.CAN.kLinearActuatorMotorLeftID, MotorType.kBrushed);
  private SparkMax rightMotor =
      new SparkMax(Constants.CAN.kLinearActuatorMotorRightID, MotorType.kBrushed);

  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();

  private double speedDutyCycle;

  public RealLinearActuatorIO() {
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(3)
        .inverted(false)
        .voltageCompensation(12);
    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(3)
        .follow(leftMotor.getDeviceId())
        .inverted(false)
        .voltageCompensation(12);

    SparkUtil.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(LinearActuatorIOInputs inputs) {
    SparkUtil.ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.motorAmps += value);
    leftMotor.set(speedDutyCycle);
  }

  public void set(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    leftMotor.set(speedDutyCycle);
  }
}
