package org.jmhsrobotics.frc2025.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoIndexerIO implements IndexerIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kIndexerMotorID, MotorType.kBrushless);
  private SparkMaxConfig motorConfig;
  private RelativeEncoder encoder = motor.getEncoder();
  private double speedDutyCycle;

  public NeoIndexerIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(25) // may need to change current limit
        .voltageCompensation(12) // may need to change voltage compensation
        .inverted(true); // may need to change inverted as well

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorRPM = value);
    SparkUtil.ifOk(
        motor, motor::getMotorTemperature, (value) -> inputs.motorTemperatureCelcius = value);
    inputs.outputSpeedDutyCycle = this.speedDutyCycle;
  }

  @Override
  public void set(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    motor.set(this.speedDutyCycle);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
