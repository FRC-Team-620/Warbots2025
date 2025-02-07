package org.jmhsrobotics.frc2025.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoClimberIO implements ClimberIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kClimberMotorID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkMaxConfig motorConfig;

  public NeoClimberIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder = motor.getEncoder();
  }

  public void updateInputs(ClimberIOInputs inputs) {
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorRPM = value);

    // TODO: Figure out Conversion gearing conversion factor to change from relative encoder output
    // to angle of climber
    SparkUtil.ifOk(motor, encoder::getPosition, (value) -> inputs.positionDegrees = value);
  }

  public void set(double speedDutyCycle) {
    motor.set(speedDutyCycle);
  }
}
