package org.jmhsrobotics.frc2025.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoClimberIO implements ClimberIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kClimberMotorID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkMaxConfig motorConfig = new SparkMaxConfig();

  private AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

  public NeoClimberIO() {
    encoderConfig.positionConversionFactor(360);

    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12)
        .inverted(false)
        .signals
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    motorConfig.absoluteEncoder.apply(encoderConfig);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorRPM = value);

    // TODO: Figure out Conversion gearing conversion factor to change from relative encoder output
    // to angle of climber
    SparkUtil.ifOk(motor, encoder::getPosition, (value) -> inputs.positionDegrees = value);
  }

  @Override
  public void set(double speedDutyCycle) {
    motor.set(speedDutyCycle);
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
