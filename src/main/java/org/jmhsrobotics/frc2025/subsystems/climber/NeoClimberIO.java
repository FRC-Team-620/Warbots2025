package org.jmhsrobotics.frc2025.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoClimberIO implements ClimberIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kClimberMotorID, MotorType.kBrushless);
  private SparkMaxConfig motorConfig = new SparkMaxConfig();

  private AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
  private AbsoluteEncoder encoder = motor.getAbsoluteEncoder();

  private SoftLimitConfig softLimitConfig = new SoftLimitConfig();

  public NeoClimberIO() {
    encoderConfig.positionConversionFactor(360);
    softLimitConfig
        .forwardSoftLimit(Constants.ClimberConstants.kTopRotationLimit)
        .reverseSoftLimit(Constants.ClimberConstants.kBottomRotationLimit);

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
    motorConfig.softLimit.apply(softLimitConfig);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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
    // if climber is within limits, allow movement in either direction
    // if (encoder.getPosition() > Constants.ClimberConstants.kTopRotationLimit
    //     && encoder.getPosition() < Constants.ClimberConstants.kBottomRotationLimit)
    //   motor.set(speedDutyCycle);
    // else {
    //   // If climber is too high, only negative output can go to motors
    //   if (encoder.getPosition() < Constants.ClimberConstants.kTopRotationLimit
    //       && speedDutyCycle < 0) motor.set(speedDutyCycle);
    //   // if climber is too low, only positive output can go to motors
    //   else if (encoder.getPosition() > Constants.ClimberConstants.kBottomRotationLimit
    //       && speedDutyCycle > 0) motor.set(speedDutyCycle);
    // }
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
