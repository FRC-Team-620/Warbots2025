// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2025.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public static final double maxSpeedMetersPerSec = 5.0;

  // change coefficients when we want to implement turbo
  public static final double turboCoefficient = 1;
  public static final double nonTurboCoefficient = 0.9;

  public static final double odometryFrequency = 100.0; // Hz

  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  public static class revConstants {
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    public static final double trackWidth = Units.inchesToMeters(22.5); // make this a variable?
    public static final double wheelBase = Units.inchesToMeters(22.5); // this too?
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    public static final double maxAngularSpeedRadPerSec =
        DriveConstants.maxSpeedMetersPerSec / DriveConstants.revConstants.driveBaseRadius;

    // Zeroed rotation values for each module, see setup instructions

    // Device CAN IDs
    public static final int pigeonCanId = 62; // gyro

    public static final int frontLeftDriveCanId = 10;
    public static final int backLeftDriveCanId = 12;
    public static final int frontRightDriveCanId = 11;
    public static final int backRightDriveCanId = 13;

    public static final int frontLeftTurnCanId = 20;
    public static final int backLeftTurnCanId = 22;
    public static final int frontRightTurnCanId = 21;
    public static final int backRightTurnCanId = 23;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction =
        (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.2;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 0.1;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 4;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
  }

  public static class thriftyConstants {
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    public static final double trackWidth = Units.inchesToMeters(22.5); // make this a variable?
    public static final double wheelBase = Units.inchesToMeters(22.5); // this too?
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final double maxAngularSpeedRadPerSec =
        DriveConstants.maxSpeedMetersPerSec / DriveConstants.revConstants.driveBaseRadius;

    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions

    // Device CAN IDs
    // TBD
    public static final int pigeonCanId = 62; // gyro

    // TBD
    public static final int frontLeftDriveCanId = 10;
    public static final int backLeftDriveCanId = 12;
    public static final int frontRightDriveCanId = 11;
    public static final int backRightDriveCanId = 13;

    public static final int frontLeftTurnCanId = 20;
    public static final int backLeftTurnCanId = 22;
    public static final int frontRightTurnCanId = 21;
    public static final int backRightTurnCanId = 23;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 45;
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double driveMotorReduction = 6.23;

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.025;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.126;
    // public static final double driveKf = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0; // Useless?

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    // 14 for p value was too much
    public static final double turnKp = 6;
    public static final double turnKd = 0.03;
    public static final double turnSimP = 0.2;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
  }
}
