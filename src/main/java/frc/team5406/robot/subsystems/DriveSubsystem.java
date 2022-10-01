// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // Create 4 new SparkMAX Objects - Used to control the drive motors.
  private CANSparkMax leftDrive = new CANSparkMax(Constants.MOTOR_DRIVE_LEFT_ONE, MotorType.kBrushless);
  private CANSparkMax leftDriveFollower = new CANSparkMax(Constants.MOTOR_DRIVE_LEFT_TWO,
      MotorType.kBrushless);
  private CANSparkMax rightDrive = new CANSparkMax(Constants.MOTOR_DRIVE_RIGHT_ONE, MotorType.kBrushless);
  private CANSparkMax rightDriveFollower = new CANSparkMax(Constants.MOTOR_DRIVE_RIGHT_TWO,
      MotorType.kBrushless);

  // Create 2 Encoder objects - used to read the Position and Velocity values from
  // the motors.
  private RelativeEncoder leftEncoder, rightEncoder;

  // Create 2 PID Controllers
  private SparkMaxPIDController leftMotorPID, rightMotorPID;

  private DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);
  SimpleMotorFeedforward driveTrainLeft = new SimpleMotorFeedforward(Constants.S_VOLTS_LEFT,
      Constants.V_VOLTS_LEFT,
      Constants.A_VOLTS_LEFT);

  SimpleMotorFeedforward driveTrainRight = new SimpleMotorFeedforward(Constants.S_VOLTS_RIGHT,
      Constants.V_VOLTS_RIGHT,
      Constants.A_VOLTS_RIGHT);

  // Create a new NavX Object - used for Gyro and Odoemtry
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveOdometry odometry;

  /**
   * Setup the motors each time a "DriveSubsystem" object is made.
   */
  public void setupMotors() {
    // Restore each of the SparkMax Controllers to Factory Settings.
    leftDriveFollower.restoreFactoryDefaults();
    rightDriveFollower.restoreFactoryDefaults();
    rightDrive.restoreFactoryDefaults();
    leftDrive.restoreFactoryDefaults();

    // Have the "follower" motors follow the main SparkMAX's
    leftDriveFollower.follow(leftDrive, false);
    rightDriveFollower.follow(rightDrive, false);

    // Invert the left side of the drive train.
    rightDrive.setInverted(false);
    leftDrive.setInverted(true);

    // Set each SparkMAX to a current limit of 40amps.
    leftDrive.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_LEFT);
    leftDriveFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_LEFT);
    rightDrive.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_RIGHT);
    rightDriveFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_DRIVE_RIGHT);

    // Get the PID controllers and encoder profiles from the SparkMAXs
    leftMotorPID = leftDrive.getPIDController();
    rightMotorPID = rightDrive.getPIDController();
    leftEncoder = leftDrive.getEncoder();
    rightEncoder = rightDrive.getEncoder();


    //Set PID values
    leftMotorPID.setP(Constants.LEFT_DRIVE_PID1_P, 1);
    leftMotorPID.setI(Constants.LEFT_DRIVE_PID1_I, 1);
    leftMotorPID.setD(Constants.LEFT_DRIVE_PID1_D, 1);
    leftMotorPID.setIZone(0, 1);
    leftMotorPID.setFF(Constants.LEFT_DRIVE_PID1_F, 1);
    leftMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);

    rightMotorPID.setP(Constants.RIGHT_DRIVE_PID1_P, 1);
    rightMotorPID.setI(Constants.RIGHT_DRIVE_PID1_I, 1);
    rightMotorPID.setD(Constants.RIGHT_DRIVE_PID1_D, 1);
    rightMotorPID.setIZone(0, 1);
    rightMotorPID.setFF(Constants.RIGHT_DRIVE_PID1_F, 1);
    rightMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);

    // Set the time it takes to go from 0 to full power on the drive motors.
    rightDrive.setOpenLoopRampRate(Constants.OPEN_LOOP_RAMP_RATE);
    leftDrive.setOpenLoopRampRate(Constants.OPEN_LOOP_RAMP_RATE);

    // Set the Conversion factors of the encoders from the default units to meters
    // and meters/second
    leftEncoder.setPositionConversionFactor(
        Constants.GEAR_RATIO_DRIVE * Math.PI * Constants.DRIVE_WHEEL_DIAMETER);
    leftEncoder.setVelocityConversionFactor(
        (Constants.GEAR_RATIO_DRIVE * Math.PI * Constants.DRIVE_WHEEL_DIAMETER) / Constants.SECONDS_PER_MINUTE);

    rightEncoder.setPositionConversionFactor(
        Constants.GEAR_RATIO_DRIVE * Math.PI * Constants.DRIVE_WHEEL_DIAMETER);
    rightEncoder.setVelocityConversionFactor(
        (Constants.GEAR_RATIO_DRIVE * Math.PI * Constants.DRIVE_WHEEL_DIAMETER) / Constants.SECONDS_PER_MINUTE);

    resetEncoders();

    // Burn all the changes to the SparkMAX's flash.
    leftDrive.burnFlash();
    leftDriveFollower.burnFlash();
    rightDrive.burnFlash();
    rightDriveFollower.burnFlash();
  }

  /**
   * 
   * @param speed    - Speed you'd wish to go
   * @param rotation - Speed you'd wish to turn
   */
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(-1*speed, rotation, true);
  }

  /**
   * 
   * @param break - Boolean, true break, false coast.
   */
  public void setBrakeMode(boolean brake) {
    IdleMode brakeMode = (brake ? IdleMode.kBrake : IdleMode.kCoast);
    leftDrive.setIdleMode(brakeMode);
    leftDriveFollower.setIdleMode(brakeMode);
    rightDrive.setIdleMode(brakeMode);
    rightDriveFollower.setIdleMode(brakeMode);
  }

  /**
   *
   * @return Left Encoders velocity (m/s)
   */
  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  /**
   *
   * @return Right Encoders velocity (m/s)
   */
  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  /**
   *
   * @return Encoders Average velocity (m/s)
   */
  public double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  /**
   * 
   * @return Left Distance travelled since start
   */
  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  /**
   * 
   * @return Right Distance travelled since start
   */
  public double getRightDistance() {
    return rightEncoder.getPosition();
  }

  // Reset Encoders
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * 
   * @return Rotation turned from the Gyro
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle() * (Constants.GYRO_REVERSED ? -1.0 : 1.0));
  }

  /**
   * Reset the Gyro to Zero.
   */
  public void resetGyro() {
    gyro.zeroYaw();
  }

  /**
   * Uses PID control, as well as a FeedForward Value to determine the voltage to apply
   * to each motor to reach target speed.
   * 
   * @param leftSpeed
   * @param rightSpeed
   */
  public void outputSpeeds(double leftSpeed, double rightSpeed) {
    double origLeftSpeed = leftSpeed;
    double origRightSpeed = rightSpeed;

    double arbFFLeft = driveTrainLeft.calculate(origLeftSpeed);
    double arbFFRight = driveTrainRight.calculate(origRightSpeed);

    leftMotorPID.setReference(leftSpeed, ControlType.kVelocity, 1, arbFFLeft,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    rightMotorPID.setReference(rightSpeed, ControlType.kVelocity, 1, arbFFRight,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    drive.feed();
  }

  /**
   * Directly set voltage of motor controllers.
   * 
   * @param leftVolts
   * @param rightVolts
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    drive.feed();
  }



  /**
   * 
   * @return Position travelled in meters from gyro.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Reset the gyro and encoders on the start of auto.
   */
  public void reset() {
    resetGyro();
    resetEncoders();
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  /**
   * Reset the odometry
   * 
   * @param pose - Position on the field
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }


  /**
   * Setup all the motors each time this class is created.
   */
  public DriveSubsystem() {
    setupMotors();
    resetGyro();
    odometry = new DifferentialDriveOdometry(getHeading());
  }

  /**
   * Update odometry every loop cycle
   */
  @Override
  public void periodic() {
    odometry.update(getHeading(), getLeftDistance(), getRightDistance());
  }

}
