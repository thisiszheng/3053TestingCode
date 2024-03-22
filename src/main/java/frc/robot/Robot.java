// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */ 

  /*---Part to ID Assignments---*/
  // DriveTrain Motors
  private VictorSPX RightFront = new VictorSPX(0);
  private VictorSPX RightRear = new VictorSPX(1);
  private VictorSPX LeftFront = new VictorSPX(2);
  private VictorSPX LeftRear = new VictorSPX(3);

  // Shooting Wheel Motors
  private TalonSRX RightShooterWheel = new TalonSRX(4);
  private TalonSRX LeftShooterWheel = new TalonSRX(7);

  // Shooting Arm Motors
  private TalonSRX RightShooterArm = new TalonSRX(5);
  private TalonSRX LeftShooterArm = new TalonSRX(6);

  // PS4 Joystick
  private Joystick driverJoystick = new Joystick(0);

  // Climbing Arm Motors
  public static final int leftMotorID = 15;
  public static final int rightMotorID = 14;
  CANSparkMax ClimberLeft = new CANSparkMax(leftMotorID, MotorType.kBrushless);
  CANSparkMax ClimberRight = new CANSparkMax(rightMotorID, MotorType.kBrushless);

  // Encoders & PID
  RelativeEncoder climberEncoder;
  SparkPIDController SparkPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /*---Calcuations & Control Assignments---*/
  // DriveTrain Varible & Control
  double speed = -driverJoystick.getRawAxis(5) * 0.3;
  double turn = driverJoystick.getRawAxis(0) * 0.3;

  // DriveTrain Calcuation
  double left = speed + turn;
  double right = speed - turn;

  // Climbing Arm Varible & Control
  double Pull = driverJoystick.getRawAxis(3) * 0.3;
  double Push = driverJoystick.getRawAxis(2) * 0.3;

  // Climbing Arm Calculation
  double climbSpeed = Pull - Push; // when ClimbingArm is extending, Pull = 1

  // Shooting Wheel Variable & Control
  // double Lift = driverJoystick.getRawButton()
    
  @Override
  public void robotInit() {
    /*--Encoder & PID Stuff (Still in Progress)--*/
    climberEncoder = ClimberLeft.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
    ClimberLeft.restoreFactoryDefaults(); // resets to 0 everytime drivestation is booted up
    SparkPIDController = ClimberLeft.getPIDController();
    SparkPIDController.setFeedbackDevice(climberEncoder);

    // PID coefficients
    kP = 0;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    SparkPIDController.setP(kP);
    SparkPIDController.setI(kI);
    SparkPIDController.setD(kD);
    SparkPIDController.setIZone(kIz);
    SparkPIDController.setFF(kFF);
    SparkPIDController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /*--Motor Power Settings*/
    // DriveTrain Setting
    RightFront.set(ControlMode.PercentOutput, 0 + right);
    RightRear.set(ControlMode.PercentOutput, 0 + right);
    LeftFront.set(ControlMode.PercentOutput, 0 + left);
    LeftRear.set(ControlMode.PercentOutput, 0 + left);

    // Climbing Arm Setting
    ClimberRight.set(0 + climbSpeed);
    ClimberLeft.set(0 + climbSpeed);

    // Shooting Wheel Setting
    // RightShooterWheel.set(0);
    // LeftShooterWheel.set(0);

    /*--PID stuff (Still in Progress)--*/
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { SparkPIDController.setP(p); kP = p; }
    if((i != kI)) { SparkPIDController.setI(i); kI = i; }
    if((d != kD)) { SparkPIDController.setD(d); kD = d; }
    if((iz != kIz)) { SparkPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { SparkPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      SparkPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    SparkPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
