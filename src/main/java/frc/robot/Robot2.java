package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.lang.Math;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot2 extends TimedRobot {
  private final TalonFX m_fx = new TalonFX(5);
  private final MotionMagicDutyCycle m_mmReq = new MotionMagicDutyCycle(0);
  private final XboxController m_joystick = new XboxController(0);
  double position = 0.0;

  public Robot2() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
  
   /* Configure Motion Magic */
      MotionMagicConfigs mm = cfg.MotionMagic;
      mm.withMotionMagicCruiseVelocity(5000) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(750) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(0.0);
        m_mmReq.FeedForward = 0.0;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 0.2; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0.0; // No output for integrated error
        slot0.kD = 0.0000001; // A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_fx.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure device. Error: " + status.toString());
        }
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder:", m_fx.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Position:", position);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
      /* Deadband the joystick */
      position = m_joystick.getLeftY()*100;
      if (Math.abs(m_joystick.getLeftY()) < 0.08) position = 0.0;

  
      /*m_fx.setControl(m_mmReq.withPosition(leftY * 10).withSlot(0));
      if (m_joystick.getBButton()) {
        m_fx.setPosition(Rotations.of(1));
      }*/
      m_fx.setControl(m_mmReq.withPosition(position).withSlot(0));
      if (m_joystick.getBButton()) {
        m_fx.setPosition(0.0);
      }
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