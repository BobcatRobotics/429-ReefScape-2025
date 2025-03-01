package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX armMotor;
  
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /**
   * This subsytem that controls the arm.
   */
  public ArmSubsystem() {

    // Set up the arm motor as a brushed motor
    String Busname = "";
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, Busname);

    TalonFXConfiguration Config = new TalonFXConfiguration();
    Config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    Config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    Config.CurrentLimits.withSupplyCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 0; // no output for overcome gravity  
    slot0Configs.kP = 0; // An error of 1 rotation results in 0 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0 V output
    Config.Slot0 = slot0Configs;
    armMotor.getConfigurator().apply(Config);
  }

  @Override
  public void periodic() {
  }

  /**
   * This is a method that makes the arm move at your desired speed
   * Positive values make it spin forward and negative values spin it in reverse
   * 
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runArm(double speed) {
    armMotor.set(speed);
  }

  public void runArmPosition(double position) {
    // create a position closed-loop request, voltage output, slot 0 configs
    armMotor.setControl(m_request.withPosition(position));
  }

  public void stopArm(){
    armMotor.stopMotor();
  }
}