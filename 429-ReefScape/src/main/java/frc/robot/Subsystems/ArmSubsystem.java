package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX armMotor;

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

  public void stopArm(){
    armMotor.stopMotor();
  }
}
