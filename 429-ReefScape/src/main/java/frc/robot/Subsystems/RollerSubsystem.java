package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class RollerSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;

  /**
   * This subsytem that controls the roller.
   */
  public RollerSubsystem() {

    // Set up the roller motor as a brushed motor
    String Busname = "";
    rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID, Busname);

    TalonFXConfiguration Config = new TalonFXConfiguration();
    Config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    Config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    Config.CurrentLimits.withSupplyCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);

  }

  @Override
  public void periodic() {
  }

  /**
   * This is a method that makes the roller move at your desired speed
   * Positive values make it spin forward and negative values spin it in reverse
   * 
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runRoller(double speed) {
    rollerMotor.set(speed);
  }
  public void stopRoller(){
    rollerMotor.stopMotor();
  }
}
