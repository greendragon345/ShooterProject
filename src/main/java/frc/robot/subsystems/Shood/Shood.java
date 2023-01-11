package frc.robot.subsystems.Shood;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootThroughDashBoard;

public class Shood extends SubsystemBase {

    private final TalonFX hoodM;
    private final TalonFX shooterM;
    private final DigitalInput limswTop;
    private final DigitalInput limswBot;
    private PIDController PIDshoot;
    private SimpleMotorFeedforward sff1;
    private PIDController PIDhood;
    //private PIDController PIDhoodPos;
    private boolean isBrake; // so we can set the hood motor to brake, if that's what we decide

    public Shood() {
        
        hoodM = new TalonFX(Constants.hoodM_port_num);
        shooterM = new TalonFX(Constants.shootM_port_num);
        
        shooterM.configFactoryDefault();
        hoodM.configFactoryDefault();

        limswTop = new DigitalInput(Constants.top_lim_num);
        limswBot = new DigitalInput(Constants.bot_lim_num);
        PIDshoot = new PIDController(Constants.Kp_shooter,
                Constants.Ki_shooter, Constants.Kd_shooter);
        //PIDhood = new PIDController(Constants.commandKp,
                //Constants.commandKi, Constants.commandKd);
        sff1 = new SimpleMotorFeedforward(Constants.hood_ks, Constants.hood_kv);

        //PIDhoodPos = new PIDController(Constants.hoodKp, Constants.hoodKi, Constants.hoodKd);

        shooterM.config_kP(0, Constants.Kp_shooter);
        shooterM.config_kI(0, Constants.Ki_shooter);
        shooterM.config_kP(0, Constants.Kd_shooter);

        hoodM.config_kP(0, Constants.Kp_hood);
        hoodM.config_kI(0, Constants.Ki_hood);
        hoodM.config_kP(0, Constants.Kp_hood);
        // setDefaultCommand(new RunCommand(()->stopHoodandShooter(),this));
    }

    //I figured I'll write the function names so I can
    // call them in the commands and you'll actually write them,
    // also add the dashboard stuff and any necessary functions I might have
    // forgotten, yay!

    // public Command getShootingCommand(){
    //     return new ShootCommand();
    // }

    public double getHoodAngle() {
        // if (limitTopReached()) {
        //     hoodM.setSelectedSensorPosition(0);
        // }
        // if (limitBotReached()) {
        //     hoodM.setSelectedSensorPosition(Constants.pulsesPer360angle);
        // }
        // return hoodM.getSelectedSensorPosition() / Constants.pulsesPer360angle;
        return hoodM.getSelectedSensorPosition();
    }

    public double getShooterSpeed() {
        return shooterM.getSelectedSensorVelocity() / Constants.pulsesPerMeterS * 10;
    }


    public void setShooterSpeed(double speed) {
        shooterM.set(ControlMode.Velocity, speed * Constants.pulsesPerMeterS / 10, DemandType.ArbitraryFeedForward,
                Constants.shooks * Math.signum(speed) +
                        speed * Constants.shookv +
                        (speed * speed) * Constants.shookv2);
    }

    public void setHoodPower(double p) {
        hoodM.set(ControlMode.PercentOutput, p);
    }

    public void setHoodVel(double v) { // v is in rotations per second
        hoodM.set(ControlMode.Velocity, v * Constants.pulsesPer360angle / 10,
                DemandType.ArbitraryFeedForward, sff1.calculate(v));
    }

    public void setHoodPosition(double a) {
        hoodM.set(ControlMode.Position, a * Constants.pulsesPer360angle);
    }

    public void setShootPower(double p) {
        shooterM.set(ControlMode.PercentOutput, p);
    }

    public void setNeutralModeHood(boolean isBrake) {
        if (isBrake) {
            hoodM.setNeutralMode(NeutralMode.Brake);
        } else {
            hoodM.setNeutralMode(NeutralMode.Coast);
        }

    }

    public boolean limitTopReached() {
        return limswTop.get();
    }

    public boolean limitBotReached() {
        return limswBot.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target velocity", 0);
        SmartDashboard.putNumber("target angle", 0);

        builder.addDoubleProperty("Shooter Velocity", this::getShooterSpeed, null);
        builder.addDoubleProperty("Hood angle", this::getHoodAngle, null);

        SmartDashboard.putData("set velocity of shooter", new RunCommand(() -> setVDashboard(), this));
        SmartDashboard.putData("set angle of hood", new RunCommand(() -> setPosHood(), this));

    }

    public void setVDashboard() {
        double v = SmartDashboard.getNumber("target velocity", 0);
        setShooterSpeed(v);
    }

    public void setPosHood() {
        double angle = SmartDashboard.getNumber("target angle", 0);
        setHoodPosition(angle);
    }
    public void stopHoodandShooter(){
        setHoodPower(0);
        setShootPower(0);
    }

}
