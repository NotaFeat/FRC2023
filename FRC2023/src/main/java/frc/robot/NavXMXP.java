package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;
public class NavXMXP implements IGyroscopeSensor{
    
    private AHRS navX;

    public NavXMXP() {
        Debug.log("Start");
        navX = new AHRS(SPI.Port.kMXP);
        navX.enableBoardlevelYawReset(false);
        
    }
    
    @Override
    public double getPitch() {
        return Math.toRadians(navX.getPitch()); 
    }

    @Override
    public double getRoll() {
        return Math.toRadians(navX.getRoll());
    }

    @Override
    public double getYaw() {
        return Math.toRadians(navX.getYaw());
    }

    @Override
    public void resetYaw() {
        navX.reset();
    }
}
