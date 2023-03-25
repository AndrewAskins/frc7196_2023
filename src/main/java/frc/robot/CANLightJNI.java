package frc.robot;

public class CANLightJNI extends mindsensorsJNI {
    
    public static native String GetLibraryVersion();
    
	public static native int Constructor(int deviceNumber);
    public static native void Destructor(int handle);
	
	public static native int GetDeviceID(int handle);
	public static native String GetDeviceName(int handle);
	public static native String GetFirmwareVersion(int handle);
	public static native String GetHardwareVersion(int handle);
	public static native String GetBootloaderVersion(int handle);
	public static native String GetSerialNumber(int handle);
	
	public static native void BlinkLED(int handle, int seconds);
	
	public static native void ShowRGB(int handle, int red, int green, int blue);
	public static native void WriteRegister(int handle, int index, int time, int red, int green, int blue);
	public static native void Reset(int handle);
	public static native void ShowRegister(int handle, int index);
	public static native void Flash(int handle, int index);
	public static native void Cycle(int handle, int fromIndex, int toIndex);
	public static native void Fade(int handle, int startIndex, int endIndex);
    
    public static native double GetBatteryVoltage(int handle);
	
}
