package frc.robot;

public class mindsensorsJNI {
    
	private static boolean libraryLoaded = false;

	static {
		if (!libraryLoaded) {
			try {
				System.loadLibrary("mindsensorsDriver");
			} catch (UnsatisfiedLinkError e) {
				e.printStackTrace();
				System.exit(1);
			}
			libraryLoaded = true;
		}
	}

}