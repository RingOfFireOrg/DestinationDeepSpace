package frc.robot;

public class Beak {
    static private boolean isOpen;

    static void open(){
        // TODO something that opens beak
        isOpen = true;
    }

    static void close(){
        //TODO something that closes beak
        isOpen = false;
    }

    static boolean isOpen(){
        return isOpen;
    }    
}