package frc.robot;

public class Beak {
    private boolean isOpen;

    public Beak(){
        
    }

    void open(){
        // TODO something that opens beak
        isOpen = true;
    }

    void close(){
        //TODO something that closes beak
        isOpen = false;
    }

    boolean isOpen(){
        return isOpen;
    }    
}