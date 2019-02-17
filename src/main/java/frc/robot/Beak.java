package frc.robot;

public class Beak {
    private boolean isOpen;
    private boolean isOut;

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

    void moveOut(){
        //TODO move beak out
        isOut = true;
    }

    void moveIn(){
        //TODO move beak in
        isOut = false;
    }

    boolean isOpen(){
        return isOpen;
    }    

    boolean isOut(){
        return isOut();
    }
}