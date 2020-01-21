import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoard {
    // This first tab creates a field holding a title.
    String title;
    ShuffleboardTab tab;
    //This is creating a ShuffleBoard object, where it creates a base for other ShuffleBoard objects.
    public ShuffleBoard(String title) {
        
        //This saves the title from before, in which case, is "title".
        this.title = title;
        //This creates the tab.
        tab =  Shuffleboard.getTab(this.title);
    
    }
    public void printInt(String key, int number) {
        tab.add(key, number);
    }  
    public void printDouble(String key, double number) {
        tab.add(key, number);
    }
    public void printBoolean(String key, Boolean number) {
        tab.add(key, number);
    }
}
