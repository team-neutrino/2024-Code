package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.awt.Cursor;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.HashSet;

import javax.swing.JFrame;

public class RobotInputListener extends JFrame implements KeyListener, MouseListener, MouseMotionListener {
    // NetworkTables instance and entries for sending keyboard and mouse states
    private NetworkTableInstance networkTableInstance;
    private NetworkTable inputTable;
    private NetworkTableEntry keyEntry;
    private NetworkTableEntry mouseButtonEntry;
    private NetworkTableEntry mouseXEntry;
    private NetworkTableEntry mouseYEntry;

    private static HashSet<String> activeKeyboardInputs = new HashSet<>();
    private static HashSet<Integer> activeMouseButtons = new HashSet<>();

    public RobotInputListener() {
        // Initialize NetworkTables
        networkTableInstance = NetworkTableInstance.getDefault();
        inputTable = networkTableInstance.getTable("InputData");

        // Entries for keyboard and mouse data
        keyEntry = inputTable.getEntry("KeyPressed");
        mouseButtonEntry = inputTable.getEntry("MouseButton");
        mouseXEntry = inputTable.getEntry("MouseX");
        mouseYEntry = inputTable.getEntry("MouseY");

        // Set up JFrame to capture keyboard and mouse events
        this.setTitle("FRC Keyboard & Mouse Listener");
        // width 1535 full screen, current bounds: 10 to 1546
        this.setSize(1555, 650);
        this.setAlwaysOnTop(true);
        this.setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
        this.setLocation(-10, 0);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setVisible(true);
        this.addKeyListener(this);
        this.addMouseListener(this);
        this.addMouseMotionListener(this);

        // Start NetworkTables client and connect to robot
        networkTableInstance.startDSClient();
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        System.out.println("mouse entered screen");
    }

    @Override
    public void mouseExited(MouseEvent e) {
        System.out.println("mouse exited screen");
    }

    // Keyboard event handling
    @Override
    public void keyPressed(KeyEvent e) {
        String key = "" + e.getKeyChar();
        activeKeyboardInputs.add(key);
        keyEntry.setString(key);
        System.out.println("Key Pressed: " + KeyEvent.getKeyText(e.getKeyCode()));
    }

    @Override
    public void keyReleased(KeyEvent e) {
        activeKeyboardInputs.remove("" + e.getKeyChar());
        keyEntry.setString("");
        System.out.println("Key Released: " + KeyEvent.getKeyText(e.getKeyCode()));
    }

    @Override
    public void keyTyped(KeyEvent e) {
        // Optional: Handle if needed
    }

    // Mouse button event handling
    @Override
    public void mousePressed(MouseEvent e) {
        int button = e.getButton();
        activeMouseButtons.add(button);
        mouseButtonEntry.setDouble(button);
        System.out.println("Mouse Button Pressed: " + button);
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        activeMouseButtons.remove(e.getButton());
        mouseButtonEntry.setDouble(0); // Reset on release
        System.out.println("Mouse Button Released: " + e.getButton());
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        // Optional: use if the event of a mouse button being pressed AND released needs
        // to be handled separately
    }

    // Mouse movement tracking
    @Override
    public void mouseMoved(MouseEvent e) {
        int x = e.getX();
        int y = e.getY();
        mouseXEntry.setDouble(x);
        mouseYEntry.setDouble(y);
        System.out.println("Mouse Moved to: (" + x + ", " + y + ")");
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        // Similar to mouseMoved, but for dragging events
        int x = e.getX();
        int y = e.getY();
        mouseXEntry.setDouble(x);
        mouseYEntry.setDouble(y);
        System.out.println("Mouse Dragged to: (" + x + ", " + y + ")");
    }

    public boolean getIsKeyActive(String ch) {
        return activeKeyboardInputs.contains(ch);
    }

    public boolean getIsMouseButtonActive(int val) {
        return activeMouseButtons.contains(val);
    }

    public static void main(String[] args) {
        new RobotInputListener();
    }
}
