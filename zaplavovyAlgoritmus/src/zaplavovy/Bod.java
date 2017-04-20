/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package zaplavovy;

import java.util.List;

/**
 *
 * @author Juraj
 */
public class Bod {
    private int x;
    private int y;
    
    public Bod(int x,int y){
        this.x = x;
        this.y = y;
    }

    public Bod clone(){
        return new Bod(x,y);
    }
    
    /**
     * @return the x
     */
    public int getX() {
        return x;
    }

    /**
     * @param x the x to set
     */
    public void setX(int x) {
        this.x = x;
    }

    /**
     * @return the y
     */
    public int getY() {
        return y;
    }

    /**
     * @param y the y to set
     */
    public void setY(int y) {
        this.y = y;
    }

    public boolean equals(Bod bod){
        return this.x == bod.getX() && this.y == bod.getY();
    }
}
