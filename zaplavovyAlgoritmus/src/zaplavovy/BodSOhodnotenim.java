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
public class BodSOhodnotenim {
    private Bod bod;
    private int ohodnotenie;
    
    public BodSOhodnotenim(Bod bod, int ohodnotenie){
        this.bod = bod;
        this.ohodnotenie = ohodnotenie;
    }
    public BodSOhodnotenim(int x,int y, int ohodnotenie){
        this.bod = new Bod(x,y);
        this.ohodnotenie = ohodnotenie;
    }


    /**
     * @return the ohodnotenie
     */
    public int getOhodnotenie() {
        return ohodnotenie;
    }

    /**
     * @param ohodnotenie the ohodnotenie to set
     */
    public void setOhodnotenie(int ohodnotenie) {
        this.ohodnotenie = ohodnotenie;
    }
    
    static int indexOhodnotenie(List<BodSOhodnotenim> body,int hladaneOhodnotenie){
        for(int i=0;i<body.size();i++){
            if(body.get(i).getOhodnotenie() == hladaneOhodnotenie) return i;
        }
        return -1;
    }

    /**
     * @return the bod
     */
    public Bod getBod() {
        return bod;
    }

    /**
     * @param bod the bod to set
     */
    public void setBod(Bod bod) {
        this.bod = bod;
    }
}
