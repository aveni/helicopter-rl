import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.SDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.auxiliary.common.UniversalStateParser;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.PropositionalFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.ObjectPainter;
import burlap.oomdp.visualizer.StateRenderLayer;
import burlap.oomdp.visualizer.StaticPainter;
import burlap.oomdp.visualizer.Visualizer;

public class Helicopter implements DomainGenerator
{
  public static final String TIME = "time";
  public static final String ATTX = "x";
  public static final String ATTY = "y";
  public static final String CLASSAGENT = "agent";
  
  public static final String ACTIONUP = "up";
  public static final String ACTIONDOWN = "down";
  public static final String ACTIONSTAY = "stay";

  
  public static final String PFEND = "end";


  protected int [][] map = new int[][]{
      {1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
      {1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
      {1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1},
      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
      {1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
      {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1}
  };
  
//  protected int [][] map = new int[][]{
//      {1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
//      {1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1},
//      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
//      {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1},
//      {1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1},
//      {1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1},
//      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1},
//      {1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
//      {1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1}
//  };
  
  @Override
  public Domain generateDomain()
  {

    
    SADomain domain = new SADomain();
    
    Attribute time = new Attribute(domain, TIME, AttributeType.INT);
    time.setLims(0, 1000000);
    Attribute xatt = new Attribute(domain, ATTX, AttributeType.INT);
    xatt.setLims(0, 10);
    
    Attribute yatt = new Attribute(domain, ATTY, AttributeType.INT);
    yatt.setLims(0, 10);

    ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
    agentClass.addAttribute(xatt);
    agentClass.addAttribute(yatt);
    agentClass.addAttribute(time);
    
    new Movement(ACTIONSTAY, domain, 0);
    new Movement(ACTIONUP, domain, 1);
    new Movement(ACTIONDOWN, domain, 2);
    
    return domain;
  }
  
  
  public static State makeState(Domain domain, int h){
    State s = new State();
    ObjectInstance agent = new ObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
    agent.setValue(ATTX, 0);
    agent.setValue(ATTY, h);
    agent.setValue(TIME, 0);

    s.addObject(agent);
    
    return s;
  }
  
  protected class Movement extends Action
  {   
    protected double [] directionProbs = new double[3];
    
    public Movement(String actionName, Domain domain, int direction)
    {
      super(actionName, domain, "");
      for(int i = 0; i < 3; i++){
        if(i == direction){
          directionProbs[i] = 1.0;
        }
        else{
          directionProbs[i] = 0;
        }
      }
    }
    
    protected int [] moveResult(int curX, int curY, int direction){
      
      int xdelta = 1;
      int ydelta = 0;
      if(direction == 0)
      {
        ydelta = 0;
      }
      else if (direction == 1)
      {
        ydelta = 1;
      }
      else
      {
        ydelta = -1;
      }
     
      int nx = (curX + xdelta)%(Helicopter.this.map.length);
      int ny = curY + ydelta;
      
      int width = Helicopter.this.map.length;
      int height = Helicopter.this.map[0].length;
      
      //make sure new position is valid (not a wall or out of bounds)
      if(ny < 0 || ny >= height){
        ny = curY;
      }
        
      
      return new int[]{nx,ny};
      
    }

    @Override
    protected State performActionHelper(State s, String[] params)
    {
      ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
      int curX = agent.getDiscValForAttribute(ATTX);
      int curY = agent.getDiscValForAttribute(ATTY);
      
      //sample directon with random roll
      double r = Math.random();
      double sumProb = 0.;
      int dir = 0;
      for(int i = 0; i < this.directionProbs.length; i++){
        sumProb += this.directionProbs[i];
        if(r < sumProb){
          dir = i;
          break; //found direction
        }
      }
      
      //get resulting position
      int [] newPos = this.moveResult(curX, curY, dir);
      
      //set the new position
      agent.setValue(ATTX, newPos[0]);
      agent.setValue(ATTY, newPos[1]);
      agent.setValue(TIME, agent.getDiscValForAttribute(TIME)+ 1);
      
      //return the state we just modified
      return s;
    }
    
    protected State performActionHelper2(State s, String[] params)
    {
      ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
      int curX = agent.getDiscValForAttribute(ATTX);
      int curY = agent.getDiscValForAttribute(ATTY);
      
      //sample directon with random roll
      double r = Math.random();
      double sumProb = 0.;
      int dir = 0;
      for(int i = 0; i < this.directionProbs.length; i++){
        sumProb += this.directionProbs[i];
        if(r < sumProb){
          dir = i;
          break; //found direction
        }
      }
      
      //get resulting position
      int [] newPos = this.moveResult(curX, curY, dir);
      
      //set the new position
      agent.setValue(ATTX, newPos[0]);
      agent.setValue(ATTY, newPos[1]);
      agent.setValue(TIME, agent.getDiscValForAttribute(TIME)+ 1);
      
      //return the state we just modified
      return s;
    }
    
    
    @Override
    public List getTransitions(State s, String [] params)
    {
    //get agent and current position
      ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
      int curX = agent.getDiscValForAttribute(ATTX);
      int curY = agent.getDiscValForAttribute(ATTY);
      
      List<TransitionProbability> tps = new ArrayList<TransitionProbability>(4);
      TransitionProbability noChangeTransition = null;
      for(int i = 0; i < this.directionProbs.length; i++){
        int [] newPos = this.moveResult(curX, curY, i);
        if(newPos[0] != curX || newPos[1] != curY){
          //new possible outcome
          State ns = s.copy();
          ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
          nagent.setValue(ATTX, newPos[0]);
          nagent.setValue(ATTY, newPos[1]);
          nagent.setValue(TIME, nagent.getDiscValForAttribute(TIME)+ 1);

          
          //create transition probability object and add to our list of outcomes
          tps.add(new TransitionProbability(ns, this.directionProbs[i]));
        }
        else{
          //this direction didn't lead anywhere new
          //if there are existing possible directions that wouldn't lead anywhere, 
          //aggregate with them
          if(noChangeTransition != null){
            noChangeTransition.p += this.directionProbs[i];
          }
          else{
            //otherwise create this new state outcome
            noChangeTransition = new TransitionProbability(s.copy(), this.directionProbs[i]);
            tps.add(noChangeTransition);
          }
        }
      }
         
      return tps;
    } 
  }

  public int padding(State s)
  {
    ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
    int ax = agent.getDiscValForAttribute(ATTX);
    int ay = agent.getDiscValForAttribute(ATTY);
    
    int height = Helicopter.this.map[0].length;
    int pad = 0;
    
    for (int i=0; i<=ay; i++)
    {
      if (Helicopter.this.map[ax][i] == 0)
      {
        pad += (ay-i)*(ay-i);
        break;
      }
    }
    for (int i=height-1; i>=ay; i--)
    {
      if (Helicopter.this.map[ax][i] == 0)
      {
        pad -= (i-ay)*(i-ay);
        break;
      }
    }
    return height*height/2 - Math.abs(pad);  
  }
  
  
  public class HeliRF implements RewardFunction{   
    @Override
    public double reward(State s, GroundedAction a, State sprime) {
      ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
      int ax = agent.getDiscValForAttribute(ATTX);
      int ay = agent.getDiscValForAttribute(ATTY);
      if (Helicopter.this.map[ax][ay] == 1) return -1000;
      
      return padding(sprime);
    }
  }
  
  public class HeliGC implements StateConditionTest
  {
    private int timelimit;
    
    public HeliGC(int timelimit)
    {
      this.timelimit = timelimit;
    }
    
    public boolean satisfies(State s)
    {
      ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
      int t = agent.getDiscValForAttribute(TIME);
      
      return t == timelimit;
    }
  }
  
  public class HeliTF implements TerminalFunction{
    
    private int timelimit;

    public HeliTF(int timelimit)
    {
      this.timelimit = timelimit;
    }
    
    
    @Override
    public boolean isTerminal(State s) {
      
      //get location of agent in next state
      ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
      int ax = agent.getDiscValForAttribute(ATTX);
      int ay = agent.getDiscValForAttribute(ATTY);
      int t = agent.getDiscValForAttribute(TIME);
      
      //are they at goal location?
      if(t == timelimit || Helicopter.this.map[ax][ay] == 1){
        return true;
      }
      
      return false;
    }
  }
   
  public int[][] getMap()
  {
    return map;
  }
  
  
  
  //Visualization
  public class WallPainter implements StaticPainter{

    @Override
    public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {
      
      //walls will be filled in black
      g2.setColor(Color.BLACK);
      
      //set up floats for the width and height of our domain
      float fWidth = Helicopter.this.map.length;
      float fHeight = Helicopter.this.map[0].length;
      
      //determine the width of a single cell on our canvas 
      //such that the whole map can be painted
      float width = cWidth / fWidth;
      float height = cHeight / fHeight;
      
      //pass through each cell of our map and if it's a wall, paint a black 
      //rectangle on our cavas of dimension widthxheight
      for(int i = 0; i < Helicopter.this.map.length; i++){
        for(int j = 0; j < Helicopter.this.map[0].length; j++){
          
          //is there a wall here?
          if(Helicopter.this.map[i][j] == 1){
          
            //left corrdinate of cell on our canvas
            float rx = i*width;
            
            //top coordinate of cell on our canvas
            //coordinate system adjustment because the java canvas
            //origin is in the top left instead of the bottom right
            float ry = cHeight - height - j*height;
          
            //paint the rectangle
            g2.fill(new Rectangle2D.Float(rx, ry, width, height));
            
          }        
        }
      }    
    }
  }
  
  public class AgentPainter implements ObjectPainter{

    @Override
    public void paintObject(Graphics2D g2, State s, ObjectInstance ob,
        float cWidth, float cHeight) {
      
      System.out.println(padding(s));
      //agent will be filled in gray
      g2.setColor(Color.GRAY);
      
      //set up floats for the width and height of our domain
      float fWidth = Helicopter.this.map.length;
      float fHeight = Helicopter.this.map[0].length;
      
      //determine the width of a single cell on our canvas 
      //such that the whole map can be painted
      float width = cWidth / fWidth;
      float height = cHeight / fHeight;
      
      int ax = ob.getDiscValForAttribute(ATTX);
      int ay = ob.getDiscValForAttribute(ATTY);
      
      //left coordinate of cell on our canvas
      float rx = ax*width;
      
      //top coordinate of cell on our canvas
      //coordinate system adjustment because the java canvas 
      //origin is in the top left instead of the bottom right
      float ry = cHeight - height - ay*height;
    
      //paint the rectangle
      g2.fill(new Ellipse2D.Float(rx, ry, width, height));
    }
  }
  
  public StateRenderLayer getStateRenderLayer(){
    StateRenderLayer rl = new StateRenderLayer();
    rl.addStaticPainter(new WallPainter());
    rl.addObjectClassPainter(CLASSAGENT, new AgentPainter());
    
    return rl;
  }

  public Visualizer getVisualizer(){
    return new Visualizer(this.getStateRenderLayer());
  }
  
  
  public static void main(String [] args){
    
    Helicopter heli = new Helicopter();
    Domain domain = heli.generateDomain();
    
    State initialState = Helicopter.makeState(domain,8);
    

    Visualizer v = heli.getVisualizer();
    VisualExplorer exp = new VisualExplorer(domain, v, initialState);
    
    exp.addKeyAction("w", ACTIONUP);
    exp.addKeyAction("s", ACTIONDOWN);
    exp.addKeyAction("d", ACTIONSTAY);
    
    exp.initGUI();
    
  }
}
