import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
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
  public static final String SIGHT = "sight";
  public static final String CEIL = "ceil";
  public static final String FLOOR = "floor";
  public static final String HEIGHT = "height";
  
  public static final String ATTY = "y";
  public static final String CLASSAGENT = "agent";

  public static final String ACTIONUP = "up";
  public static final String ACTIONDOWN = "down";
  public static final String ACTIONSTAY = "stay";

  public static final String PFEND = "end";


  private int [][] map;


  @Override
  public Domain generateDomain()
  {
    SADomain domain = new SADomain();
    
    Attribute time = new Attribute(domain, TIME, AttributeType.INT);
    time.setLims(0, 1000000);
    time.hidden = true;
    
    Attribute yatt = new Attribute(domain, ATTY, AttributeType.INT);    
    Attribute height = new Attribute(domain, HEIGHT, AttributeType.INT);
    
    ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
    agentClass.addAttribute(yatt);
    agentClass.addAttribute(time);

    Attribute ceil = new Attribute(domain, CEIL, AttributeType.INTARRAY);
    Attribute floor = new Attribute(domain, FLOOR, AttributeType.INTARRAY);

    ObjectClass sight = new ObjectClass(domain, SIGHT);
    sight.addAttribute(ceil);
    sight.addAttribute(floor);
    sight.addAttribute(height);

    new Movement(ACTIONSTAY, domain, 0);
    new Movement(ACTIONUP, domain, 1);
    new Movement(ACTIONDOWN, domain, 2);
    //System.out.println(sight.numObservableAttributes());

    return domain;
  }

  public State initialState(Domain domain, int height, int length, int gap, int ahead){
    State s = new State();

    ObjectInstance agent = new ObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
    agent.setValue(ATTY, height/2);
    agent.setValue(TIME, 0);
    s.addObject(agent);

    map = new int[2][length];
    map[0][0] = height/2 - gap/2 - 1;
    map[1][0] = height - map[0][0]-gap;
    
    for (int i=1; i<map[0].length; i++)
    {
      int pastceil = map[0][i-1];
      
      if (pastceil == 0)
      {
        map[0][i] = pastceil + ((int)(2*Math.random()));
        map[1][i] = height - map[0][i] -gap;        
      }
      else if (pastceil == height-gap)
      {
        map[0][i] = pastceil - ((int)(2*Math.random()));
        map[1][i] = height - map[0][i]-gap;        
      }
      else
      {
        map[0][i] = pastceil + ((int)(3*Math.random()) - 1);
        map[1][i] = height - map[0][i]-gap;
      }
    }
    
    
    ObjectInstance sight = new ObjectInstance(domain.getObjectClass(SIGHT), "sight");
    int[] ceil = new int[ahead];
    int[] floor = new int[ahead];
    
    for (int i=0; i<ceil.length; i++)
    {
      ceil[i] = map[0][i];
      floor[i] = map[1][i];
    }
    sight.setValue(CEIL, ceil);
    sight.setValue(FLOOR, floor);
    sight.setValue(HEIGHT, height);
    s.addObject(sight);

//    System.out.println(Arrays.toString(sight.getObservableFeatureVec()));
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
    
    public int[][] nextCeilFloor(State s)
    {
      ObjectInstance sight = s.getObject(SIGHT);
      int[] ceil = new int[sight.getIntArrayValue(CEIL).length];
      int[] floor = new int[ceil.length];
      
      for (int i=0; i<ceil.length; i++)
      {
        ceil[i] = map[0][i+1];
        floor[i] = map[1][i+1];
      }
      
      return new int[][]{ceil, floor};
    }

    public int moveResult(int curY, int dir)
    {
      int dy = 0;
      if (dir == 1) dy=1;
      if (dir == 2) dy=-1;

      return curY + dy;
    }

    @Override
    protected State performActionHelper(State s, String[] params)
    {
      ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
      int curY = agent.getDiscValForAttribute(ATTY);
      
      //UPDATE SIGHT
      ObjectInstance sight = s.getObject(SIGHT);
      int[][] arr = nextCeilFloor(s);
      sight.setValue(CEIL, arr[0]);
      sight.setValue(FLOOR, arr[1]);
      
      //UPDATE MAP
      for (int i=0; i<map[0].length-1; i++)
      {
        map[0][i] = map[0][i+1];
        map[1][i] = map[1][i+1];
      }
      
      int length = map[0].length;
      int pastceil = map[0][length-2];
      int height = sight.getDiscValForAttribute(HEIGHT);
      int gap = height - map[0][length-2] - map[1][length-2];
      
      if (pastceil == 0)
      {
        map[0][length-1] = pastceil + ((int)(2*Math.random()));
        map[1][length-1] = height - map[0][length-1] -gap;        
      }
      else if (pastceil == height-gap)
      {
        map[0][length-1] = pastceil - ((int)(2*Math.random()));
        map[1][length-1] = height - map[0][length-1]-gap;        
      }
      else
      {
        map[0][length-1] = pastceil + ((int)(3*Math.random()) - 1);
        map[1][length-1] = height - map[0][length-1]-gap;
      }
      
      //System.out.println(Arrays.toString(sight.getIntArrayValue(CEIL)));

      //UPDATE AGENT
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

      //set the new position
      //agent.setValue(ATTX, newPos[0]);
      agent.setValue(ATTY, this.moveResult(curY, dir));
      agent.setValue(TIME, agent.getDiscValForAttribute(TIME)+ 1);

      //return the state we just modified
      return s;
    }



    @Override
    public List getTransitions(State s, String [] params)
    {
      List<TransitionProbability> tps = new ArrayList<TransitionProbability>(3);
      for(int i = 0; i < this.directionProbs.length; i++){

        //new possible outcome
        State ns = s.copy();
        ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
        int curY = nagent.getDiscValForAttribute(ATTY);
        nagent.setValue(ATTY, this.moveResult(curY, i));
        nagent.setValue(TIME, nagent.getDiscValForAttribute(TIME)+ 1);

        ObjectInstance nsight = ns.getObject(SIGHT);
        int[][] arr = nextCeilFloor(ns);
        nsight.setValue(CEIL, arr[0]);
        nsight.setValue(FLOOR, arr[1]);
        
        //create transition probability object and add to our list of outcomes
        tps.add(new TransitionProbability(ns, this.directionProbs[i]));
      }


      return tps;
    } 
  }

  public static boolean crash (State s)
  {
    ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
    int ay = agent.getDiscValForAttribute(ATTY);

    ObjectInstance sight = s.getFirstObjectOfClass(SIGHT);
    int[] ceil = sight.getIntArrayValue(CEIL);
    int[] floor = sight.getIntArrayValue(FLOOR);
    int height = sight.getDiscValForAttribute(HEIGHT);
    
    return ay+1 <= floor[0] || ay >= height - ceil[0]; 
  }

  public class HeliRF implements RewardFunction{   
    @Override
    public double reward(State s, GroundedAction a, State sprime) {      

      ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
      int ay = agent.getDiscValForAttribute(ATTY);
      int t = agent.getDiscValForAttribute(TIME);
      
      ObjectInstance sight = s.getFirstObjectOfClass(SIGHT);
      int[] ceil = sight.getIntArrayValue(CEIL);
      int[] floor = sight.getIntArrayValue(FLOOR);
      int height = sight.getDiscValForAttribute(HEIGHT);

      if (crash(sprime)) return -1000;
      else
      {
        int gap = height - ceil[0] - floor[0];
        double center = floor[0] + gap/2.0;
        return 100-100*(center - ay)*(center - ay);
//        return 100;
      }

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
      int t = agent.getDiscValForAttribute(TIME);

      //are they at goal location?
      if(t == timelimit || crash(s)){
        return true;
      }

      return false;
    }
  }

  //Visualization
  public class WallPainter implements StaticPainter{

    @Override
    public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {

      ObjectInstance sight = s.getObject(SIGHT);
      int[] ceil = sight.getIntArrayValue(CEIL);
      int[] floor = sight.getIntArrayValue(FLOOR);
      int h = sight.getDiscValForAttribute(HEIGHT);

      //walls will be filled in black
      g2.setColor(Color.BLACK);

      //set up floats for the width and height of our domain
      float fWidth = map[0].length;
      float fHeight = h;

      //determine the width of a single cell on our canvas 
      //such that the whole sight can be painted
      float width = cWidth / fWidth;
      float height = cHeight / fHeight;

      //pass through each cell of our sight and if it's a wall, paint a black 
      //rectangle on our canvas of dimension widthxheight
      for(int i = 0; i < map[0].length; i++){
        for(int j = 0; j < h; j++){

          //is there a wall here?
          if(j+1 <= map[1][i] || j >= h - map[0][i]){

            //left coordinate of cell on our canvas
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

      ObjectInstance sight = s.getObject(SIGHT);
      int h = sight.getDiscValForAttribute(HEIGHT);

      //System.out.println(padding(s));
      
      //agent will be filled in blue
      g2.setColor(Color.BLUE);

      //set up floats for the width and height of our domain
      float fWidth = map[0].length;
      float fHeight = h;

      //determine the width of a single cell on our canvas 
      //such that the whole sight can be painted
      float width = cWidth / fWidth;
      float height = cHeight / fHeight;

      int ay = ob.getDiscValForAttribute(ATTY);

      //left coordinate of cell on our canvas
      float rx = 0;

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

    int height = 10;
    int length = 10;
    int ahead = 2;
    int gap = 3;
    
    State initialState = heli.initialState(domain, height, length, gap, ahead);

    Visualizer v = heli.getVisualizer();
    VisualExplorer exp = new VisualExplorer(domain, v, initialState);

    exp.addKeyAction("w", ACTIONUP);
    exp.addKeyAction("s", ACTIONDOWN);
    exp.addKeyAction("d", ACTIONSTAY);

    exp.initGUI();

  }
}
