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
  public static final String MAP = "map";
  public static final String CEIL = "ceil";
  public static final String FLOOR = "floor";
  public static final String HEIGHT = "height";

  public static final String ATTY = "y";
  public static final String CLASSAGENT = "agent";

  public static final String ACTIONUP = "up";
  public static final String ACTIONDOWN = "down";
  public static final String ACTIONSTAY = "stay";

  public static final String PFEND = "end";


  //  protected int [][] map = new int[][]{
  //      {1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
  //      {1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1},
  //      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
  //      {1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
  //      {1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
  //      {1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1}
  //  };

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
    time.hidden = true;
    
    Attribute yatt = new Attribute(domain, ATTY, AttributeType.INT);    
    Attribute height = new Attribute(domain, HEIGHT, AttributeType.INT);
    
    ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
    agentClass.addAttribute(yatt);
    agentClass.addAttribute(time);

    Attribute ceil = new Attribute(domain, CEIL, AttributeType.INTARRAY);
    Attribute floor = new Attribute(domain, FLOOR, AttributeType.INTARRAY);
    ceil.hidden = true;
    floor.hidden = true;


    ObjectClass map = new ObjectClass(domain, MAP);
    map.addAttribute(ceil);
    map.addAttribute(floor);
    map.addAttribute(height);

    new Movement(ACTIONSTAY, domain, 0);
    new Movement(ACTIONUP, domain, 1);
    new Movement(ACTIONDOWN, domain, 2);
    System.out.println(map.numObservableAttributes());

    return domain;
  }

  public static State makeState(Domain domain, int height, int h0, int[][] start){
    State s = new State();

    ObjectInstance agent = new ObjectInstance(domain.getObjectClass(CLASSAGENT), "agent0");
    agent.setValue(ATTY, h0);
    agent.setValue(TIME, 0);
    s.addObject(agent);

    ObjectInstance map = new ObjectInstance(domain.getObjectClass(MAP), "map");
    int[] ceil = new int[start.length];
    int[] floor = new int[start.length]; 
    for (int i=0; i<ceil.length; i++)
    {
      ceil[i] = start[i][0];
      floor[i] = start[i][1];
    }
    map.setValue(CEIL, ceil);
    map.setValue(FLOOR, floor);
    map.setValue(HEIGHT, height);
    s.addObject(map);

    System.out.println(Arrays.toString(map.getObservableFeatureVec()));
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
      ObjectInstance map = s.getObject(MAP);
      int[] ceil = map.getIntArrayValue(CEIL);
      int[] floor = map.getIntArrayValue(FLOOR);
      int height = map.getDiscValForAttribute(HEIGHT);

      int[][] arr =  new int[2][ceil.length];
      for (int i=0; i<ceil.length-1; i++)
      {
        arr[0][i] = ceil[i+1];
        arr[1][i] = floor[i+1];
      }
      int gap = 4;
      int pastceil = arr[0][ceil.length-2];
      
      if (pastceil == 0)
      {
        arr[0][ceil.length-1] = pastceil + 1;
        arr[1][floor.length-1] = height - arr[0][ceil.length-1]-gap;        
      }
      else if (pastceil == height-gap)
      {
        arr[0][ceil.length-1] = pastceil - 1;
        arr[1][floor.length-1] = height - arr[0][ceil.length-1]-gap;        
      }
      else
      {
        arr[0][ceil.length-1] = pastceil + ((int)(3*Math.random()) - 1);
        arr[1][floor.length-1] = height - arr[0][ceil.length-1]-gap;
      }

      return arr;
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

      ObjectInstance map = s.getObject(MAP);
      int[][] arr = nextCeilFloor(s);
      map.setValue(CEIL, arr[0]);
      map.setValue(FLOOR, arr[1]);

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

        ObjectInstance nmap = ns.getObject(MAP);
        int[][] arr = nextCeilFloor(ns);
        nmap.setValue(CEIL, arr[0]);
        nmap.setValue(FLOOR, arr[1]);
        
        //create transition probability object and add to our list of outcomes
        tps.add(new TransitionProbability(ns, this.directionProbs[i]));
      }


      return tps;
    } 
  }

  public int padding(State s)
  {
    ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
    int ay = agent.getDiscValForAttribute(ATTY);

    ObjectInstance map = s.getFirstObjectOfClass(MAP);
    int[] ceil = map.getIntArrayValue(CEIL);
    int[] floor = map.getIntArrayValue(FLOOR);
    int height = map.getDiscValForAttribute(HEIGHT);


    int pad =0;
    pad += (ay - floor[0] + 1) * (ay - floor[0] + 1);
    pad += (height - ceil[0] - ay) * (height - ceil[0] - ay);   

    return height*height/2 - pad;
  }

  public static boolean crash (State s)
  {
    ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
    int ay = agent.getDiscValForAttribute(ATTY);

    ObjectInstance map = s.getFirstObjectOfClass(MAP);
    int[] ceil = map.getIntArrayValue(CEIL);
    int[] floor = map.getIntArrayValue(FLOOR);
    int height = map.getDiscValForAttribute(HEIGHT);
    
    return ay+1 <= floor[0] || ay >= height - ceil[0]; 
  }

  public class HeliRF implements RewardFunction{   
    @Override
    public double reward(State s, GroundedAction a, State sprime) {      

      ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
      int ay = agent.getDiscValForAttribute(ATTY);
      int t = agent.getDiscValForAttribute(TIME);

//      return -(4-ay)*(4-ay);
      if (crash(sprime)) return -1000;
      else return 100;

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

      ObjectInstance map = s.getObject(MAP);
      int[] ceil = map.getIntArrayValue(CEIL);
      int[] floor = map.getIntArrayValue(FLOOR);
      int h = map.getDiscValForAttribute(HEIGHT);

      //walls will be filled in black
      g2.setColor(Color.BLACK);

      //set up floats for the width and height of our domain
      float fWidth = ceil.length;
      float fHeight = h;

      //determine the width of a single cell on our canvas 
      //such that the whole map can be painted
      float width = cWidth / fWidth;
      float height = cHeight / fHeight;

      //pass through each cell of our map and if it's a wall, paint a black 
      //rectangle on our canvas of dimension widthxheight
      for(int i = 0; i < ceil.length; i++){
        for(int j = 0; j < h; j++){

          //is there a wall here?
          if(j+1 <= floor[i] || j >= h - ceil[i]){

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

      ObjectInstance map = s.getObject(MAP);
      int[] ceil = map.getIntArrayValue(CEIL);
      int h = map.getDiscValForAttribute(HEIGHT);

      //System.out.println(padding(s));
      
      //agent will be filled in gray
      g2.setColor(Color.GRAY);

      //set up floats for the width and height of our domain
      float fWidth = ceil.length;
      float fHeight = h;

      //determine the width of a single cell on our canvas 
      //such that the whole map can be painted
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

    int [][] start = new int[][]{
        {3,3},
        {3,2},
        {3,3},
        {2,2},
        {2,2},
        {2,3},
        {3,3},
        {2,2},
        {2,2},
        {1,2}
    };
    State initialState = Helicopter.makeState(domain,10,3, start);

    Visualizer v = heli.getVisualizer();
    VisualExplorer exp = new VisualExplorer(domain, v, initialState);

    exp.addKeyAction("w", ACTIONUP);
    exp.addKeyAction("s", ACTIONDOWN);
    exp.addKeyAction("d", ACTIONSTAY);

    exp.initGUI();

  }
}
