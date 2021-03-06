import burlap.behavior.singleagent.*;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.domain.singleagent.gridworld.*;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.*;
import burlap.oomdp.singleagent.common.*;
import burlap.oomdp.visualizer.Visualizer;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.behavior.singleagent.planning.deterministic.*;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.singleagent.common.VisualActionObserver;

import java.awt.Color;
import java.util.List;

import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
        


public class BasicBehavior {

  
  
  GridWorldDomain       gwdg;
  Domain            domain;
  StateParser         sp;
  RewardFunction        rf;
  TerminalFunction      tf;
  StateConditionTest      goalCondition;
  State           initialState;
  DiscreteStateHashFactory  hashingFactory;
  
  public BasicBehavior(){
    
    //create the domain
    gwdg = new GridWorldDomain(11, 11);
    gwdg.setMapToFourRooms(); 
    domain = gwdg.generateDomain();
    
    //create the state parser
    sp = new GridWorldStateParser(domain); 
    
    //define the task
    rf = new UniformCostRF(); 
    tf = new SinglePFTF(domain.getPropFunction(GridWorldDomain.PFATLOCATION)); 
    goalCondition = new TFGoalCondition(tf);
    
    //set up the initial state of the task
    initialState = GridWorldDomain.getOneAgentOneLocationState(domain);
    GridWorldDomain.setAgent(initialState, 0, 0);
    GridWorldDomain.setLocation(initialState, 0, 10, 10);
    
    //set up the state hashing system
    hashingFactory = new DiscreteStateHashFactory();
    hashingFactory.setAttributesForClass(GridWorldDomain.CLASSAGENT, 
    domain.getObjectClass(GridWorldDomain.CLASSAGENT).attributeList); 
  
    VisualActionObserver observer = new VisualActionObserver(domain, 
    GridWorldVisualizer.getVisualizer(gwdg.getMap()));
    ((SADomain)this.domain).setActionObserverForAllAction(observer);
    observer.initGUI();
  }
  
  public void visualize(String outputPath){
    Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
    EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, sp, outputPath);
  }
  
  public void valueFunctionVisualize(QComputablePlanner planner, Policy p){
    List <State> allStates = StateReachability.getReachableStates(initialState, 
      (SADomain)domain, hashingFactory);
    LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
    rb.addNextLandMark(0., Color.RED);
    rb.addNextLandMark(1., Color.BLUE);
    
    StateValuePainter2D svp = new StateValuePainter2D(rb);
    svp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX, 
      GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
    
    PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
    spp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX, 
      GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
    spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
    spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
    spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
    spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
    spp.setRenderStyle(PolicyGlyphRenderStyle.DISTSCALED);
    
    ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, planner);
    gui.setSpp(spp);
    gui.setPolicy(p);
    gui.setBgColor(Color.GRAY);
    gui.initGUI();
}
  
  public void BFSExample(String outputPath){
    
    if(!outputPath.endsWith("/")){
      outputPath = outputPath + "/";
    }
    
    //BFS ignores reward; it just searches for a goal condition satisfying state
    DeterministicPlanner planner = new BFS(domain, goalCondition, hashingFactory); 
    planner.planFromState(initialState);
    
    //capture the computed plan in a partial policy
    Policy p = new SDPlannerPolicy(planner);
    
    //record the plan results to a file
    p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "planResult", sp);
      
  }
  

  public void ValueIterationExample(String outputPath){
    
    if(!outputPath.endsWith("/")){
      outputPath = outputPath + "/";
    }
    
    
    OOMDPPlanner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 100);
    planner.planFromState(initialState);
    
    //create a Q-greedy policy from the planner
    Policy p = new GreedyQPolicy((QComputablePlanner)planner);
    
    //record the plan results to a file
    p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "planResult", sp);
    
    //view policy
    this.valueFunctionVisualize((QComputablePlanner)planner, p);    

  }
  
  public void QLearningExample(String outputPath){
    
    if(!outputPath.endsWith("/")){
      outputPath = outputPath + "/";
    }
    
    //creating the learning algorithm object; discount= 0.99; initialQ=0.0; learning rate=0.9
    LearningAgent agent = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0., 0.9);
    
    //run learning for 100 episodes
    for(int i = 0; i < 100; i++){
      EpisodeAnalysis ea = agent.runLearningEpisodeFrom(initialState);
      ea.writeToFile(String.format("%se%03d", outputPath, i), sp); 
      System.out.println(i + ": " + ea.numTimeSteps());
    }
    
  }   
  
  public static void main(String[] args) {
    
    BasicBehavior example = new BasicBehavior();
    String outputPath = "output/"; //directory to record results
    
    //we will call planning and learning algorithms here
    example.QLearningExample(outputPath);

    
    //run the visualizer
    example.visualize(outputPath);

  }
  
}
        