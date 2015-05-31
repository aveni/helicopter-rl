import burlap.behavior.singleagent.*;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.SDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.domain.singleagent.gridworld.*;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.auxiliary.common.UniversalStateParser;
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
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;


import java.awt.Color;
import java.util.List;

import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;


public class HelicopterTest
{

  Helicopter heli;
  Domain domain;
  StateParser sp;        
  RewardFunction rf;
  TerminalFunction tf;
  StateConditionTest goalCondition;  
  DiscreteStateHashFactory hashingFactory;  
  State initialState;
  
  int timelimit = 100;    
  int height = 10;
  int length = 10;
  int gap = 3;
  int ahead = 1;

  public HelicopterTest(){

    //create the domain
    heli = new Helicopter();
    domain = heli.generateDomain();

    //create the state parser
    sp = new UniversalStateParser(domain); 

    //define the task

    rf = heli.new HeliRF();
    tf = heli.new HeliTF(timelimit);
    goalCondition = heli.new HeliGC(timelimit);

    //set up the initial state of the task
    initialState = heli.initialState(domain,height, length, gap, ahead);

    //set up the state hashing system
    hashingFactory = new DiscreteStateHashFactory();
    hashingFactory.setAttributesForClass(Helicopter.CLASSAGENT, 
        domain.getObjectClass(Helicopter.CLASSAGENT).attributeList);   

//        VisualActionObserver observer = new VisualActionObserver(domain, 
//        heli.getVisualizer());
//        ((SADomain)this.domain).setActionObserverForAllAction(observer);
//        observer.setFrameDelay(100);
//
//        observer.initGUI(); 
  }

  public void visualize(String outputPath){
    Visualizer v = heli.getVisualizer();
    EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, sp, outputPath);
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


  public void QLearningExample(String outputPath){

    if(!outputPath.endsWith("/")){
      outputPath = outputPath + "/";
    }

    //creating the learning algorithm object; discount= 0.99; initialQ=0.0; learning rate=0.9
    QComputablePlanner agent = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0.0, 0.9);

    //run learning
    double avg=0;
    int numReach=0;
    int mod=1000;
    for(int i = 1; i <= 25000; i++){
      EpisodeAnalysis ea = ((LearningAgent) agent).runLearningEpisodeFrom(heli.initialState(domain,height, length, gap, ahead));
      ea.writeToFile(String.format("%se%03d", outputPath, i), sp);
      avg+=ea.numTimeSteps();
      if (ea.numTimeSteps() == timelimit+1) numReach++;
      if (i%mod == 0)
      {
        System.out.println(i + ": " + avg/mod + ", " + numReach);
        numReach=0;
        avg = 0;
      }
    }

    Policy p = new GreedyQPolicy(agent);

    VisualActionObserver observer = new VisualActionObserver(domain, 
        heli.getVisualizer());
    observer.setFrameDelay(120);
    ((SADomain)this.domain).setActionObserverForAllAction(observer);
    observer.initGUI(); 
    
    int c = 1;
    while (true)
    {
      System.out.println("Test " + c);
      p.evaluateBehavior(heli.initialState(domain,height, length, gap, ahead), rf, tf);
      c++;
    }
    //    System.out.println(ea.getActionSequenceString("\n"));
    //    System.out.println(ea.numTimeSteps());
  }     

  public static void main(String[] args) {


    HelicopterTest example = new HelicopterTest();
    String outputPath = "helioutput/"; //directory to record results

    //we will call planning and learning algorithms here
    example.QLearningExample(outputPath);

    //    
    //    //run the visualizer
    //    example.visualize(outputPath);



  }

}
