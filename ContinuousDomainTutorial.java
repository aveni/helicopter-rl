import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.auxiliary.StateGridder;
import burlap.behavior.singleagent.learning.GoalBasedRF;
import burlap.behavior.singleagent.learning.lspi.LSPI;
import burlap.behavior.singleagent.learning.lspi.SARSCollector;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.learning.tdmethods.vfa.GradientDescentSarsaLam;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.sparsesampling.SparseSampling;
import burlap.behavior.singleagent.vfa.ValueFunctionApproximation;
import burlap.behavior.singleagent.vfa.cmac.CMACFeatureDatabase;
import burlap.behavior.singleagent.vfa.common.ConcatenatedObjectFeatureVectorGenerator;
import burlap.behavior.singleagent.vfa.fourier.FourierBasis;
import burlap.behavior.singleagent.vfa.rbf.DistanceMetric;
import burlap.behavior.singleagent.vfa.rbf.RBFFeatureDatabase;
import burlap.behavior.singleagent.vfa.rbf.functions.GaussianRBF;
import burlap.behavior.singleagent.vfa.rbf.metrics.EuclideanDistance;
import burlap.behavior.statehashing.NameDependentStateHashFactory;
import burlap.domain.singleagent.cartpole.InvertedPendulum;
import burlap.domain.singleagent.cartpole.InvertedPendulumStateParser;
import burlap.domain.singleagent.cartpole.InvertedPendulumVisualizer;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.lunarlander.*;
import burlap.domain.singleagent.mountaincar.MCRandomStateGenerator;
import burlap.domain.singleagent.mountaincar.MountainCar;
import burlap.domain.singleagent.mountaincar.MountainCarVisualizer;
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.VisualActionObserver;
import burlap.oomdp.visualizer.Visualizer;

import java.util.List;


public class ContinuousDomainTutorial {

  public static void main(String [] args){

    //uncomment the example you want to see and comment out the rest.

    //MCLSPIFB();
    MCLSPIRBF();
    //IPSS();
    //LLSARSA();
  }


  public static void MCLSPIFB(){

    MountainCar mcGen = new MountainCar();
    Domain domain = mcGen.generateDomain();
    TerminalFunction tf = mcGen.new ClassicMCTF();
    RewardFunction rf = new GoalBasedRF(tf, 100);

    StateGenerator rStateGen = new MCRandomStateGenerator(domain);
    SARSCollector collector = new SARSCollector.UniformRandomSARSCollector(domain);
    SARSData dataset = collector.collectNInstances(rStateGen, rf, 5000, 20, tf, null);

    ConcatenatedObjectFeatureVectorGenerator featureVectorGenerator = 
                 new ConcatenatedObjectFeatureVectorGenerator(true, MountainCar.CLASSAGENT);
    FourierBasis fb = new FourierBasis(featureVectorGenerator, 4);

    LSPI lspi = new LSPI(domain, rf, tf, 0.99, fb);
    lspi.setDataset(dataset);

    lspi.runPolicyIteration(30, 1e-6);

    GreedyQPolicy p = new GreedyQPolicy(lspi);

    Visualizer v = MountainCarVisualizer.getVisualizer(mcGen);
    VisualActionObserver vexp = new VisualActionObserver(domain, v);
    vexp.initGUI();
    ((SADomain)domain).addActionObserverForAllAction(vexp);
  
    State s = mcGen.getCleanState(domain);
    for(int i = 0; i < 5; i++){
      p.evaluateBehavior(s, rf, tf);
    }

    System.out.println("Finished.");

  }


  public static void MCLSPIRBF(){

    MountainCar mcGen = new MountainCar();
    Domain domain = mcGen.generateDomain();
    TerminalFunction tf = mcGen.new ClassicMCTF();
    RewardFunction rf = new GoalBasedRF(tf, 100);

    //get a state definition earlier, we'll use it soon.
    State s = mcGen.getCleanState(domain);

    StateGenerator rStateGen = new MCRandomStateGenerator(domain);
    SARSCollector collector = new SARSCollector.UniformRandomSARSCollector(domain);
    SARSData dataset = collector.collectNInstances(rStateGen, rf, 5000, 20, tf, null);

    //set up RBF feature database
    RBFFeatureDatabase rbf = new RBFFeatureDatabase(true);
    StateGridder gridder = new StateGridder();
    gridder.gridEntireDomainSpace(domain, 5);
    List<State> griddedStates = gridder.gridInputState(s);

    DistanceMetric metric = new EuclideanDistance(
                new ConcatenatedObjectFeatureVectorGenerator(true, MountainCar.CLASSAGENT));
    
    for(State g : griddedStates){
      rbf.addRBF(new GaussianRBF(g, metric, .2));
    }

    //notice we pass LSPI our RBF features this time
    LSPI lspi = new LSPI(domain, rf, tf, 0.99, rbf);
    lspi.setDataset(dataset);

    lspi.runPolicyIteration(30, 1e-6);

    GreedyQPolicy p = new GreedyQPolicy(lspi);

    Visualizer v = MountainCarVisualizer.getVisualizer(mcGen);
    VisualActionObserver vexp = new VisualActionObserver(domain, v);
    vexp.initGUI();
    ((SADomain)domain).addActionObserverForAllAction(vexp);


    for(int i = 0; i < 5; i++){
      p.evaluateBehavior(s, rf, tf);
    }

    System.out.println("Finished.");

  }



  public static void IPSS(){

    InvertedPendulum ip = new InvertedPendulum();
    ip.actionNoise = 0.;
    Domain domain = ip.generateDomain();
    RewardFunction rf = new InvertedPendulum.InvertedPendulumRewardFunction(Math.PI/8.);
    TerminalFunction tf = new InvertedPendulum.InvertedPendulumTerminalFunction(Math.PI/8.);
    State initialState = InvertedPendulum.getInitialState(domain);


    SparseSampling ss = new SparseSampling(domain, rf, tf, 1, 
                             new NameDependentStateHashFactory(), 10, 1);
    ss.setForgetPreviousPlanResults(true);
    Policy p = new GreedyQPolicy(ss);

    EpisodeAnalysis ea = p.evaluateBehavior(initialState, rf, tf, 500);
    StateParser sp = new InvertedPendulumStateParser(domain);

    ea.writeToFile("ip/ssPlan", sp);
    System.out.println("Num Steps: " + ea.numTimeSteps());

    Visualizer v = InvertedPendulumVisualizer.getInvertedPendulumVisualizer();
    new EpisodeSequenceVisualizer(v, domain, sp, "ip");


  }


  public static void LLSARSA(){

    LunarLanderDomain lld = new LunarLanderDomain();
    Domain domain = lld.generateDomain();
    RewardFunction rf = new LunarLanderRF(domain);
    TerminalFunction tf = new LunarLanderTF(domain);

    StateParser sp = new LLStateParser(domain);

    State s = LunarLanderDomain.getCleanState(domain, 0);
    LunarLanderDomain.setAgent(s, 0., 5.0, 0.0);
    LunarLanderDomain.setPad(s, 75., 95., 0., 10.);

    int nTilings = 5;
    CMACFeatureDatabase cmac = new CMACFeatureDatabase(nTilings, 
                              CMACFeatureDatabase.TilingArrangement.RANDOMJITTER);
    double resolution = 10.;

    double angleWidth = 2 * lld.getAngmax() / resolution;
    double xWidth = (lld.getXmax() - lld.getXmin()) / resolution;
    double yWidth = (lld.getYmax() - lld.getYmin()) / resolution;
    double velocityWidth = 2 * lld.getVmax() / resolution;

    cmac.addSpecificationForAllTilings(LunarLanderDomain.AGENTCLASS, 
                                           domain.getAttribute(LunarLanderDomain.AATTNAME), 
                                           angleWidth);
    cmac.addSpecificationForAllTilings(LunarLanderDomain.AGENTCLASS, 
                                           domain.getAttribute(LunarLanderDomain.XATTNAME), 
                                           xWidth);
    cmac.addSpecificationForAllTilings(LunarLanderDomain.AGENTCLASS, 
                                           domain.getAttribute(LunarLanderDomain.YATTNAME), 
                                           yWidth);
    cmac.addSpecificationForAllTilings(LunarLanderDomain.AGENTCLASS, 
                                           domain.getAttribute(LunarLanderDomain.VXATTNAME), 
                                           velocityWidth);
    cmac.addSpecificationForAllTilings(LunarLanderDomain.AGENTCLASS, 
                                           domain.getAttribute(LunarLanderDomain.VYATTNAME), 
                                           velocityWidth);

    double defaultQ = 0.5;
    ValueFunctionApproximation vfa = cmac.generateVFA(defaultQ/nTilings);
    GradientDescentSarsaLam agent = new GradientDescentSarsaLam(domain, rf, tf, 0.99, 
                                                                vfa, 0.02, 10000, 0.5);

    for(int i = 0; i < 5000; i++){
      EpisodeAnalysis ea = agent.runLearningEpisodeFrom(s); //run learning episode
      ea.writeToFile(String.format("lunarLander/e%04d", i), sp); //record episode to a file
      System.out.println(i + ": " + ea.numTimeSteps()); //print the performance 
    }


    Visualizer v = LLVisualizer.getVisualizer(lld);
    new EpisodeSequenceVisualizer(v, domain, sp, "lunarLander");

  }




}