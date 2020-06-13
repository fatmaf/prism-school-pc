package demos;
//takes a team mdp 

import java.util.AbstractMap;

//creates a joint mdp from the team mdp solution 

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;

import explicit.MDPSimple;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismException;
import prism.PrismLangException;
import prism.PrismLog;
import strat.MDStrategyArray;

public class RewriteJointPolicyBuilder
{

	private int numRobots;
	private int numTasks;
	private int numSharedStates;
	private ArrayList<String> sharedStatesNamesList;
	MDPCreator jointMDP;
	BitSet accStates; 
	private ArrayList<MDPRewardsSimple> seqTeamMDPRewards;
	PriorityQueue<StateExtended> failureStates;
	List<State> allExploredStates;
	HashMap<Entry<Integer, Integer>, Double> progressionRewardsHashMap = null;
	HashMap<Entry<Integer, Integer>, ArrayList<Double>> otherRewardsHashMap = null;

	class HelperClass<T>
	{
		public int getNumCombsFromSizes(ArrayList<Integer> numElements)
		{
			int numcombs = 1;
			for (int r = 0; r < numElements.size(); r++)
				numcombs *= numElements.get(r);
			return numcombs;
		}

		public int getNumCombs(ArrayList<ArrayList<T>> robotStates)
		{

			int numcombs = 1;

			for (int r = 0; r < robotStates.size(); r++) {

				numcombs *= robotStates.get(r).size();
			}
			return numcombs;
		}

		public ArrayList<ArrayList<T>> generateCombinations(ArrayList<ArrayList<T>> robotStates)
		{
			// so lets get the number of states for each robot
			int[] numStates = new int[robotStates.size()];
			int[] currStateNum = new int[robotStates.size()];
			int numcombs = 1;
			int lastrobotnum = robotStates.size() - 1;
			for (int r = 0; r < robotStates.size(); r++) {
				List<T> rs = robotStates.get(r);
				numStates[r] = rs.size();

				currStateNum[r] = 0;
				numcombs *= rs.size();
			}
			ArrayList<ArrayList<T>> combs = new ArrayList<>();
			boolean docomb = true;
			while (docomb) {
				// so now we just loop over things
				// its a lot of while loops
				ArrayList<T> currcomb = new ArrayList<>();
				for (int r = 0; r < robotStates.size(); r++) {
					T rs = robotStates.get(r).get(currStateNum[r]);
					currcomb.add(rs);
				}
				combs.add(currcomb);

				boolean doInc = true;
				for (int lr = lastrobotnum; lr >= 0; lr--) {
					if (currStateNum[lr] + 1 == numStates[lr]) {
						currStateNum[lr] = 0;
					} else {
						currStateNum[lr]++;
						doInc = false;
					}
					if (!doInc) {
						break;
					}
				}
				int indsum = 0;
				for (int r = 0; r < numStates.length; r++) {
					indsum += currStateNum[r];
				}
				if (indsum == 0)
					docomb = false;
			}

			return combs;
		}

	}

	class TaskAllocation
	{
		ArrayList<Entry<HashMap<Integer, Integer>, HashMap<Integer, Integer>>> changedValues;
		List<Integer> path;
		List<Integer> terminalStates;
		List<String> actions;
		//a class that stores a task allocation for a robot 
		//a task allocation consists of 
	}

	class TeamMDPToJointMDPMap
	{
		HashMap<Integer, Integer> daMap;
		HashMap<Integer, Integer> ssMap;
		HashMap<Integer, ArrayList<Integer>> rMap;
		int[] daInds;
		int[] ssInds;
		int rnumInd;
		ArrayList<Integer> rInds;
		int teamMDPNumVars;

		public TeamMDPToJointMDPMap(int numda, int numss, int numRobots, int teamMDPNumVars)
		{
			daMap = new HashMap<>();
			ssMap = new HashMap<>();
			rMap = new HashMap<>();
			daInds = new int[numda];
			ssInds = new int[numss];
			rInds = new ArrayList<>();
			this.teamMDPNumVars = teamMDPNumVars;

		}

		public void setDAInd(int danum, int ind)
		{
			daInds[danum] = ind;
		}

		public void setSSInd(int ssnum, int ind)
		{
			ssInds[ssnum] = ind;
		}

		public void addr(int ti, int ji)
		{
			if (!rMap.containsKey(ti))
				rMap.put(ti, new ArrayList<Integer>());
			rMap.get(ti).add(ji);

		}

		public int getR(int ti, int rnum)
		{
			return rMap.get(ti).get(rnum);
		}

		public int getRobotNumFromTeamMDPState(State state)
		{
			//always at 1 
			return (int) state.varValues[rnumInd];
		}

		//things we need are 
		//dastates 
		//and ssstates 
		public int getDAState(int danum, State state)
		{
			int daInd = this.daInds[danum];
			return (int) state.varValues[daInd];
		}

		public int getSSState(int ssnum, State state)
		{
			int sInd = this.ssInds[ssnum];
			return (int) state.varValues[sInd];
		}

	}

	class JointMDPToTeamMap
	{
		HashMap<Integer, Integer> daMap;
		HashMap<Integer, Integer> ssMap;
		HashMap<Integer, Integer> rMap;
		int[] daInds;
		int[] ssInds;
		ArrayList<ArrayList<Integer>> rInds;
		int rnumInd;

		//dainitalstates 
		//dafinalstate 
		ArrayList<Integer> daInitialStates;
		ArrayList<BitSet> daFinalStates;
		int safetyDAInd;

		public JointMDPToTeamMap(int numda, int numss, int numRobots)
		{
			daMap = new HashMap<>();
			ssMap = new HashMap<>();
			rMap = new HashMap<>();
			daInds = new int[numda];
			ssInds = new int[numss];
			rInds = new ArrayList<>();
			daInitialStates = new ArrayList<>();
			daFinalStates = new ArrayList<>();
		}

		public void setDAInd(int danum, int ind)
		{
			daInds[danum] = ind;
		}

		public void setSSInd(int ssnum, int ind)
		{
			ssInds[ssnum] = ind;
		}

		public void setRind(int rnum, int ind)
		{
			while (rInds.size() <= rnum)
				rInds.add(new ArrayList<>());
			//			rInds[rnum] = ind;
			rInds.get(rnum).add(ind);
		}

		public int getDAState(int danum, State state)
		{
			int daInd = this.daInds[danum];
			return (int) state.varValues[daInd];
		}

		public int getSSState(int ssnum, State state)
		{
			int sInd = this.ssInds[ssnum];
			return (int) state.varValues[sInd];
		}

		public boolean isDAaccState(int danum, State state)
		{
			int daInd = daInds[danum];
			int daVal = (int) state.varValues[daInd];
			boolean isacc = false;
			if (daFinalStates.get(danum).get(daVal))
				isacc = true;
			return isacc;
		}

		public void addr(int ti, int ji)
		{
			//			if (!rMap.containsKey(ti))
			//				rMap.put(ti, new ArrayList<Integer>());
			//			rMap.get(ti).add(ji);
			rMap.put(ti, ji);

		}

		public int getNumVars()
		{
			return this.ssInds.length + this.daInds.length + rMap.keySet().size();
		}

	}

	TeamMDPToJointMDPMap teammdptojointmdpmap;
	JointMDPToTeamMap jointmdptoteammdpmap;
	private MDPRewardsSimple progressionRewards;
	private ArrayList<MDPRewardsSimple> otherRewards;

	public void setInitialState(State s)
	{
		jointMDP.setInitialState(s);
	}
	public RewriteJointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, VarList seqTeamMDPVarList,
			ArrayList<MDPRewardsSimple> rewards, ArrayList<DAInfo> daList, PrismLog log) throws PrismException
	{
		//the components are 
		//da states 
		//robot states 
		//shared states 
		ArrayList<String> isolatedStatesList = new ArrayList<String>();
		for (int i = 0; i < seqTeamMDPVarList.getNumVars(); i++) {
			String name = seqTeamMDPVarList.getName(i);
			if ((!sharedStatesList.contains(name)) && (!name.contains("da")) && (name != "r")) {
				isolatedStatesList.add(name);
			}
		}
		//		statesMap = new HashMap<State, Integer>();
		numRobots = nrobots;
		numTasks = ntasks;
		numSharedStates = sharedStatesList.size();
		sharedStatesNamesList = sharedStatesList;
		jointMDP = new MDPCreator();
		jointMDP.setVarList(createVarList(seqTeamMDPVarList, daList));
		//		jointMDP.setStatesList(new ArrayList<State>());
		//		this.failedStatesQueue = new PriorityQueue<StateExtended>();
		//		this.statesExploredOrder = new ArrayList<Entry<State, Double>>();

		progressionRewardsHashMap = new HashMap<Entry<Integer, Integer>, Double>();
		otherRewardsHashMap = new HashMap<Entry<Integer, Integer>, ArrayList<Double>>();
		failureStates = new PriorityQueue<StateExtended>();
		allExploredStates = new ArrayList<>();
		if (rewards.size() > 1) {
			if (this.seqTeamMDPRewards == null)
				seqTeamMDPRewards = new ArrayList<MDPRewardsSimple>();
			// so the assumption that we dont care about progression rewards
			for (int i = 1; i < rewards.size(); i++) {
				seqTeamMDPRewards.add(rewards.get(i));
			}

		}
		accStates = new BitSet();

	}

	//things we need 
	//an mdp strategy 
	//
	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, SequentialTeamMDP seqTeamMDP, State jointState,
			boolean reallocateOnSingleAgentDeadend, double initStateProb, int firstRobot) throws PrismException
	{
		if (!allExploredStates.contains(jointState)) {
			BitSet teamMDPAcc = seqTeamMDP.acceptingStates;
			boolean doSeq = true;
			MDPSimple mdp = seqTeamMDP.teamMDPWithSwitches;
			List<State> statesList = seqTeamMDP.teamMDPWithSwitches.getStatesList();
			State initialState = jointState;
			Queue<Entry<State, Double>> q = new LinkedList<Entry<State, Double>>();
			//		Queue<State> ps = new LinkedList<State>();
			ArrayList<State> explored = new ArrayList<State>();
			q.add(new AbstractMap.SimpleEntry<State, Double>(jointState, initStateProb));
			//		ps.add(null);
			//the steps 
			//create queue 
			//while q is not empty 
			TaskAllocation prevTa = null;
			Queue<TaskAllocation> taQ = new LinkedList<TaskAllocation>();
			taQ.add(prevTa);
			allExploredStates.add(initialState);
			while (!q.isEmpty()) {
				//s = q.pop 
				Entry<State, Double> stateProbComb = q.remove();
				State currentState = stateProbComb.getKey();
				double currentStateProb = stateProbComb.getValue();
				prevTa = taQ.remove();
				if (explored.contains(currentState)) {
					continue;
				}
				explored.add(currentState);
				//			if (!isTerminal(currentState)) {
				//if s is not terminal 
				//rs=get robot states from joint state 
				int[] rs = this.getRobotStatesFromJointState(currentState, statesList);
				//ta=get task allocation (rs) 
				TaskAllocation ta = getTaskAllocation(prevTa, rs, strat, mdp, firstRobot, doSeq, teamMDPAcc);

				//updatedrs = applyta(rs)
				int[] updatedRobotStates = applyTaskAllocation(ta, rs, statesList, firstRobot, doSeq);
				//ras = getActions(strat,updatedrs) 
				Object[] actions = getActions(strat, updatedRobotStates);
				//succs = getSuccessors(seqteammdp,ras,rs) 
				//so the actions correspond to the updatedRobotStates 
				//we've got to remember to match them 
				//we apply these actions on the old robot states 
				ArrayList<Double> rews = new ArrayList<>();
				ArrayList<ArrayList<Entry<Integer, Double>>> robotsSuccessors = executeActions(mdp, rs, actions, rews);
				//now generate the combinations 
				HelperClass<Entry<Integer, Double>> hc = new HelperClass<>();
				ArrayList<ArrayList<Entry<Integer, Double>>> combs = hc.generateCombinations(robotsSuccessors);
				//for each combination we've got to create the successors 
				//to create the successors we need the parentstate and the initialstate

				ArrayList<Entry<State, Double>> successors = createSuccessorStatesFromCombs(combs, currentState, initialState, statesList);

				if (isTerminal(currentState, successors, currentStateProb))
					continue;
				String jointAction = createJointAction(actions, rs, statesList);
				int jointMDPActionIndex = jointMDP.addAction(currentState, jointAction, successors);
				int jointMDPStateIndex = jointMDP.getStateIndex(currentState);

				double expTaskRew = 0;
				for (Entry<State, Double> succ : successors) {
					int numtasks = numTasksCompleted( succ.getKey(),currentState);
					expTaskRew += (double) numtasks * succ.getValue();
					succ.setValue(succ.getValue() * currentStateProb);
					q.add(succ);
					taQ.add(ta);
					//					ps.add(currentState);
				}
				//set the rewards 
				addTaskCompletionRew(jointMDPStateIndex, jointMDPActionIndex, expTaskRew);
				addStateActionRewards(jointMDPStateIndex, jointMDPActionIndex, rews);
				//			System.out.println("meh");
				//for each succ comb
				//create successor state 
				//add to q 
				//			}
			}
//			jointMDP.saveMDP("../prism/tests/tro_examples/results/logs/debug/", "tempPol.dot");
		}

	}

	void addTaskCompletionRew(int s, int a, double rew)
	{
		Entry<Integer, Integer> saPair = new AbstractMap.SimpleEntry<Integer, Integer>(s, a);
		progressionRewardsHashMap.put(saPair, rew);
	}

	void addStateActionRewards(int s, int actionIndex, ArrayList<Double> rews)
	{
		Entry<Integer, Integer> saPair = new AbstractMap.SimpleEntry<Integer, Integer>(s, actionIndex);
		//		if(!otherRewardsHashMap.containsKey(saPair))
		otherRewardsHashMap.put(saPair, rews);
	}

	boolean isAccepting(State currentState) throws PrismException
	{
		boolean isacc = true;
		for (int i = 0; i < numTasks; i++) {
			isacc = isacc & jointmdptoteammdpmap.isDAaccState(i, currentState);
		}
		if(isacc)
		{
			//this state is in the joint mdp 
			int sIndex = jointMDP.getStateIndex(currentState); 
			if(sIndex>-1)
			{
				this.accStates.set(sIndex);
			}
			else
			{
				throw new PrismException("This state does not exist");
			}
		}
		return isacc;
	}

	int numTasksCompleted(State currentState, State previousState)
	{
		int sumTasks = 0;
		//		jointmdptoteammdpmap.safetyDAInd
		for (int i = 0; i < numTasks; i++) {
			if (i != this.jointmdptoteammdpmap.safetyDAInd) {
				int ps = jointmdptoteammdpmap.getDAState(i, previousState);
				int cs = jointmdptoteammdpmap.getDAState(i, currentState);
				if (ps != cs) {
					if (jointmdptoteammdpmap.isDAaccState(i, currentState))
						sumTasks++;
				}
			}

		}
		return sumTasks;
	}

	boolean isTerminal(State currentState, ArrayList<Entry<State, Double>> successors, double currentStateProb) throws PrismException
	{
		boolean isTerminal = false;
		boolean isAcc = isAccepting(currentState);
		isTerminal = isAcc;
		if (!isAcc) {
			if (successors.size() < 2) {
				//0 or 1 successors , possibly a deadend 
				if (successors.size() == 1) {
					State succ = successors.get(0).getKey();
					if (succ.compareTo(currentState) == 0) {
						isTerminal = true;
					}
				} else
					isTerminal = true;
			}
			if (isTerminal) {
				if (!allExploredStates.contains(currentState)) {
					StateExtended se = new StateExtended(currentState, currentStateProb);
					if (!failureStates.contains(se))
						failureStates.add(se);
				}
			}
		}
		return isTerminal;
	}

	String createJointAction(Object[] actions, int[] rs, List<State> statesList)
	{
		String[] actionS = new String[rs.length];

		String jointAction = "";
		for (int i = 0; i < rs.length; i++) {
			int s = rs[i];
			State state = statesList.get(s);
			int rnum = this.teammdptojointmdpmap.getRobotNumFromTeamMDPState(state);
			actionS[rnum] = "r" + rnum + "_";
			if (actions[i] != null) {
				actionS[rnum] += actions[i].toString();
			}
		}
		for (int i = 0; i < actionS.length; i++) {

			jointAction += actionS[i];
			if (i < actionS.length - 1) {
				jointAction += "_";
			}

		}
		return jointAction;
	}

	ArrayList<Entry<State, Double>> createSuccessorStatesFromCombs(ArrayList<ArrayList<Entry<Integer, Double>>> combs, State parentState, State initialState,
			List<State> statesList)
	{
		ArrayList<Entry<State, Double>> successors = new ArrayList<>();
		for (int comb = 0; comb < combs.size(); comb++) {
			ArrayList<Entry<Integer, Double>> currentComb = combs.get(comb);
			double cumProb = 1.0;
			ArrayList<State> robotStates = new ArrayList<>();
			for (int snum = 0; snum < currentComb.size(); snum++) {
				Entry<Integer, Double> rComb = currentComb.get(snum);
				State state = statesList.get(rComb.getKey());
				robotStates.add(state);
				cumProb *= rComb.getValue();
			}

			State succState = createJointStateFromRobotStates(robotStates, parentState, initialState, false);
			successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, cumProb));

		}
		return successors;

	}

	ArrayList<ArrayList<Entry<Integer, Double>>> executeActions(MDPSimple mdp, int[] rs, Object[] actions, ArrayList<Double> rewards)
	{

		ArrayList<ArrayList<Entry<Integer, Double>>> robotsSuccessors = new ArrayList<>();
		for (int r = 0; r < this.seqTeamMDPRewards.size(); r++)
			rewards.add(0.0);
		//		rew = seqTeamMDPRewards.get(i).getTransitionReward(state, choice);
		for (int i = 0; i < rs.length; i++) {
			int s = rs[i];
			Object a = actions[i];
			ArrayList<Entry<Integer, Double>> robotSuccessors = new ArrayList<>();
			//					boolean repeatState = true;
			if (a != null) {
				if (a.toString() != "*" || a.toString() != "?" || !a.toString().contains("switch")) {
					int c = findActionChoiceIndex(mdp, s, a);
					if (c > -1) {
						for (int r = 0; r < this.seqTeamMDPRewards.size(); r++) {
							double rewhere = seqTeamMDPRewards.get(r).getTransitionReward(s, c);
							rewards.set(r, rewhere + rewards.get(r));
						}
						//						repeatState = false; 
						Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, c);
						while (tranIter.hasNext()) {
							robotSuccessors.add(tranIter.next());
						}
					}
				}
			}
			if (robotSuccessors.size() == 0) {

				robotSuccessors.add(new AbstractMap.SimpleEntry<Integer, Double>(s, 1.0));
			}
			robotsSuccessors.add(robotSuccessors);
		}
		return robotsSuccessors;
	}

	int findActionChoiceIndex(MDPSimple mdp, int s, Object a)
	{
		int choice = -1;
		int numChoices = mdp.getNumChoices(s);
		for (int c = 0; c < numChoices; c++) {
			Object choiceAct = mdp.getAction(s, c);
			if (a.toString().contentEquals(choiceAct.toString())) {
				choice = c;
				break;
			}
		}
		return choice;
	}

	Object[] getActions(MDStrategyArray strat, int[] rs)
	{
		Object[] toret = new Object[rs.length];
		for (int i = 0; i < rs.length; i++) {
			int s = rs[i];
			toret[i] = strat.getChoiceAction(s);
		}
		return toret;

	}

	State undoTaskAllocation(TaskAllocation ta, State s, int currRobot, int firstRobot, boolean doSeq)
	{
		//so if seq we go all over the robots 
		//start from current robot's next robot and go all the way to the end
		int startRobot = (currRobot + 1) % numRobots;
		State newState = new State(s);
		if (doSeq) {
			while (startRobot != firstRobot) {

				//apply start robot's ta values to this state 
				HashMap<Integer, Integer> changedStates = ta.changedValues.get(startRobot).getKey();
				for (int sI : changedStates.keySet()) {
					newState.setValue(sI, changedStates.get(sI));
				}
				startRobot = (startRobot + 1) % numRobots;

			}
		} else {
			for (int i = 0; i < numRobots; i++) {
				if (i != currRobot) {
					HashMap<Integer, Integer> changedStates = ta.changedValues.get(i).getKey();
					for (int sI : changedStates.keySet()) {
						newState.setValue(sI, changedStates.get(sI));
					}
				}
			}
		}
		return newState;
	}

	State applyTaskAllocation(TaskAllocation ta, State s, int currRobot, int firstRobot, boolean doSeq)
	{
		//so if seq we go all over the robots 
		//start from first robot all the way to currRobot 
		int startRobot = firstRobot;
		State newState = new State(s);
		if (doSeq) {
			while (startRobot != currRobot) {

				//apply start robot's ta values to this state 
				HashMap<Integer, Integer> changedStates = ta.changedValues.get(startRobot).getValue();
				for (int sI : changedStates.keySet()) {
					newState.setValue(sI, changedStates.get(sI));
				}
				startRobot = (startRobot + 1) % numRobots;
				if (startRobot == firstRobot)
					break;

			}
		} else {
			for (int i = 0; i < numRobots; i++) {
				if (i != currRobot) {
					HashMap<Integer, Integer> changedStates = ta.changedValues.get(i).getValue();
					for (int sI : changedStates.keySet()) {
						newState.setValue(sI, changedStates.get(sI));
					}
				}
			}
		}
		return newState;
	}

	int[] applyTaskAllocation(TaskAllocation ta, int[] rs, List<State> statesList, int firstRobot, boolean doSeq)
	{
		int[] rStates = new int[rs.length];

		for (int r = 0; r < numRobots; r++) {
			State currentRobotState = statesList.get(rs[r]);

			State updatedState = applyTaskAllocation(ta, currentRobotState, r, firstRobot, doSeq);
			int us = findStateInStatesList(updatedState, statesList);
			rStates[r] = us;

		}
		return rStates;
	}

	int[] undoTaskAllocation(TaskAllocation ta, int[] rs, List<State> statesList, int firstRobot, boolean doSeq)
	{
		int[] rStates = new int[rs.length];

		for (int r = 0; r < numRobots; r++) {
			State currentRobotState = statesList.get(rs[r]);

			State updatedState = undoTaskAllocation(ta, currentRobotState, r, firstRobot, doSeq);
			int us = findStateInStatesList(updatedState, statesList);
			rStates[r] = us;

		}
		return rStates;
	}

	SimpleEntry<SimpleEntry<List<Integer>, ArrayList<Integer>>, Boolean> getMostProbablePath(int initialState, MDStrategyArray strat, MDPSimple mdp,
			boolean stopAtRobotsTerminal, List<String> actionList, int[] statesToIncludeInPath, BitSet mdpAccStates)
	{
		boolean acceptingStateFound = false;
		ArrayList<Integer> terminalStates = new ArrayList<>();
		int initialRobotNum = 0;
		if (stopAtRobotsTerminal)
			initialRobotNum = this.teammdptojointmdpmap.getRobotNumFromTeamMDPState(mdp.getStatesList().get(initialState));

		PriorityQueue<StateExtended> statesQ = new PriorityQueue<>();
		StateExtended s = new StateExtended(initialState, 1.0);
		statesQ.add(s);
		List<Integer> probablePath = new ArrayList<Integer>();
		while (!statesQ.isEmpty()) {
			StateExtended se = statesQ.remove();
			int snum = se.childState;
			double sprob = se.parentToChildTransitionProbability;
			if (probablePath.contains(snum))
				continue;
			if (mdpAccStates.get(snum)) {
				acceptingStateFound = false;
			}
			probablePath.add(snum);
			//returns a single path 
			//			strat.initialise(snum);
			int choiceIndex = strat.getChoiceIndex(snum);
			Object action = strat.getChoiceAction(snum);
			if (action != null) {
//				System.out.println(action.toString());
				if (actionList != null)
					actionList.add(action.toString());

			} else {
				if (actionList != null)
					actionList.add("");
			}
			if (action != null) {
				//current robot num 
				int currentRobotNum = teammdptojointmdpmap.getRobotNumFromTeamMDPState(mdp.getStatesList().get(snum));
				if (action.toString().contains("switch") || action.toString().contains("*") || action.toString().contains("?")) {
					while (terminalStates.size() <= currentRobotNum) {
						terminalStates.add(null);
					}
					terminalStates.set(currentRobotNum, snum);
				}
			}
			if (choiceIndex > -1) {
				se.actionInChildState = action.toString();
				se.choice = choiceIndex;
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(snum, choiceIndex);
				StateExtended bestChild = null;
				boolean doBestChild = true;
				while (tranIter.hasNext()) {
					Entry<Integer, Double> sp = tranIter.next();
					StateExtended child = new StateExtended(sp.getKey(), sp.getValue() * sprob);

					if (statesToIncludeInPath != null) {
						for (int stateToInclude : statesToIncludeInPath) {
							if (sp.getKey() == stateToInclude) {
								doBestChild = false;
								break;
							}
						}
					}
					if (doBestChild) {
						if (bestChild == null)
							bestChild = child;
						else {
							if (child.parentToChildTransitionProbability > bestChild.parentToChildTransitionProbability) {
								bestChild = child;
							}
						}
					} else {
						bestChild = child;
						break;
					}
				}

				if (bestChild != null) {
					if (stopAtRobotsTerminal) {
						int bestChildRNum = teammdptojointmdpmap.getRobotNumFromTeamMDPState(mdp.getStatesList().get(bestChild.childState));

						if (initialRobotNum != bestChildRNum) {
							bestChild = null;
						}
					}
					if (bestChild != null)
						statesQ.add(bestChild);

				}

			}
		}
		AbstractMap.SimpleEntry<List<Integer>, ArrayList<Integer>> toret = new AbstractMap.SimpleEntry<List<Integer>, ArrayList<Integer>>(probablePath,
				terminalStates);
		return new AbstractMap.SimpleEntry<AbstractMap.SimpleEntry<List<Integer>, ArrayList<Integer>>, Boolean>(toret, acceptingStateFound);

	}

	private TaskAllocation getNewTaskAllocation(int startState, MDStrategyArray strat, MDPSimple mdp, int firstRobot, int[] statesToInclude, int[] backupStates,
			BitSet mdpAcc)
	{

		List<String> actionList = new ArrayList<String>();
		//	int startState = rs[firstRobot];
		Entry<SimpleEntry<List<Integer>, ArrayList<Integer>>, Boolean> mostProbablePathAndTerminalStatesAccState = getMostProbablePath(startState, strat, mdp,
				false, actionList, statesToInclude, mdpAcc);
		boolean accStateReached = mostProbablePathAndTerminalStatesAccState.getValue();
		SimpleEntry<List<Integer>, ArrayList<Integer>> mostProbablePathAndTerminalStates = mostProbablePathAndTerminalStatesAccState.getKey();
		//we've got a path and what else do we need 
		//we actually need the terminal states for each robot on this path 
		List<Integer> mostProbablePath = mostProbablePathAndTerminalStates.getKey();
		ArrayList<Integer> terminalStates = mostProbablePathAndTerminalStates.getValue();
		//if the accstate hasnt been reached
		//lets see if we can keep getting stuff 
		if (mostProbablePath.size() == 1 & !accStateReached) {
			//the first robot failed or did not have any actions 
			//perhaps other robots still have actions //it would be good to get those if we can 
			//so what is the probable path for other robots 
			int nextRobot = (firstRobot + 1) % numRobots;
			while (nextRobot != firstRobot) {
				Entry<SimpleEntry<List<Integer>, ArrayList<Integer>>, Boolean> tempMostProbablePathAndTerminalStatesAccState = getMostProbablePath(
						backupStates[nextRobot], strat, mdp, false, actionList, statesToInclude, mdpAcc);
				boolean tempAccStateReached = tempMostProbablePathAndTerminalStatesAccState.getValue();
				SimpleEntry<List<Integer>, ArrayList<Integer>> tempMostProbablePathAndTerminalStates = tempMostProbablePathAndTerminalStatesAccState.getKey();
				List<Integer> tempMPP = tempMostProbablePathAndTerminalStates.getKey();
				ArrayList<Integer> temTS = tempMostProbablePathAndTerminalStates.getValue();
				mostProbablePath.addAll(tempMPP);
				//only add terminal states for the bits not null 
				for (int i = 0; i < temTS.size(); i++) {
					if (terminalStates.size() > i) {
						if (terminalStates.get(i) == null) {
							terminalStates.set(i, temTS.get(i));
						}
					} else {
						terminalStates.add(temTS.get(i));
					}
				}
				//				terminalStates.addAll(temTS);
				//				if (tempMPP.size() > 1)
				//					break;
				if (tempAccStateReached)
					break;

				nextRobot = (nextRobot + 1) % numRobots;
			}
		}
		//so we have the terminalStates now 
		//now we've got to get the updates to make honestly 
		//this is confusing 

//		for (int i : terminalStates) {
//			System.out.println(mdp.getStatesList().get(i));
//		}
//		System.out.println("terms");
//		for (int i : mostProbablePath) {
//			System.out.println(mdp.getStatesList().get(i));
//		}
//		System.out.println("path");
		ArrayList<Entry<HashMap<Integer, Integer>, HashMap<Integer, Integer>>> allChangedValues = new ArrayList<>();
		int currRobot = firstRobot;
		int nextRobot = firstRobot;
		int firstState;
		State ns;
		do {

			if (currRobot == firstRobot) {
				firstState = startState;
			} else {
				firstState = terminalStates.get(currRobot);
			}
			State fs = mdp.getStatesList().get(firstState);
			if (terminalStates.size() <= nextRobot) {

				ns = null;
				terminalStates.add(backupStates[nextRobot]);
				mostProbablePath.add(backupStates[nextRobot]);
			} else {
				ns = mdp.getStatesList().get(terminalStates.get(nextRobot));
			}
			Entry<HashMap<Integer, Integer>, HashMap<Integer, Integer>> changedValues = getChangedStates(fs, ns, true, true);

			while (allChangedValues.size() <= nextRobot) {
				allChangedValues.add(null);
			}
			allChangedValues.set(nextRobot, changedValues);
			currRobot = nextRobot;
			nextRobot = (currRobot + 1) % numRobots;
		} while (nextRobot != firstRobot);
//		System.out.println(allChangedValues.toString());
		TaskAllocation newta = new TaskAllocation();
		newta.changedValues = allChangedValues;
		newta.path = mostProbablePath;
		newta.terminalStates = terminalStates;
		newta.actions = actionList;
		TaskAllocation tatoret = newta;
		//what do I need to apply a task allocation 
		//so this task allocation has this allChangedValues 
		//and it has the path 

		//i'll have a robot state which will be a team mdp state 
		//the task allocation is also a team mdp 
		//so for each robot we get a set of states that were changed 
		//from the previous robot's states 
		//we also save these states 

		//			else
		//			{
		//				throw new PrismException("Not implemented");
		//				//the idea again we should get the probable task and terminal state for each robot in turn 
		//				//before we have to apply the previous robots task allocation to it 
		//				//which is technically the same 
		//				//so its not different at all 
		//				
		//			}

		return tatoret;
	}

	private TaskAllocation getTaskAllocation(TaskAllocation prevTa, int[] rs, MDStrategyArray strat, MDPSimple mdp, int firstRobot, boolean doSeq,
			BitSet mdpAcc) throws PrismException
	{
		TaskAllocation tatoret = null;
		// TODO Auto-generated method stub
		boolean newTA = false;
		//		boolean partialTA = false;
		int[] statesToInclude = null;
		int startState = -1;
		int[] backupStates = rs;
		if (prevTa == null) {
			newTA = true;
			statesToInclude = null;
			startState = rs[firstRobot];
		} else {
			//check if prevTA still applies
			//apply prevTA to the robots 
			//undo any tasks that have been done by other robots since 
			int[] updatedRobotStates = undoTaskAllocation(prevTa, rs, mdp.getStatesList(), firstRobot, doSeq);
			updatedRobotStates = applyTaskAllocation(prevTa, updatedRobotStates, mdp.getStatesList(), firstRobot, doSeq);
			//			int startState = -1;
			backupStates = updatedRobotStates;
			int robotNotOnPath = -1;
			//then go on 
			//now check if all of the current updated states are on the path 
			int r = firstRobot;

			int lastStateOnPath = -1;
			int indexOfLastStateOnPath = -1;
			boolean robotOnPath = false;
			for (int i = 0; i < prevTa.path.size(); i++) {
				int pathstate = prevTa.path.get(i);
				if (pathstate == updatedRobotStates[r]) {
					robotOnPath = true;
					lastStateOnPath = prevTa.path.get(i);
					indexOfLastStateOnPath = i;
					//go ahead on to the next robot 
					r = (r + 1) % numRobots;
					if (r == firstRobot)
						break;
					robotOnPath = false;

				}

			}
			if (!robotOnPath) {
				robotNotOnPath = r;

				if (robotNotOnPath != firstRobot) {
					//well then we're somewhere in the middle 
					//so the last robot on the path was 
					int lastRobotOnPath = (robotNotOnPath - 1) % numRobots;
					//and we have the last state on the path and the index of that state 
					//now we need to get the most probable path which includes as many of the updated states of our other robots as possible

					//					partialTA = true; 
					//so we get a new ta from the last state 
					//and then we merge the two tas 
					//easier just to make sure all the updated states are on this path and get a new ta 
					//more expensive but easier 
					//					int startState = updatedRobotStates[firstRobot];
					//					tatoret = getNewTaskAllocation(updatedRobotStates[firstRobot], strat, mdp, firstRobot, updatedRobotStates);
					startState = updatedRobotStates[firstRobot];
					statesToInclude = updatedRobotStates;
					newTA = true;

				} else {
					startState = updatedRobotStates[firstRobot];
					newTA = true;
				}
			} else {
				tatoret = prevTa;
			}

		}
		if (newTA) {

			//			int startState = rs[firstRobot];
			tatoret = getNewTaskAllocation(startState, strat, mdp, firstRobot, statesToInclude, backupStates, mdpAcc);

		}
		return tatoret;
	}

	Entry<HashMap<Integer, Integer>, HashMap<Integer, Integer>> getChangedStates(State oldState, State newState, boolean isTeam, boolean justDA)
	{
		HashMap<Integer, Integer> prevValues = new HashMap<>();
		HashMap<Integer, Integer> newValues = new HashMap<>();
		if (oldState != null && newState != null) {
			for (int i = 0; i < oldState.varValues.length; i++) {
				if (justDA) {
					if (isTeam) {
						if (!teammdptojointmdpmap.daMap.containsKey(i)) {
							continue;
						}
					} else {
						if (!this.jointmdptoteammdpmap.daMap.containsKey(i)) {
							continue;
						}
					}
				}
				int prevVal = (int) oldState.varValues[i];
				int newVal = (int) newState.varValues[i];
				if (prevVal != newVal) {
					prevValues.put(i, prevVal);
					newValues.put(i, newVal);
				}
			}
		}
		return new AbstractMap.SimpleEntry<HashMap<Integer, Integer>, HashMap<Integer, Integer>>(prevValues, newValues);
	}

	public int[] getRobotStatesFromJointState(State currentJointState, List<State> sl)
	{
		ArrayList<State> rss = getRobotStateStatesFromJointState(currentJointState);
		int[] toret = new int[rss.size()];
		for (int i = 0; i < toret.length; i++) {
			State s = rss.get(i);
			toret[i] = findStateInStatesList(s, sl);
		}
		return toret;
	}

	public ArrayList<State> getRobotStateStatesFromJointState(State currentJointState)
	{
		ArrayList<State> statesToRet = new ArrayList<>();
		//		for(int i = 0; i<this.numRobots; i++)
		State statesofar = new State(this.teammdptojointmdpmap.teamMDPNumVars);
		//this should be a simple mapping so nothing fancy needs to happen here 
		for (int i = 0; i < this.numTasks; i++) {
			int jsInd = jointmdptoteammdpmap.daInds[i];
			int jsVal = jointmdptoteammdpmap.getDAState(i, currentJointState);
			int tInd = jointmdptoteammdpmap.daMap.get(jsInd);
			statesofar.setValue(tInd, jsVal);

		}
		for (int i = 0; i < this.numSharedStates; i++) {
			int jsInd = jointmdptoteammdpmap.ssInds[i];
			int jsVal = jointmdptoteammdpmap.getSSState(i, currentJointState);
			int tInd = jointmdptoteammdpmap.ssMap.get(jsInd);
			statesofar.setValue(tInd, jsVal);

		}
		//		Set<Integer> rIndsInJs = jointmdptoteammdpmap.rMap.keySet();
		for (int r = 0; r < numRobots; r++) {
			State rs = new State(statesofar);
			//get the rind for this 
			//			int jInd = 
			//set the rnum ind 
			rs.setValue(jointmdptoteammdpmap.rnumInd, r);
			ArrayList<Integer> rinds = jointmdptoteammdpmap.rInds.get(r);
			for (int j = 0; j < rinds.size(); j++) {
				int jind = rinds.get(j);
				//corresponding teammdp index is 
				int jVal = (int) currentJointState.varValues[jind];
				int tind = this.jointmdptoteammdpmap.rMap.get(jind);
				rs.setValue(tind, jVal);

			}
			statesToRet.add(rs);

		}
		return statesToRet;
	}

	public int findStateInStatesList(State s, List<State> statesList)
	{
		int sIndex = -1;
		for (int i = 0; i < statesList.size(); i++) {
			State sHere = statesList.get(i);
			if (sHere.compareTo(s) == 0) {
				sIndex = i;
				break;
			}
		}
		return sIndex;
	}

	public State createJointStateFromRobotStates(int initialState, SequentialTeamMDP seqTeamMDP)
	{
		State currentState = seqTeamMDP.teamMDPWithSwitches.getStatesList().get(initialState);
		int firstRobotNumber = StatesHelper.getRobotNumberFromSeqTeamMDPState(currentState);
		int[] currentRobotStates = new int[numRobots];
		currentRobotStates[firstRobotNumber] = initialState;
		for (int i = 0; i < numRobots; i++) {
			if (i != firstRobotNumber) {
				currentRobotStates[i] = seqTeamMDP.initialStates.get(i).nextSetBit(0);
			}
		}
		return createJointStateFromRobotStates(currentRobotStates, seqTeamMDP.teamMDPWithSwitches.getStatesList(), null, null, true);

	}

	//	int numStateVars()
	//	{
	//		return this.numRobots + this.numTasks + this.numSharedStates;
	//	}

	public State createJointStateFromRobotStates(int[] robotStates, List<State> statesList, State ps, State initialState, boolean useFirstRobotForDASS)
	{

		ArrayList<State> robotStatesStates = new ArrayList<>();
		for (int i = 0; i < robotStates.length; i++) {
			robotStatesStates.add(statesList.get(robotStates[i]));
		}
		//creating a joint state means 
		//getting the da states 
		//getting the shared states 
		//finally getting the robot states 
		return createJointStateFromRobotStates(robotStatesStates, ps, initialState, useFirstRobotForDASS);

	}

	public State createJointStateFromRobotStates(ArrayList<State> robotStates, State ps, State initialState, boolean useFirstRobotForDASS)
	{
		//so first we collect all the da states 
		//and then we just combine them 
		//how do we combine them 
		//a da state is basically updated if it is not the initial state 
		//and not the same as the previous state
		//initialState has the initial state values - root
		//ps has the parent state (immediate ancestor)
		//so now we have everything we need 
		State jointState;
		if (ps == null) {
			if (initialState == null)
				jointState = new State(jointmdptoteammdpmap.getNumVars());
			else
				jointState = new State(initialState);
		} else
			jointState = new State(ps);

		for (int i = 0; i < numTasks; i++) {
			int daVal = getDAState(i, robotStates, ps, initialState, useFirstRobotForDASS);
			int jointInd = this.jointmdptoteammdpmap.daInds[i];
			jointState.setValue(jointInd, daVal);

		}

		for (int i = 0; i < numSharedStates; i++) {
			int ssVal = getSState(i, robotStates, ps, initialState, useFirstRobotForDASS);
			int jointInd = this.jointmdptoteammdpmap.ssInds[i];
			jointState.setValue(jointInd, ssVal);

		}
		for (int i = 0; i < robotStates.size(); i++) {
			State rs = robotStates.get(i);
			//get robot number 
			int rnum = this.teammdptojointmdpmap.getRobotNumFromTeamMDPState(rs);
			//now get the robot state for this 
			//now just map them all 
			//get the index of all the robot states from teammdptojointmdpmap

			for (int j = 0; j < this.teammdptojointmdpmap.rInds.size(); j++) {
				int teamInd = teammdptojointmdpmap.rInds.get(j);
				int rsVal = (int) rs.varValues[teamInd];
				int jointInd = teammdptojointmdpmap.rMap.get(teamInd).get(rnum);
				jointState.setValue(jointInd, rsVal);

			}
		}

		return jointState;

	}

	int getDAState(int danum, ArrayList<State> rs, State ps, State initialState, boolean useFirstRobotForDASS)
	{
		boolean daValSet = false;
		int daVal = -1;
		int psVal = -1;
		int initVal = -1;
		//ps is a joint state
		if (ps != null) {
			psVal = this.jointmdptoteammdpmap.getDAState(danum, ps);
			daVal = psVal;
			daValSet = true;
		}
		if (initialState != null) {
			initVal = this.jointmdptoteammdpmap.getDAState(danum, initialState);
			if (!daValSet) {
				daVal = initVal;
				daValSet = true;
			}
		}

		int maxrnum = rs.size();
		if (useFirstRobotForDASS)
			maxrnum = 1;
		for (int r = 0; r < maxrnum; r++) {
			State state = rs.get(r);
			//now we get the da state from this 
			int rdaval = this.teammdptojointmdpmap.getDAState(danum, state);
			if (!daValSet) {
				daVal = rdaval;
				daValSet = true;
			} else {
				//lets do all our comparisons 
				//we update iff the current value is not the same as the next value 
				boolean doUpdate = true;
				if (daVal == rdaval) {
					doUpdate = false;
				} else {
					if (ps != null) {
						if (rdaval == psVal) {
							doUpdate = false;
						}
					}
					if (doUpdate) {
						if (initialState != null) {
							if (rdaval == initVal) {
								doUpdate = false;
							}
						}
					}
				}
				if (doUpdate) {
					daVal = rdaval;
				}
			}
		}
		return daVal;
	}

	int getSState(int ssnum, ArrayList<State> rs, State ps, State initialState, boolean useFirstRobotForDASS)
	{
		boolean ssValSet = false;
		int ssVal = -1;
		int psVal = -1;
		int initVal = -1;
		//ps is a joint state
		if (ps != null) {
			psVal = this.jointmdptoteammdpmap.getSSState(ssnum, ps);
			ssVal = psVal;
			ssValSet = true;
		}
		if (initialState != null) {
			initVal = this.jointmdptoteammdpmap.getSSState(ssnum, initialState);
			if (!ssValSet) {
				ssVal = initVal;
				ssValSet = true;
			}
		}
		int maxrnum = rs.size();
		if (useFirstRobotForDASS)
			maxrnum = 1;
		for (int r = 0; r < maxrnum; r++) {
			State state = rs.get(r);
			//now we get the da state from this 
			int rdaval = this.teammdptojointmdpmap.getSSState(ssnum, state);
			if (!ssValSet) {
				ssVal = rdaval;
				ssValSet = true;
			} else {
				//lets do all our comparisons 
				//we update iff the current value is not the same as the next value 
				boolean doUpdate = true;
				if (ssVal == rdaval) {
					doUpdate = false;
				} else {
					if (ps != null) {
						if (rdaval == psVal) {
							doUpdate = false;
						}
					}
					if (doUpdate) {
						if (initialState != null) {
							if (rdaval == initVal) {
								doUpdate = false;
							}
						}
					}
				}
				if (doUpdate) {
					ssVal = rdaval;
				}
			}
		}
		return ssVal;
	}

	private VarList createVarList(VarList seqTeamMDPVarList, ArrayList<DAInfo> daList) throws PrismException
	{
		teammdptojointmdpmap = new TeamMDPToJointMDPMap(this.numTasks, this.numSharedStates, this.numRobots, seqTeamMDPVarList.getNumVars());
		jointmdptoteammdpmap = new JointMDPToTeamMap(this.numTasks, this.numSharedStates, this.numRobots);
		VarList varlist = new VarList();

		// end of the list - isolated robot states in order
		// we're making the assumption that all isolated states can be lumped into one
		String decname;
		try {

			// keep the order of the shared states the same as the order in the seqteam mdp
			// so just go over all the states in the varlist

			for (int i = seqTeamMDPVarList.getNumVars() - 1; i >= 0; i--) {
				String name = seqTeamMDPVarList.getName(i);
				if (sharedStatesNamesList.contains(name)) {

					varlist.addVar(0, new Declaration(name, new DeclarationIntUnbounded()), 1, null);
				}
			}

			ArrayList<String> danames = new ArrayList<>();
			for (int i = seqTeamMDPVarList.getNumVars() - 1; i >= 0; i--) {
				String name = seqTeamMDPVarList.getName(i);
				if (name.contains("da")) {
					varlist.addVar(0, new Declaration(name, new DeclarationIntUnbounded()), 1, null);
					danames.add(name);

				}
			}
			//how many extras do we add 
			int robotStates = teammdptojointmdpmap.teamMDPNumVars - sharedStatesNamesList.size() - numTasks - 1;
			for (int i = 0; i < numRobots; i++) {
				if (robotStates == 1) {
					decname = "r" + i;
					varlist.addVar(new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
				} else {
					for (int j = 0; j < seqTeamMDPVarList.getNumVars(); j++) {
						String name = seqTeamMDPVarList.getName(j);
						if (!sharedStatesNamesList.contains(name) && !danames.contains(name) && !name.contentEquals("r")) {
							decname = "r" + i + "_" + name;
							varlist.addVar(new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
						}
					}
				}
			}
			for (int i = 0; i < sharedStatesNamesList.size(); i++) {
				String ssName = sharedStatesNamesList.get(i);

				int teamIndex = seqTeamMDPVarList.getIndex(ssName);

				int jointMDPIndex = varlist.getIndex(ssName);
				this.jointmdptoteammdpmap.setSSInd(i, jointMDPIndex);
				this.teammdptojointmdpmap.setSSInd(i, teamIndex);
			}
			for (int i = 0; i < danames.size(); i++) {
				DAInfo daInfo = daList.get(i);

				String daName = danames.get(i);

				int teamIndex = seqTeamMDPVarList.getIndex(daName);
				if (daInfo.associatedIndexInProduct != (teamIndex - 1))
					throw new PrismException("Not matching DA");
				int jointMDPIndex = varlist.getIndex(daName);
				this.jointmdptoteammdpmap.setDAInd(i, jointMDPIndex);
				this.teammdptojointmdpmap.setDAInd(i, teamIndex);
				//we've also got to save the acc states and initial states 
				this.jointmdptoteammdpmap.daInitialStates.add(daInfo.da.getStartState());
				if (daInfo.isSafeExpr) {
					BitSet newBS = new BitSet();
					newBS.set(daInfo.da.getStartState());
					jointmdptoteammdpmap.daFinalStates.add(newBS);
					jointmdptoteammdpmap.safetyDAInd = i;
				} else {
					jointmdptoteammdpmap.daFinalStates.add(daInfo.daAccStates);
				}

			}
			//			for (int i = 0; i < numRobots; i++) {
			//				String rName = "r" + i;
			//				int jointMDPIndex = varlist.getIndex(rName);
			//				this.jointmdptoteammdpmap.setRind(i, jointMDPIndex);
			//			}
			//mapping!! 
			for (int i = 0; i < seqTeamMDPVarList.getNumVars(); i++) {
				int teamIndex = i;
				String teamName = seqTeamMDPVarList.getName(teamIndex);
				int jointMDPIndex = -1;
				if (sharedStatesNamesList.contains(teamName)) {
					jointMDPIndex = varlist.getIndex(teamName);
					this.jointmdptoteammdpmap.ssMap.put(jointMDPIndex, teamIndex);
					this.teammdptojointmdpmap.ssMap.put(teamIndex, jointMDPIndex);
				} else if (danames.contains(teamName)) {
					jointMDPIndex = varlist.getIndex(teamName);
					this.jointmdptoteammdpmap.daMap.put(jointMDPIndex, teamIndex);
					this.teammdptojointmdpmap.daMap.put(teamIndex, jointMDPIndex);
				} else {
					//everything else is the robot 
					if (teamName.contentEquals("r")) {
						//if its just r it is the rnum ind 
						teammdptojointmdpmap.rnumInd = teamIndex;
						jointmdptoteammdpmap.rnumInd = teamIndex;

					} else {
						for (int r = 0; r < numRobots; r++) {
							String jointMDPName;
							if (robotStates == 1)
								jointMDPName = "r" + r;
							else {
								jointMDPName = "r" + r + "_" + teamName;
							}
							jointMDPIndex = varlist.getIndex(jointMDPName);
							this.jointmdptoteammdpmap.setRind(r, jointMDPIndex);
							this.jointmdptoteammdpmap.addr(jointMDPIndex, teamIndex);
							this.teammdptojointmdpmap.addr(teamIndex, jointMDPIndex);
							if (!teammdptojointmdpmap.rInds.contains(teamIndex))
								teammdptojointmdpmap.rInds.add(teamIndex);

						}
					}

				}
			}

			//so we have the seqTeamMDP 

		} catch (PrismLangException e) {
			e.printStackTrace();
		}

		return varlist;

	}

	public void createRewardStructures()
	{
		MDPSimple mdp = jointMDP.mdp;

		// the assumption is we're all done
		// so we can just add stuff
		if (otherRewardsHashMap != null & progressionRewardsHashMap != null) {
			progressionRewards = new MDPRewardsSimple(mdp.getNumStates());
			otherRewards = new ArrayList<MDPRewardsSimple>();

			for (Entry<Integer, Integer> saPair : progressionRewardsHashMap.keySet()) {
				progressionRewards.addToTransitionReward(saPair.getKey(), saPair.getValue(), progressionRewardsHashMap.get(saPair));
			}
			for (Entry<Integer, Integer> saPair : otherRewardsHashMap.keySet()) {
				for (int i = 0; i < otherRewardsHashMap.get(saPair).size(); i++) {
					if (otherRewards.size() < (i + 1)) {
						otherRewards.add(new MDPRewardsSimple(mdp.getNumStates()));
					}
					otherRewards.get(i).addToTransitionReward(saPair.getKey(), saPair.getValue(), otherRewardsHashMap.get(saPair).get(i));

				}
			}
		}
	}

	public ArrayList<MDPRewardsSimple> getExpTaskAndCostRewards()
	{
		ArrayList<MDPRewardsSimple> rewstoret = new ArrayList<MDPRewardsSimple>();
		rewstoret.add(progressionRewards);
		rewstoret.add(otherRewards.get(0));
		return rewstoret;
	}

}
