package demos;

import java.util.BitSet;

import parser.State;

/*
 * Storing state related information parent state, robot number, probability etc
 * just for ease of use
 */
public class StateExtended implements Comparable<StateExtended>
{
	protected int parentState = -1;
	protected int parentStateRobot = -1;
	protected int childState = -1;
	protected int childStateRobot = -1;
	protected double parentToChildTransitionProbability = -1;
	protected String actionInChildState = null;
	protected int choice = -1;
	public BitSet statesToAvoid = null;
	protected State childStateState = null;

	public StateExtended()
	{
		// dummy
	}


	@Override
	public boolean equals(Object obj)
	{
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		StateExtended other = (StateExtended) obj;
		if(this.childStateState!=null)
		{
			if(other.childStateState!=null)
			{
				if(this.childStateState == other.childStateState)
				{
					return true; 
				}
			}
		}
		if(this.childState == other.childState)
		{
			if(this.parentToChildTransitionProbability == other.parentToChildTransitionProbability)
				return true; 
		}
			return false; 
//		if (actionInChildState == null) {
//			if (other.actionInChildState != null)
//				return false;
//		} else if (!actionInChildState.equals(other.actionInChildState))
//			return false;
//		if (childState != other.childState)
//			return false;
//		if (childStateRobot != other.childStateRobot)
//			return false;
//		if (childStateState == null) {
//			if (other.childStateState != null)
//				return false;
//		} else if (!childStateState.equals(other.childStateState))
//			return false;
//		if (choice != other.choice)
//			return false;
//		if (parentState != other.parentState)
//			return false;
//		if (parentStateRobot != other.parentStateRobot)
//			return false;
//		if (Double.doubleToLongBits(parentToChildTransitionProbability) != Double.doubleToLongBits(other.parentToChildTransitionProbability))
//			return false;
//		if (statesToAvoid == null) {
//			if (other.statesToAvoid != null)
//				return false;
//		} else if (!statesToAvoid.equals(other.statesToAvoid))
//			return false;
//		return true;
	}

	public StateExtended(int ps, int psr, int cs, int csr, double prob, String a)
	{
		parentState = ps;
		parentStateRobot = psr;
		childState = cs;
		childStateRobot = csr;
		parentToChildTransitionProbability = prob;
		actionInChildState = a;
	}

	public StateExtended(int s, double prob, String a)
	{
		childState = s;
		parentToChildTransitionProbability = prob;
		actionInChildState = a;
	}

	public StateExtended(StateExtended other)
	{
		this.parentState = other.parentState;
		this.parentStateRobot = other.parentStateRobot;
		this.childState = other.childState;
		this.childStateRobot = other.childStateRobot;
		this.parentToChildTransitionProbability = other.parentToChildTransitionProbability;
		this.actionInChildState = other.actionInChildState;
	}

	public StateExtended(int initialState, double d)
	{
		childState = initialState;
		parentToChildTransitionProbability = d;

	}
	public StateExtended(State s, double d)
	{
		childStateState = s;
		parentToChildTransitionProbability = d;

	}
	public State getChildStateState()
	{
		return childStateState; 
	}
	@Override
	public int compareTo(StateExtended other)
	{
		double comp = this.parentToChildTransitionProbability - other.parentToChildTransitionProbability;
		int res = 0;
		if (comp > 0)
			res = -1;
		else {
			if (comp < 0) {
				res = 1;
			}
		}
		return res;
	}

	@Override
	public String toString()
	{
		String strtoret = "[";

		if (parentState != -1)
			strtoret += "ps=" + parentState;

		if (parentStateRobot != -1)
			strtoret += ", psRob=" + parentStateRobot;

		if (childState != -1)
			strtoret += ", cs=" + childState;

		if (childStateRobot != -1)
			strtoret += ", csRob=" + childStateRobot;

		if (parentToChildTransitionProbability > 0)
			strtoret += ", ps->csProb=" + parentToChildTransitionProbability;

		if (actionInChildState != null)
			strtoret += ", a=" + actionInChildState;
		strtoret += "]";

		return strtoret; 
	}

}