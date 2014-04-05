package agents;
/**
 * Author: Rouhollah
 * This class contains the VC information each node has
 */
import java.io.Serializable;
import java.util.AbstractMap.SimpleEntry;
import java.util.HashMap;
import java.util.Set;

import yaes.ui.text.TextUi;

public class VCLocalTable implements Serializable {

	private static final long serialVersionUID = -6214223276984851880L;
	
	private long updateTime = 0;
	HashMap<MySimpleEntry<VCAgent, VCAgent>, Integer> hopsFromNodesToAnchors = new HashMap<MySimpleEntry<VCAgent,VCAgent>, Integer>();
	
	public int getNumberOfHops(VCAgent source, VCAgent anchor) {
		return getNumberOfHops(new MySimpleEntry<>(source, anchor));
	}
	
	public void setNumberOfHops(VCAgent source, VCAgent anchor, int hopsCount) {
		setNumberOfHops(new MySimpleEntry<>(source, anchor), hopsCount);
	}
	
	public int getNumberOfHops(MySimpleEntry<VCAgent,VCAgent> key) {
		return hopsFromNodesToAnchors.get(key);
	}
	
	public void setNumberOfHops(MySimpleEntry<VCAgent, VCAgent> key, int hopsCount) {
//		if(key.getKey().getContext().isForwardVCofAllNodes() || key.getKey().isAnchor() ) {
			hopsFromNodesToAnchors.put(key, hopsCount);
//		}
	}
	
	public boolean containsKey(MySimpleEntry<VCAgent, VCAgent> simpleEntry) {
		return hopsFromNodesToAnchors.containsKey(simpleEntry);
//		Set<MySimpleEntry<VCAgent, VCAgent>> keyArray = hopsFromNodesToAnchors.keySet();
//		for(MySimpleEntry<VCAgent,VCAgent> key : keyArray)
//			if(key.getKey().equals(simpleEntry.getKey()) && key.getValue().equals(simpleEntry.getValue()))
//				return true;
//		return false;
	}
	
	public Set<MySimpleEntry<VCAgent,VCAgent>> keySet() {
		return hopsFromNodesToAnchors.keySet();
	}
	
	/**
	 * This method takes the VC table information from the current message and updates it 
	 * with the local table at the node. 
	 * @param sender
	 * @param receiver
	 * @param currentTable
	 * @param receivedTable
	 * @return
	 */
	public static VCLocalTable combineVCLocalTables(VCAgent sender, VCAgent receiver, VCLocalTable currentTable, VCLocalTable receivedTable) {
		Set<MySimpleEntry<VCAgent,VCAgent>> keyArray = receivedTable.keySet();
		for( MySimpleEntry<VCAgent,VCAgent> key : keyArray) {
			if(!currentTable.containsKey(key) || (currentTable.containsKey(key) 
					&& currentTable.getNumberOfHops(key) > receivedTable.getNumberOfHops(key) )) {
				currentTable.setNumberOfHops(key, receivedTable.getNumberOfHops(key));
//				messagesForwardedWithoutVCUpdate = 0;
			}
			if(key.getKey().getName().equals(sender.getName())) {
				MySimpleEntry<VCAgent,VCAgent> newKey = new MySimpleEntry<VCAgent, VCAgent>(receiver, key.getValue());
				if(!currentTable.containsKey(newKey) || (currentTable.containsKey(newKey) 
						&& currentTable.getNumberOfHops(newKey) > receivedTable.getNumberOfHops(key) + 1 )) {
					currentTable.setNumberOfHops(newKey, receivedTable.getNumberOfHops(key) + 1);
				}
			}
//			TextUi.print("(" + key + " " + (Integer)message.getValue(key) + "/" + key + " " + (Integer)hopsToAnchors.get(key) + ") ");
		}
		return currentTable;
	}

	public long getUpdateTime() {
		return updateTime;
	}

	public void setUpdateTime(long updateTime) {
		this.updateTime = updateTime;
	}
}

class MySimpleEntry<T, E>  extends SimpleEntry<T, E> implements Serializable{

	/**
	 * 
	 */
	private static final long serialVersionUID = -902761315503385178L;
	public MySimpleEntry(T arg0, E arg1) {
		super(arg0, arg1);
	}
	@Override
	public boolean equals(Object o) {
		try {
			@SuppressWarnings("unchecked")
			MySimpleEntry<VCAgent, VCAgent> entry = (MySimpleEntry<VCAgent, VCAgent>) o;
			if(((VCAgent)this.getKey()).getName().equals(entry.getKey().getName()) && ((VCAgent)this.getValue()).getName().equals(entry.getValue().getName())) {
				return true;
			}
		} catch(Exception e) {
			e.printStackTrace();
			return false;
		}
		return false;
	}
	

	
	
}
