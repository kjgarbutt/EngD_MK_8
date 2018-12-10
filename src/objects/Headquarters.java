package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import sim.EngD_MK_8;
import sim.engine.SimState;
import sim.engine.Steppable;
import swise.agents.SpatialAgent;
import swise.objects.network.GeoNode;
import utilities.HeadquartersUtilities;

public class Headquarters extends SpatialAgent implements Burdenable {

	EngD_MK_8 world;
	
	GeoNode myNode = null;
	
	ArrayList <AidParcel> parcels;
	ArrayList <ArrayList <AidParcel>> rounds;

	int numBays;
	ArrayList <Driver> inBays;
	ArrayList <Driver> waiting;
	
	public Headquarters (Coordinate c, int numbays, EngD_MK_8 world){
		super(c);
		parcels = new ArrayList <AidParcel> ();
		inBays = new ArrayList <Driver> ();
		waiting = new ArrayList <Driver> ();
		this.world = world;
		this.numBays = numbays;
		rounds = new ArrayList <ArrayList <AidParcel>> ();
	}
	
	public void setNode(GeoNode node){
		myNode = node;
	}
	
	public GeoNode getNode(){ return myNode;}
	
	@Override
	public void addParcel(AidParcel p) {
		parcels.add(p);
	}

	@Override
	public boolean removeParcel(AidParcel p) {
		return parcels.remove(p);
	}

	public boolean removeParcels(ArrayList <AidParcel> ps){
		return parcels.removeAll(ps);
	}
	

	@Override
	public void addParcels(ArrayList<AidParcel> ps) {
		parcels.addAll(ps);
	}

	@Override
	public boolean transferTo(Object o, Burdenable b) {
		try{
			if(o instanceof ArrayList){
				parcels.removeAll((ArrayList <AidParcel>) o);
				b.addParcels((ArrayList <AidParcel>) o);
			}
			else {
				parcels.remove((AidParcel) o);
				b.addParcel((AidParcel) o);
			}
			return true;
		} catch (Exception e){
			return false;
		}
	}

	
	@Override
	public Coordinate getLocation() {
		return geometry.getCoordinate();
	}
	
	@Override
	public void step(SimState arg0){
		world.schedule.scheduleOnce(this);
	}
	
	/**
	 * 
	 * @param d - the driver
	 * @return the amount of time before which to activate again. If <0, the Depot will
	 * activate the Driver when ready. 
	 */
	public int enterDepot(Driver d){
		
		//System.out.println("Driver: " + d.toString() + " has entered HQ!");
		System.out.println(d.driverID + " has entered HQ!");
		
		if(rounds.size() == 0)
			return -1; // finished with everything
		else if(inBays.size() >= numBays){
			waiting.add(d);
			world.schedule.scheduleOnce(new Steppable(){

				@Override
				public void step(SimState state) {
					if(inBays.size() < numBays){
						waiting.remove(d);
						enterBay(d);
					}
					else
						state.schedule.scheduleOnce(this);
				}
				
			});
		}
		else {
			enterBay(d);
			
		}
		
		return EngD_MK_8.loadingTime;
	}
	
	ArrayList <AidParcel> getNextRound(){
		return rounds.remove(0);
	}
	
	void enterBay(Driver d){
		inBays.add(d);
		if(rounds.size() <= 0)
			return;
		
		else
			world.schedule.scheduleOnce(world.schedule.getTime() + world.loadingTime, new Steppable(){

				@Override
				public void step(SimState state) {
					ArrayList <AidParcel> newRound = getNextRound();
					if(d.myVehicle != null){
						transferTo(newRound, d.myVehicle);	
						d.updateRound();
					}
					else
						d.addParcels(newRound);
					
					System.out.println(d.toString() + " has taken on a new load: " + newRound.toArray().toString());
					leaveDepot(d);
					d.startRoundClock();
				}
			
			});
	}
	
	/**
	 * 
	 * @param d the Driver to remove from the Depot
	 */
	public void leaveDepot(Driver d){
		
		// if the Driver was originally there, remove it
		if(inBays.contains(d)){
			inBays.remove(d);
			world.schedule.scheduleOnce(d);
			
			// if there are Drivers waiting in the queue, let the next one move in
			if(waiting.size() > 0){
				Driver n = waiting.remove(0);
				inBays.add(n);
				world.schedule.scheduleOnce(world.schedule.getTime() + EngD_MK_8.loadingTime, n);
			}
		}
		else
			System.out.println("Error: driver was never in bay");
	}
	
	public void addRounds(ArrayList <ArrayList <AidParcel>> rounds){
		this.rounds = rounds;
	}
	
	public void generateRounds(){
		rounds.addAll(HeadquartersUtilities.gridDistribution(parcels, world.deliveryLocationLayer, world.approxManifestSize));
	}
}