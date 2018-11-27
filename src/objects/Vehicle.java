package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import sim.util.geo.MasonGeometry;
import swise.agents.MobileAgent;

public class Vehicle extends MobileAgent implements Burdenable {

	Driver owner;
	ArrayList <AidParcel> parcels = new ArrayList <AidParcel> ();
	
	public Vehicle(Coordinate c, Driver d){
		super((Coordinate)c.clone());
		isMovable = true;
		owner = d;
	}
	
	@Override
	public void addParcel(AidParcel p) {
		parcels.add(p);
	}

	@Override
	public boolean removeParcel(AidParcel p) {
		return parcels.remove(p);
	}

	@Override
	public void addParcels(ArrayList<AidParcel> ps) {
		parcels.addAll(ps);
	}

	@Override
	public boolean removeParcels(ArrayList<AidParcel> ps) {
		return parcels.removeAll(ps);
	}

	@Override
	public Coordinate getLocation() {
		return geometry.getCoordinate();
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
	
	void setStationary(){
		if(owner != null){
			Coordinate c = owner.getLocation();
			updateLoc(new Coordinate(c.x, c.y));
			owner = null;
		}
		else
			updateLoc(getLocation());
	}
	
	void setDriver(Driver d){
		this.owner = d;
	}
}