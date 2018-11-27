package objects;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

public interface Burdenable {

	public void addParcel(AidParcel p);
	public boolean removeParcel(AidParcel p);
	public void addParcels(ArrayList <AidParcel> ps);
	public boolean removeParcels(ArrayList <AidParcel> ps);
	public Coordinate getLocation();
	public boolean transferTo(Object o, Burdenable b);
}
