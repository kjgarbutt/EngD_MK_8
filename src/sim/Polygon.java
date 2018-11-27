package sim;

import sim.util.geo.MasonGeometry;
import java.util.ArrayList;

import schellingpolygon.Person;

/**
 * Polygon.java
 *
 * Copyright 2011 by Sarah Wise, Mark Coletti, Andrew Crooks, and
 * George Mason University.
 *
 * Licensed under the Academic Free License version 3.0
 *
 * See the file "LICENSE" for more information
 *
 * $Id: Polygon.java 842 2012-12-18 01:09:18Z mcoletti $
 */
public class Polygon extends MasonGeometry	{
	int id = -1;
	String polyColour;

    ArrayList<Polygon> osviPolygons;

    public Polygon()	{
        super();
        osviPolygons = new ArrayList<Polygon>();
    }

    public void init()	{
    	id = getIntegerAttribute("ID").intValue();
        polyColour = getStringAttribute("RankColN");
    }
    
    int getID()
    {
        if (id == -1)
        {
            init();
        }
        return id;
    }

    String getPolyColour()	{
        if (polyColour == null)
        {
            init();
        }
        return polyColour;
    }
}