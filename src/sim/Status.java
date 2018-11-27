/**
*
* "MK_7_" is iteration 7.1 of my EngD project model. It varies little from MK_7, 
* and just has updated details to include Gloucestershire date. It was originally
* adapted from the MASON demo, "Gridlock", made by Sarah Wise, Mark Coletti, and 
* Andrew Crooks.
* 
* The model reads a number of GIS shapefiles and displays a road network, two 
* Environment Agency flood maps and a bespoke Open Source Vulnerability Index 
* (OSVI). The model reads in a .CSV and generates a predetermined number of agents 
* with set characteristics. The agents are placed on the road network and are 
* located at a Red Cross office. The model reads a separate .CSV and assigns goal 
* locations to each agent at random from a predetermined list. The agents are 
* assigned speeds at random. Once the model is started, the agents move from 
* A to B, then they change direction and head back to their start position. 
* The process repeats until the user quits.
*
*	REPLENISHING = stocking up on supplies whilst at HQ
*	OUTBOUND = travelling to LSOA
*	DISTRIBUTING = at LSOA and distributing supplies
*	INBOUND = travelling back to HQ
*
* @author KJGarbutt
*
*/
package sim;

public enum Status { REPLENISHING, OUTBOUND, DISTRIBUTING, INBOUND };

