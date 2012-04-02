package org.graphstream.boids.forces.distributed;

import java.util.Collection;

import org.d3.Actor;
import org.d3.Console;
import org.d3.actor.CallException;
import org.d3.actor.Future;
import org.d3.app.boids.BoidData;
import org.d3.app.boids.DistributedBoid;
import org.d3.app.boids.DistributedBoidGraph;
import org.graphstream.boids.Boid;
import org.graphstream.boids.BoidForces;
import org.graphstream.boids.BoidGraph;
import org.graphstream.boids.BoidSpecies;
import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class DistributedForces extends BoidForces {

	DistributedBoidGraph localPart;
	Actor distributedBoid;
	Collection<BoidData> currentNeigh;
	BoidData data;
	Point3 nextPosition;

	public DistributedForces(DistributedBoidGraph localPart, Boid b) {
		super(b);

		this.currentNeigh = null;
		this.localPart = localPart;
		this.nextPosition = new Point3();
	}

	public void setBoidData(BoidData data) {
		this.data = data;
		this.dir = data.getDirection();
	}

	public BoidData getBoidData() {
		return data;
	}

	public void setActor(Actor actor) {
		distributedBoid = actor;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForces#getNeighborhood()
	 */
	public Collection<Boid> getNeighborhood() {
		throw new UnsupportedOperationException();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForces#getNextPosition()
	 */
	public Point3 getNextPosition() {
		return nextPosition;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForces#getPosition()
	 */
	public Point3 getPosition() {
		return data.getPosition();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForces#setPosition(double, double, double)
	 */
	public void setPosition(double x, double y, double z) {
		data.getPosition().set(x, y, z);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForces#compute()
	 */
	@Override
	public void compute() {
		if (distributedBoid.isRemote())
			computeRemote();
		else
			computeLocal();
		
		localPart.call(DistributedBoidGraph.CALLABLE_UPDATE_BOID, boid, currentNeigh);
	}

	public void computeLocal() {
		Collection<BoidData> neigh = currentNeigh;
		BoidSpecies species = boid.getSpecies();
		Vector3 dir = getDirection();
		Vector3 rep = new Vector3();
		Point3 nextPos = getNextPosition();

		nextPos.copy(data.getPosition());
		barycenter.set(0, 0, 0);
		direction.fill(0);
		attraction.fill(0);
		repulsion.fill(0);
		countAtt = 0;
		countRep = 0;

		if (neigh != null)
			for (BoidData b : neigh)
				actionWithNeighboor(b, rep);

		if (countAtt > 0) {
			barycenter.scale(1f / countAtt, 1f / countAtt, 1f / countAtt);
			direction.scalarDiv(countAtt);
			attraction
					.set(barycenter.x - boid.getPosition().x, barycenter.y
							- boid.getPosition().y, barycenter.z
							- boid.getPosition().z);
		}

		if (countRep > 0) {
			repulsion.scalarDiv(countRep);
		}

		direction.scalarMult(species.getDirectionFactor());
		attraction.scalarMult(species.getAttractionFactor());
		repulsion.scalarMult(species.getRepulsionFactor());
		dir.scalarMult(species.getInertia());

		dir.add(direction);
		dir.add(attraction);
		dir.add(repulsion);

		if (((BoidGraph) boid.getGraph()).isNormalizeMode()) {
			double len = dir.normalize();
			if (len <= species.getMinSpeed())
				len = species.getMinSpeed();
			else if (len >= species.getMaxSpeed())
				len = species.getMaxSpeed();

			dir.scalarMult(species.getSpeedFactor() * len);
		} else {
			dir.scalarMult(species.getSpeedFactor());
		}

		checkWalls();
		nextPos.move(dir);
	}

	protected void computeRemote() {
		if (boid.getDegree() == 0) {

		} else {
			Future f = new Future();
			BoidData bd;
			Point3 nextPos = getNextPosition();

			distributedBoid.call(DistributedBoid.CALLABLE_GET_DATA, f);

			try {
				bd = f.getValue();

				nextPos.copy(bd.getPosition());
				getDirection().copy(bd.getDirection());
			} catch (CallException e) {
				Console.exception(e);
			} catch (InterruptedException e) {
				Console.exception(e);
			}
		}
	}

	protected void actionWithNeighboor(BoidData b, Vector3 rep) {
		Point3 p1 = boid.getPosition();
		Point3 p2 = b.getPosition();

		BoidSpecies p1Species = boid.getSpecies();
		BoidSpecies p2Species = ((BoidGraph) boid.getGraph()).getSpecies(b
				.getSpeciesName());

		double v = boid.getSpecies().getViewZone();

		rep.set(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);

		double len = rep.length();

		if (len != 0) {
			if (p1Species != p2Species)
				rep.scalarMult(1 / (len * len) * p2Species.getFearFactor());
			else
				rep.scalarMult(1 / (len * len));
		}

		double a = Math.log(Math.min(len, v)) / Math.log(v);
		rep.scalarMult(a);

		repulsion.add(rep);
		countRep++;

		if (p1Species == p2Species) {
			barycenter.move(p2);
			direction.add(b.getDirection());
			countAtt++;
		}
	}

	public void setCurrentNeighborhood(Collection<BoidData> neigh) {
		currentNeigh = neigh;
	}
}
