package org.d3.app.boids;

import java.io.IOException;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import org.d3.Actor;
import org.d3.ActorNotFoundException;
import org.d3.actor.ActorInternalException;
import org.d3.actor.Agency;
import org.d3.actor.Feature;
import org.d3.actor.RemoteActor;
import org.d3.actor.UnregisteredActorException;
import org.d3.agency.AgencyListener;
import org.d3.annotation.Callable;
import org.d3.remote.RemoteAgency;
import org.graphstream.boids.Boid;
import org.graphstream.boids.BoidGraph;
import org.graphstream.boids.BoidSpecies;
import org.graphstream.boids.forces.distributed.DistributedForces;
import org.graphstream.boids.forces.distributed.DistributedForcesFactory;
import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class DistributedBoidGraph extends Feature {

	public static final String CALLABLE_NEAR_OF = "boids.nearOf";
	public static final String CALLABLE_NEW = "boids.new";
	public static final String CALLABLE_DEL = "boids.del";
	public static final String CALLABLE_GET = "boids.get";
	public static final String CALLABLE_UPDATE_EDGES = "boids.update.edges";

	private BoidGraph localPart;
	private String boidConfiguration;
	private HashMap<Boid, DistributedBoid> boidDistribution;
	private List<RemoteActor> remoteParts, unmutableRemoteParts;

	protected DistributedBoidGraph(String id) {
		super(id);

		remoteParts = new CopyOnWriteArrayList<RemoteActor>();
		unmutableRemoteParts = Collections.unmodifiableList(remoteParts);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.Feature#initFeature()
	 */
	public void initFeature() {
		localPart = new BoidGraph();
		localPart.setForcesFactory(new DistributedForcesFactory(this));

		try {
			localPart.loadDGSConfiguration(boidConfiguration);
		} catch (IOException e) {
			e.printStackTrace();
		}

		boidDistribution = new HashMap<Boid, DistributedBoid>();

		for (Boid b : localPart.<Boid> getEachNode()) {
			BoidData data = new BoidData(b.getId(), b.getSpecies().getName());
			new DistributedBoid(b.getId(), getId(), data);
		}
	}

	public Iterable<RemoteActor> getEachPart() {
		return unmutableRemoteParts;
	}

	public void step() {

	}

	@Callable(CALLABLE_NEAR_OF)
	public Collection<BoidData> getBoidsNearOf(BoidData data) {
		checkActorThreadAccess();

		LinkedList<BoidData> boids = new LinkedList<BoidData>();
		BoidSpecies species = localPart.getOrCreateSpecies(data.speciesName);

		for (Boid b : localPart.<Boid> getEachNode()) {
			if (isVisible(data, species, b.getPosition()))
				boids.add(((DistributedForces) b.getForces()).getBoidData());
		}

		return boids;
	}

	@Callable(CALLABLE_NEW)
	public Boid hostNewBoid(DistributedBoid dBoid, BoidData data) {
		checkActorThreadAccess();

		Boid b = localPart.getNode(data.boidId);

		if (b == null)
			b = localPart.addNode(data.boidId);

		boidDistribution.put(b, dBoid);
		((DistributedForces) b.getForces()).setBoidData(data);

		return b;
	}

	@Callable(CALLABLE_GET)
	public Collection<Boid> getBoids(Collection<BoidData> neigh) {
		checkActorThreadAccess();

		LinkedList<Boid> boids = new LinkedList<Boid>();

		for (BoidData bd : neigh) {
			Boid b = localPart.getNode(bd.boidId);

			if (b == null) {
				try {
					Actor a = Agency.getLocalAgency().getActors().get(
							bd.getURI());

					if (a.isRemote()) {
						b = localPart.addNode(bd.getBoidId());
						b.addAttribute("remote");
					} else {
						throw new ActorInternalException("WTF?");
					}
				} catch (ActorNotFoundException e) {
					e.printStackTrace();
				} catch (UnregisteredActorException e) {
					e.printStackTrace();
				}
			}

			boids.add(b);
		}

		return boids;
	}

	@Callable(CALLABLE_UPDATE_EDGES)
	public void updateEdges(Boid b, Collection<BoidData> neigh) {
		checkActorThreadAccess();

		Collection<Boid> boids = getBoids(neigh);
		b.checkNeighborhood(boids.toArray(new Boid[boids.size()]));
	}

	protected boolean isVisible(BoidData boid, BoidSpecies species, Point3 point) {
		double d = boid.position.distance(point);

		if (d <= species.getViewZone()) {
			if (species.getAngleOfView() > -1) {
				double angle;
				Vector3 dir = new Vector3(boid.direction);
				Vector3 light = new Vector3(point.x - boid.position.x, point.y
						- boid.position.y, point.z - boid.position.z);

				dir.normalize();
				light.normalize();

				angle = dir.dotProduct(light);

				if (angle > species.getAngleOfView())
					return true;
			} else {
				return true;
			}
		}

		return false;
	}

	class RemotePartUpdater implements AgencyListener {
		/*
		 * (non-Javadoc)
		 * 
		 * @see org.d3.agency.AgencyListener#agencyExit(org.d3.actor.Agency)
		 */
		public void agencyExit(Agency agency) {
			// TODO Auto-generated method stub

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see
		 * org.d3.agency.AgencyListener#identifiableObjectRegistered(org.d3.
		 * Actor)
		 */
		public void identifiableObjectRegistered(Actor idObject) {
			// TODO Auto-generated method stub

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see
		 * org.d3.agency.AgencyListener#identifiableObjectUnregistered(org.d3
		 * .Actor)
		 */
		public void identifiableObjectUnregistered(Actor idObject) {
			// TODO Auto-generated method stub

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @seeorg.d3.agency.AgencyListener#newAgencyRegistered(org.d3.remote.
		 * RemoteAgency)
		 */
		public void newAgencyRegistered(RemoteAgency rad) {
			// TODO Auto-generated method stub

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see
		 * org.d3.agency.AgencyListener#remoteAgencyDescriptionUpdated(org.d3
		 * .remote.RemoteAgency)
		 */
		public void remoteAgencyDescriptionUpdated(RemoteAgency rad) {
			// TODO Auto-generated method stub

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see org.d3.agency.AgencyListener#requestReceived(org.d3.Actor,
		 * org.d3.Actor, java.lang.String)
		 */
		public void requestReceived(Actor source, Actor target, String name) {
			// TODO Auto-generated method stub

		}
	}
}
