package org.d3.app.boids;

import java.util.Collection;
import java.util.LinkedList;

import org.d3.actor.Feature;
import org.d3.actor.RemoteActor;
import org.d3.annotation.Callable;
import org.graphstream.boids.Boid;
import org.graphstream.boids.BoidGraph;
import org.graphstream.boids.BoidSpecies;
import org.graphstream.boids.forces.distributed.DistributedForces;
import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class DistributedBoidGraph extends Feature {

	public static final String CALLABLE_NEAR_OF = "boids.nearOf";
	public static final String CALLABLE_NEW = "boids.new";
	public static final String CALLABLE_DEL = "boids.del";

	BoidGraph localPart;

	protected DistributedBoidGraph(String id) {
		super(id);
	}

	public void initFeature() {
		// TODO Auto-generated method stub

	}

	@Callable(CALLABLE_NEAR_OF)
	public Collection<BoidData> getBoidsNearOf(BoidData data) {
		LinkedList<BoidData> boids = new LinkedList<BoidData>();
		BoidSpecies species = localPart.getOrCreateSpecies(data.speciesName);

		for (Boid b : localPart.<Boid> getEachNode()) {
			if (isVisible(data, species, b.getPosition()))
				boids.add(((DistributedForces) b.getForces()).getBoidData());
		}

		return boids;
	}

	@Callable(CALLABLE_NEW)
	public void hostNewBoid(BoidData data) {
		Boid b;

		if (data != null)
			b = localPart.addNode(data.boidId);
		else
			b = localPart.addNode(localPart.getDefaultSpecies().createNewId());

		if (data != null && data.position != null)
			b.getForces().setPosition(data.position.x, data.position.y,
					data.position.z);

		if (data != null && data.direction != null)
			b.getForces().getDirection().copy(data.direction);
	}

	public Iterable<RemoteActor> getEachPart() {
		// TODO
		return null;
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
}
