package org.d3.app.boids;

import java.util.Collection;
import java.util.LinkedList;
import java.util.concurrent.LinkedBlockingQueue;

import org.d3.ActorNotFoundException;
import org.d3.Console;
import org.d3.actor.ActorInternalException;
import org.d3.actor.Agency;
import org.d3.actor.CallException;
import org.d3.actor.Entity;
import org.d3.actor.Future;
import org.d3.actor.LocalActor;
import org.d3.actor.RemoteActor;
import org.d3.actor.UnregisteredActorException;
import org.d3.annotation.ActorPath;
import org.d3.annotation.Callable;
import org.d3.annotation.Direct;
import org.d3.annotation.Local;
import org.d3.entity.Migratable;
import org.d3.entity.migration.MigrationException;
import org.d3.remote.RemoteAgency;
import org.d3.tools.FutureToQueue;
import org.graphstream.boids.Boid;
import org.graphstream.boids.forces.distributed.DistributedForces;
import org.miv.pherd.geom.Point3;

@ActorPath("/boids")
public class DistributedBoid extends Entity {
	private static final long serialVersionUID = 254153604563418868L;

	public static final String CALLABLE_STEP = "boid.step";
	public static final String CALLABLE_SWAP = "boid.swap";
	public static final String CALLABLE_GET_DATA = "boid.data";

	protected Boid boid;
	protected DistributedBoidGraph boidGraph;

	@Migratable
	protected String part;

	@Migratable
	protected BoidData data;

	public DistributedBoid(String id) {
		this(id, null, null);
	}

	public DistributedBoid(String id, String part, BoidData data) {
		super(id);

		this.data = data;
		this.part = part;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.Entity#initEntity()
	 */
	@Override
	public void initEntity() {
		LocalActor ac;
		String path = "/boids/" + part;

		try {
			ac = Agency.getLocalAgency().getActors().get(path);
			boidGraph = (DistributedBoidGraph) ac;
		} catch (ActorNotFoundException e) {
			throw new ActorInternalException(e);
		} catch (UnregisteredActorException e) {
			throw new ActorInternalException(e);
		}

		data.setURI(getURI());
		initBoid();

		Console.info("started on part %s", path);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.Entity#beforeMigration()
	 */
	@Override
	public void beforeMigration() {
		boidGraph.call(DistributedBoidGraph.CALLABLE_DEL, data.boidId);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.Entity#migrationFailed(org.d3.remote.RemoteAgency,
	 * org.d3.entity.migration.MigrationException)
	 */
	@Override
	public void migrationFailed(RemoteAgency dest, MigrationException e) {
		initBoid();
	}

	@Direct
	@Callable(CALLABLE_GET_DATA)
	public BoidData getBoidData() {
		return data;
	}

	@Local
	@Callable(CALLABLE_SWAP)
	public void swapPosition() {
		Point3 p = boid.getForces().getNextPosition();
		boid.setPosition(p.x, p.y, p.z);
	}
	
	@Local
	@Callable(CALLABLE_STEP)
	public void step() {
		LinkedList<BoidData> neigh = new LinkedList<BoidData>();
		LinkedBlockingQueue<Future> queue = new LinkedBlockingQueue<Future>();
		int size = 1;

		boidGraph.call(DistributedBoidGraph.CALLABLE_NEAR_OF,
				new FutureToQueue(queue), data);

		for (RemoteActor remotePart : boidGraph.getEachPart()) {
			remotePart.call(DistributedBoidGraph.CALLABLE_NEAR_OF,
					new FutureToQueue(queue), data);
			size++;
		}
		
		while (size > 0) {
			try {
				Future f = queue.take();
				size--;

				try {
					Collection<BoidData> c = f.getValue();
					neigh.addAll(c);
				} catch (CallException e) {
					Console.exception(e);
				}
			} catch (InterruptedException e) {
				Console.exception(e);
			}
		}

		//Console.info("step (size=%d, neigh=%d)", s1, neigh.size());

		try {
			((DistributedForces) boid.getForces())
					.setCurrentNeighborhood(neigh);
			boid.getForces().compute();
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	protected void initBoid() {
		Future future = new Future();

		if (data == null)
			throw new NullPointerException();

		boidGraph.call(DistributedBoidGraph.CALLABLE_NEW, future, this, data);

		try {
			future.waitForValue();
			boid = future.get();
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (CallException e) {
			e.printStackTrace();
		}
	}
}
