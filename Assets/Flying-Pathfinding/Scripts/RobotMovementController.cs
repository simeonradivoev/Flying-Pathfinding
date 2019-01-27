using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class RobotMovementController : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private float maxDistanceRebuildPath = 1;
    [SerializeField] private float acceleration = 1;
    [SerializeField] private float minReachDistance = 2f;
    [SerializeField] private float minFollowDistance = 4f;
    [SerializeField] private float pathPointRadius = 0.2f;
    [SerializeField] private Octree octree;
    [SerializeField] private LayerMask playerSeeLayerMask = -1;
    [SerializeField] private GameObject playerObject;
    private Octree.PathRequest oldPath;
    private Octree.PathRequest newPath;
    private Rigidbody rigidbody;
    private Vector3 currentDestination;
    private Vector3 lastDestination;
    private SphereCollider sphereCollider;

    public GameObject Target
    {
        get { return target.gameObject; }
        set { target = value.transform; }
    }

    // Use this for initialization
    void Start()
    {
        sphereCollider = GetComponent<SphereCollider>();
        rigidbody = GetComponent<Rigidbody>();
        octree = FindObjectOfType<Octree>();
        if (octree == null)
        {
            Debug.LogError("There is no `octree` component in your seen. Please add one so that Flying-Pathfinding can work.");
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (target == null)
        {
            return;
        }
        if ((newPath == null || !newPath.isCalculating) && Vector3.SqrMagnitude(target.transform.position - lastDestination) > maxDistanceRebuildPath && (!CanSeePlayer() || Vector3.Distance(target.position, transform.position) > minFollowDistance) && !octree.IsBuilding)
        {
            lastDestination = target.transform.position;

            oldPath = newPath;
            newPath = octree.GetPath(transform.position, lastDestination);
        }

        /*if (newPath != null && !newPath.isCalculating)
		{
			if (newPath.Path.Count > 0)
			{
				float distanceSoFar = 0;
				int lastPoint = newPath.Path.Count - 1;
				for (int i = lastPoint; i >= 1; i--)
				{
					distanceSoFar += Vector3.Distance(newPath.Path[i], newPath.Path[i - 1]);

					if (distanceSoFar <= minFollowDistance)
					{
						lastPoint = i;
					}
					else
					{
						break;
					}
				}
				if (lastPoint > 0)
				{
					newPath.Path.RemoveRange(lastPoint, newPath.Path.Count - lastPoint);
				}
			}
		}*/

        var curPath = Path;

        if (!curPath.isCalculating && curPath != null && curPath.Path.Count > 0)
        {
            if (Vector3.Distance(transform.position, target.position) < minFollowDistance && CanSeePlayer())
            {
                curPath.Reset();
            }

            currentDestination = curPath.Path[0] + Vector3.ClampMagnitude(rigidbody.position - curPath.Path[0], pathPointRadius);

            rigidbody.velocity += Vector3.ClampMagnitude(currentDestination - transform.position, 1) * Time.deltaTime * acceleration;
            float sqrMinReachDistance = minReachDistance * minReachDistance;

            Vector3 predictedPosition = rigidbody.position + rigidbody.velocity * Time.deltaTime;
            float shortestPathDistance = Vector3.SqrMagnitude(predictedPosition - currentDestination);
            int shortestPathPoint = 0;

            for (int i = 0; i < curPath.Path.Count; i++)
            {
                float sqrDistance = Vector3.SqrMagnitude(rigidbody.position - curPath.Path[i]);
                if (sqrDistance <= sqrMinReachDistance)
                {
                    if (i < curPath.Path.Count)
                    {
                        curPath.Path.RemoveRange(0, i + 1);
                    }
                    shortestPathPoint = 0;
                    break;
                }

                float sqrPredictedDistance = Vector3.SqrMagnitude(predictedPosition - curPath.Path[i]);
                if (sqrPredictedDistance < shortestPathDistance)
                {
                    shortestPathDistance = sqrPredictedDistance;
                    shortestPathPoint = i;
                }
            }

            if (shortestPathPoint > 0)
            {
                curPath.Path.RemoveRange(0, shortestPathPoint);
            }
        }
        else
        {
            rigidbody.velocity -= rigidbody.velocity * Time.deltaTime * acceleration;
        }
    }

    private bool CanSeePlayer()
    {
        RaycastHit hit;
        if (Physics.Raycast(new Ray(transform.position, transform.position - target.position), out hit, Vector3.Distance(transform.position, target.position) + 1, playerSeeLayerMask))
        {
            return hit.transform.gameObject == playerObject;
        }
        return false;
    }

    private Octree.PathRequest Path
    {
        get
        {
            if ((newPath == null || newPath.isCalculating) && oldPath != null)
            {
                return oldPath;
            }
            return newPath;
        }
    }

    public bool HasTarget
    {
        get
        {
            return Path != null && Path.Path.Count > 0;

        }
    }

    public Vector3 CurrentTargetPosition
    {
        get
        {
            if (Path != null && Path.Path.Count > 0)
            {
                return currentDestination;
            }
            else
            {
                return target.position;
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (rigidbody != null)
        {
            Gizmos.color = Color.blue;
            Vector3 predictedPosition = rigidbody.position + rigidbody.velocity * Time.deltaTime;
            Gizmos.DrawWireSphere(predictedPosition, sphereCollider.radius);
        }

        if (Path != null)
        {
            var path = Path;
            for (int i = 0; i < path.Path.Count - 1; i++)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(path.Path[i], minReachDistance);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(path.Path[i], Vector3.ClampMagnitude(rigidbody.position - path.Path[i], pathPointRadius));
                Gizmos.DrawWireSphere(path.Path[i], pathPointRadius);
                Gizmos.DrawLine(path.path[i], path.Path[i + 1]);
            }
        }
    }
}
