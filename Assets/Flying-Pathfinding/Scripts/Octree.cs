using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Priority_Queue;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class Octree : MonoBehaviour
{
    [Tooltip("Turn on debug information, such as providing Gizmo's showing cell status. Note that drawing all these Gizmo's takes some time, so normally you'll want this set to false.")]
    public bool isDebug = false;

	[SerializeField] private float minCellSize = 2;
	[SerializeField] private LayerMask mask = -1;
	[SerializeField] private float maxMilisecondsPerFrame = 10;
	[SerializeField] private int cellCount;
	[SerializeField] private Transform player;
	[SerializeField] private Transform destination;
	//[SerializeField] private PathfindingAlgorith algorithm = PathfindingAlgorith.AStar;
	[SerializeField] private float maxActivePathfinds = 6;

	private BoxCollider boxCollider;
	private OctreeElement root;
	private Queue<OctreeElement> toBeSplit = new Queue<OctreeElement>();
	private Queue<PathRequest> requests = new Queue<PathRequest>();
	private List<PathRequest> running = new List<PathRequest>();
       
    /// <summary>
    /// Test if the cell that encompasses a given position is traversable.
    /// </summary>
    /// <param name="position">The position the cell most embody.</param>
    /// <returns>True is the node containing the position is traversable, otherwise returns false.</returns>
    internal bool IsTraversableCell(Vector3 position)
    {
        if (!Contains(position))
        {
            return false;
        }
        Octree.OctreeElement node = GetNode(position);
        return node != null ? node.Empty : false;
    }

    // Use this for initialization
    void Start ()
	{
		boxCollider = GetComponent<BoxCollider>();
		root = new OctreeElement(null,boxCollider.bounds,0);
		toBeSplit.Enqueue(root);
		cellCount++;
		float doubleMinCellSize = minCellSize * 2f;

		while (toBeSplit.Count > 0)
		{
			var elem = toBeSplit.Dequeue();

			elem.Empty = !Physics.CheckBox(elem.Bounds.center, elem.Bounds.extents, Quaternion.identity, mask, QueryTriggerInteraction.Ignore);

			if (elem.Bounds.size.magnitude > doubleMinCellSize && !elem.Empty)
			{
				elem.Split();

				foreach (var child in elem.Children)
				{
					toBeSplit.Enqueue(child);
					cellCount++;
				}
			}
		}

		CalculateNeighborsRecursive(root);
	}
	
	// Update is called once per frame
	void Update ()
	{
		if (requests.Count > 0 && running.Count < maxActivePathfinds)
		{
			var newRequest = requests.Dequeue();
			ThreadPool.QueueUserWorkItem(GetPathAstar, newRequest);
			running.Add(newRequest);
		}

		if (running.Count > 0)
		{
			for (int i = running.Count-1; i >= 0; i--)
			{
				if (!running[i].isCalculating)
				{
					running.RemoveAt(i);
				}
			}
		}
	}

	private void CalculateNeighborsRecursive(OctreeElement element)
	{
		if (element.Children == null)
		{
			element.Neigbors = new OctreeElement[6][];
			for (int i = 0; i < 6; i++)
			{
				List<OctreeElement> neighbors = new List<OctreeElement>();
				GetNeighbors(element, (OctreeElement.Dir)i, neighbors);
				element.Neigbors[i] = neighbors.ToArray();
			}
		}
		else
		{
			for (int i = 0; i < element.Children.Length; i++)
			{
				CalculateNeighborsRecursive(element.Children[i]);
			}
		}
	}

	public PathRequest GetPath(Vector3 from, Vector3 to, RobotMovementController controller)
	{
		PathRequest request = new PathRequest() {from = from, to = to, isCalculating = true, controller = controller};
		requests.Enqueue(request);
		return request;
	}

	public void GetPathAstar(object context)
	{
		PathRequest request = (PathRequest)context;

		try
		{
			FastPriorityQueue<OctreeElementQueueElemenet> fronteer = new FastPriorityQueue<OctreeElementQueueElemenet>(16);
			Dictionary<OctreeElement, OctreeElement> cameFrom = new Dictionary<OctreeElement, OctreeElement>();
			Dictionary<OctreeElement, float> weights = new Dictionary<OctreeElement, float>();
			OctreeElement startNode = GetNode(request.from);
			OctreeElement endNode = GetNode(request.to);
			if (startNode == null || endNode == null) return;
			weights.Add(startNode, startNode.BaseCost);
            fronteer.Enqueue(new OctreeElementQueueElemenet(startNode), startNode.WeightedCost(request.controller.preferredFlightHeight, request.controller.minFlightHeight, request.controller.maxFlightHeight));
			OctreeElement current;
			OctreeElement closest = startNode;
			float closestDistance = Vector3.SqrMagnitude(startNode.Bounds.center - request.to);
			long lastMiliseconds = 0;
			Stopwatch stopwatch = Stopwatch.StartNew();

			while (fronteer.Count > 0)
			{
				current = fronteer.Dequeue().Element;
				if (current == endNode) break;
				//still building path
				if (current.Neigbors != null)
				{
					for (int i = 0; i < 6; i++)
					{
						for (int n = 0; n < current.Neigbors[i].Length; n++)
						{
							var next = current.Neigbors[i][n];
							if (!next.Empty && next != endNode) continue;
							float sqrDistance = Vector3.SqrMagnitude(next.Bounds.center - request.to);
							if (sqrDistance < closestDistance )
							{
								closestDistance = sqrDistance;
								closest = next;
							}
							float distance = (next.Bounds.center - request.to).sqrMagnitude;
							float newWeight = weights[current] + next.WeightedCost(request.controller.preferredFlightHeight, request.controller.minFlightHeight, request.controller.maxFlightHeight) + distance;
							if (!weights.ContainsKey(next) || newWeight < weights[next])
							{
								weights[next] = newWeight;
								cameFrom[next] = current;
								if (fronteer.MaxSize <= fronteer.Count)
								{
									fronteer.Resize(fronteer.MaxSize * 2);
								}
								fronteer.Enqueue(new OctreeElementQueueElemenet(next), newWeight + distance);
							}
						}
					}
				}

				if (maxMilisecondsPerFrame > 0 && stopwatch.ElapsedMilliseconds - lastMiliseconds > maxMilisecondsPerFrame)
				{
					lastMiliseconds = stopwatch.ElapsedMilliseconds;
					Thread.Sleep(1);
				}
			}

			current = endNode;
			bool particalPath = false;
			while (current != startNode)
			{
				if (!cameFrom.TryGetValue(current, out current))
				{
					particalPath = true;
					current = closest;
				}
				request.path.Insert(0, current.Bounds.center);
			}

			if (!particalPath)
			{
				request.path.Add(request.to);
			}

			request.isCalulated = true;
		}
		catch (Exception e)
		{
			throw e;
		}
		finally
		{
			request.isCalculating = false;
		}
	}

	private static int getNeighborStartingDepth;
	//records neighbor search traversal
	private static int[] neighborPathPositions = new int[32]; 

	private void GetNeighbors(OctreeElement startNode, OctreeElement.Dir dir,ICollection<OctreeElement> neighbors)
	{
		getNeighborStartingDepth = startNode.Depth;
		var topmostNeighbor = GetNeighborRec(startNode, dir);
		if(topmostNeighbor != null) GetAllChildrentInDir(topmostNeighbor, dir, neighbors);
	}

	private OctreeElement GetNeighborRec(OctreeElement startNode, OctreeElement.Dir dir)
	{
		OctreeElement parent = startNode.Parent;
		if (parent == null) return null;

		//find local neighbor
		int localIndex = Array.IndexOf(parent.Children, startNode);
		StorePositionAtDepth(parent.Depth, localIndex);
		int localNeighborIndex = OctreeElement.localNeighborIndex[localIndex][(int)dir];
		if (localNeighborIndex >= 0)
		{
			return parent.Children[localNeighborIndex];
		}

		OctreeElement topmostNeighbor = GetNeighborRec(parent,dir);
		//this means the edge of the octree volume so we return null
		if (topmostNeighbor == null) return null;

		//find the lowest mirrored child of the parent neighbor
		OctreeElement lowerMostReflectedChild = GetLowestChild(topmostNeighbor, dir, getNeighborStartingDepth);
		return lowerMostReflectedChild;
	}

	private OctreeElement GetLowestChild(OctreeElement start,OctreeElement.Dir dir,int maxDepth)
	{
		if (start.Children != null && start.Depth < maxDepth)
		{
			OctreeElement.Pos reflectedPos = OctreeElement.ReflectedPos[(int)dir][neighborPathPositions[start.Depth]];
			return GetLowestChild(start.Children[(int)reflectedPos], dir, maxDepth);
		}
		return start;
	}

	private void GetAllChildrentInDir(OctreeElement start, OctreeElement.Dir dir, ICollection<OctreeElement> elements )
	{
		if (start.Children != null)
		{
			var oppositeDir = (int) OctreeElement.OppositeDirs[(int) dir];
			for (int i = 0; i < OctreeElement.PosInDir[oppositeDir].Length; i++)
			{
				GetAllChildrentInDir(start.Children[(int)OctreeElement.PosInDir[oppositeDir][i]], dir, elements);
			}
		}
		else
		{
			elements.Add(start);
		}
	}

	private void StorePositionAtDepth(int depth, int pos)
	{
		if (depth >= neighborPathPositions.Length)
		{
			Array.Resize(ref neighborPathPositions, depth+1);
		}

		neighborPathPositions[depth] = pos;
	}

    internal bool Contains(Vector3 position)
    {
        return root.Bounds.Contains(position);
    }

    /// <summary>
    /// Get the smallest known node that encompasses the position.
    /// </summary>
    /// <param name="position">The position that must be containers</param>
    /// <returns>The smallest node containing the position or null if the octree does not contain the position.</returns>
	internal OctreeElement GetNode(Vector3 position)
	{
        OctreeElement node = GetNode(root, position);
#if UNITY_EDITOR
        if (node == null)
        {
            Debug.LogError("Requested a node containing a position " + position + " which is not within the Octree. This should not happen. First check that the position is within the Octree using Contains(position)");
        }
#endif
        return node;
    }

    /// <summary>
    /// Get the smallest known node that encompasses the position.
    /// If the supplied parent node has children and it encompasses the position
    /// work through the children until the one that contains the position is found.
    /// This is repeated recursively to return the smallest node possible.
    /// </summary>
    /// <param name="parent">The node to start the search within.</param>
    /// <param name="position">The position that must be containers</param>
    /// <returns>The smallest child node containing the position or null if the parent does not contain the position.</returns>
	private OctreeElement GetNode(OctreeElement parent, Vector3 position)
	{
		if (parent.Bounds.Contains(position))
		{
			if (parent.Children != null)
			{
				for (int i = 0; i < parent.Children.Length; i++)
				{
					OctreeElement child = GetNode(parent.Children[i], position);
					if (child != null) return child;
				}
			}
			else
			{
				return parent;
			}
		}
		return null;
	}

    private void OnDrawGizmos()
    {
        if (root == null || !isDebug)
        {
            return;
        }

        root.DrawGizmos();
    }

    public bool IsBuilding { get { return toBeSplit.Count > 0; } }

	public class PathRequest
	{
		internal Vector3 from;
		internal Vector3 to;
		internal List<Vector3> path;
		internal bool isCalulated;
		internal bool isCalculating;
        internal RobotMovementController controller;

		public PathRequest()
		{
			path = new List<Vector3>();
		}

		public List<Vector3> Path
		{
			get { return path; }
		}

		public void Reset()
		{
			isCalulated = false;
			isCalculating = false;
			path.Clear();
		}
	}

	public enum PathfindingAlgorith
	{
		AStar,
		Greedy
	}

	public class OctreeElementQueueElemenet : FastPriorityQueueNode
	{
		public OctreeElement Element { get; set; }

		public OctreeElementQueueElemenet(OctreeElement element)
		{
			Element = element;
		}
	}

    public class OctreeElement
    {
        public static readonly Vector3[] splitDirs = { new Vector3(1, -1, -1), new Vector3(1, -1, 1), new Vector3(1, 1, -1), new Vector3(1, 1, 1), new Vector3(-1, -1, -1), new Vector3(-1, -1, 1), new Vector3(-1, 1, -1), new Vector3(-1, 1, 1) };
        public static readonly int[][] localNeighborIndex =
        {
            new []{-1,4,2,-1,1,-1},new []{-1,5,3,-1,-1,0},new []{-1,6,-1,0,3,-1},new []{-1,7,-1,1,-1,2},new []{0,-1,6,-1,5,-1},new []{1,-1,7,-1,-1,4},
            new []{2,-1,-1,4,7,-1},new []{3,-1,-1,5,-1,6}

        };

        public static readonly Dir[] OppositeDirs = { Dir.R, Dir.L, Dir.D, Dir.U, Dir.B, Dir.F };
        //def new[]{ Pos.LBD, Pos.LFD, Pos.LBU, Pos.LFU, Pos.RBD, Pos.RFD, Pos.RBU, Pos.RFU }
        public static readonly Pos[][] ReflectedPos =
        {
            new[]{ Pos.RBD, Pos.RFD, Pos.RBU, Pos.RFU, Pos.LBD, Pos.LFD, Pos.LBU, Pos.LFU },
            new[]{ Pos.RBD, Pos.RFD, Pos.RBU, Pos.RFU, Pos.LBD, Pos.LFD, Pos.LBU, Pos.LFU },
            new[]{ Pos.LBU, Pos.LFU, Pos.LBD, Pos.LFD, Pos.RBU, Pos.RFU, Pos.RBD, Pos.RFD },
            new[]{ Pos.LBU, Pos.LFU, Pos.LBD, Pos.LFD, Pos.RBU, Pos.RFU, Pos.RBD, Pos.RFD },
            new[]{ Pos.LFD, Pos.LBD, Pos.LFU, Pos.LBU, Pos.RFD, Pos.RBD, Pos.RFU, Pos.RBU },
            new[]{ Pos.LFD, Pos.LBD, Pos.LFU, Pos.LBU, Pos.RFD, Pos.RBD, Pos.RFU, Pos.RBU }
        };

        public static readonly Pos[][] PosInDir =
        {
            new[] {Pos.LBD, Pos.LFD, Pos.LBU, Pos.LFU},
            new[] {Pos.RBD, Pos.RFD, Pos.RBU, Pos.RFU},
            new[]{ Pos.LBU, Pos.LFU, Pos.RBU, Pos.RFU },
            new[]{ Pos.LBD, Pos.LFD, Pos.RBD, Pos.RFD},
            new[]{ Pos.LFD, Pos.LFU, Pos.RFD, Pos.RFU },
            new[]{ Pos.LBD, Pos.LBU, Pos.RBD, Pos.RBU}
        };
        public Bounds Bounds;
        private float approxBoundsHeight;
        public OctreeElement[] Children;
        public OctreeElement Parent;
        public OctreeElement[][] Neigbors;
        public int Depth;
        public bool Empty;

        private float _cost = 1;
        /// <summary>
        /// Cost of movement through this cell, ignoring the preferences of any particular agent.
        /// </summary>
        public float BaseCost
        {
            get { return _cost; }
        }

        /// <summary>
        /// Cost of movement through this cell, taking into account the preferences of an agent.
        /// </summary>
        /// <param name="preferredHeight"></param>
        /// <param name="minHeight"></param>
        /// <param name="maxHeight"></param>
        /// <returns></returns>
        public float WeightedCost(float preferredHeight, float minHeight, float maxHeight)
        {
            // I think this weighting is breaking things, too many nodes were being added to the fronteer
            //float weight = Math.Abs(((approxBoundsHeight - preferredHeight) / (maxHeight - minHeight)));
            //return BaseCost * weight;
            return BaseCost;
        }


        public OctreeElement(OctreeElement parent,Bounds bounds,int depth)
		{
			Parent = parent;
			Bounds = bounds;
            // TODO Raytrace down to find surface below, there may be a building or tree or similar here.
            approxBoundsHeight = Bounds.center.y + Terrain.activeTerrain.SampleHeight(Bounds.center);
            Depth = depth;
		}

		public void Split()
		{
			Children = new OctreeElement[splitDirs.Length];
			for (int i = 0; i < Children.Length; i++)
			{
				Children[i] = new OctreeElement(this,new Bounds(Bounds.center + Vector3.Scale(splitDirs[i],Bounds.extents/2f),Bounds.extents),Depth+1);
			}
		}

		public enum Dir
		{
			L,R,U,D,F,B
		}

		public enum Pos
		{
			LBD,LFD,LBU,LFU,RBD,RFD,RBU,RFU
		}

        internal void DrawGizmos()
        {
            if (Empty)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawWireCube(Bounds.center, Bounds.size);
            }
            else
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireCube(Bounds.center, Bounds.size);
            }
            if (Children != null)
            {
                foreach (OctreeElement child in Children)
                {
                    child.DrawGizmos();
                }
            }
        }
    }
}
