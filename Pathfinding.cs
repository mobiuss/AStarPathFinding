using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System;

public class Pathfinding : MonoBehaviour
{
	Grid grid;
	PathRequestManager requestManager;

    void Awake()
    {
		requestManager = GetComponent<PathRequestManager> ();
		grid = GetComponent<Grid> ();
    }

	public void StartFindPath(Vector3 startPos, Vector3 targetPos){
		StartCoroutine(FindPath(startPos, targetPos));
	}

    IEnumerator FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Stopwatch sw = new Stopwatch();
        sw.Start(); // For counting how long it takes to make path

		Vector3[] waypoints = new Vector3[0];
		bool pathSuccess = false;

        Node startNode = grid.NodeFromWorldPoint(startPos); // get node position on grid
        Node targetNode = grid.NodeFromWorldPoint(targetPos); // get node position on grid
		
		if (startNode.walkable && targetNode.walkable) {

			Heap<Node> openSet = new Heap<Node> (grid.maxSize); // set the node heap = height x width of grid
			HashSet<Node> closedSet = new HashSet<Node> (); // nodes already checked
			openSet.Add (startNode); //add start node to open set

			while (openSet.Count > 0) {
				Node currentNode = openSet.RemoveFirst (); // remove the first in open set and add it to closed set
				closedSet.Add (currentNode); 

				if (currentNode == targetNode) { //if the current node being checked = target node then path is found
					sw.Stop (); // stop stopwatch
					print ("Path Found" + sw.ElapsedMilliseconds + "ms"); // print time
					pathSuccess = true;
					break;
				}

				foreach (Node neighbour in grid.GetNeighbours(currentNode)) {
					if (!neighbour.walkable || closedSet.Contains (neighbour)) {
						continue;
					}

					int newMovementCostToNeighbour = currentNode.gCost + GetDistance (currentNode, neighbour);
					if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains (neighbour)) {
						neighbour.gCost = newMovementCostToNeighbour;
						neighbour.hCost = GetDistance (neighbour, targetNode);
						neighbour.parent = currentNode;

						if (!openSet.Contains (neighbour))
							openSet.Add (neighbour);
						else
							openSet.UpdateItem(neighbour);
					}
				}
			}
			yield return null;
			if (pathSuccess) {
				waypoints = RetracePath (startNode, targetNode);
			}
			requestManager.FinishedProcessingPath (waypoints, pathSuccess);
		}
    }

    Vector3[] RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
			Vector3[] waypoints = SimplifyPath (path);
			Array.Reverse (waypoints);
			return waypoints;
    }

	Vector3[] SimplifyPath(List<Node> path){
		List<Vector3> waypoints = new List<Vector3> ();
		Vector2 directionOld = Vector2.zero;
			for (int i = 1; i< path.Count; i++) {
				Vector2 directionNew = new Vector2(path[i-1].gridX - path[i].gridX, path[i-1].gridY - path[i].gridY);
				if(directionNew != directionOld){
					waypoints.Add(path[i].worldPosition);
			}
				directionOld = directionNew;
		}
			return waypoints.ToArray ();
	}

    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        	return 14 * dstX + 10 * (dstY - dstX);
    }
}
