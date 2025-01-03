using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class AStarManager
{
    // The a star algorithm.
    private AStar AStar = new AStar();
    // The waypoint graph.
    private Graph aGraph = new Graph();
    // The Heuristic.
    private Heuristic aHeuristic = new Heuristic();
    public AStarManager()
    {
    }
    // Add Connection.
    public void AddConnection(Connection connection)
    {
        aGraph.AddConnection(connection);
    }
    // Find path.
    public List<Connection> PathfindAStar(GameObject start, GameObject end)
    {
        return AStar.PathfindAStar(aGraph, start, end, aHeuristic);
    }
    // Remove all connections associated with a given node.
public void RemoveConnectionsFromNode(GameObject node)
{
    aGraph.RemoveConnectionsFromNode(node);
}

}