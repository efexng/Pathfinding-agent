using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MasterController : MonoBehaviour
{
    // References for Pathfinding and Movement
    private AStarManager AStarManager = new AStarManager();
    private MovementAlgorithms movAlg = new MovementAlgorithms();
    
    // Waypoint and Path Variables
    private List<GameObject> Waypoints = new List<GameObject>();
    private List<Connection> AStarPath = new List<Connection>();
    private List<GameObject> WaypointGoals = new List<GameObject>();
    private int currentTargetArrayIndex = 0;
    private Vector3 currentTargetPos;

    // Agent and Obstacle Avoidance Variables
    private AgentProps myProps = new AgentProps();
    private SteeringOutput SOutput = new SteeringOutput();
    private GameObject TargetGO;

    // Configuration
    [SerializeField] private GameObject start;
    [SerializeField] private GameObject end;
    [SerializeField] private float currentSpeed = 8;
    [SerializeField] private float obstacleDetectionRange = 3;
    [SerializeField] private float obstacleAvoidanceStrength = 8;
    private Vector3 OffSet = new Vector3(0, 0.3f, 0);

    // Runtime Variables
    private bool agentMove = true;
    private float timer = 0;
    private float totalDistance = 0;
    private Vector3 lastPosition = new Vector3();
    private HashSet<GameObject> disabledWaypoints = new HashSet<GameObject>();
    private GameObject originalStart;
    private bool isPathCompleted = false;

    // Cooldown for obstacle avoidance
    private float obstacleCooldown = 1f; // Cooldown in seconds
    private float obstacleCooldownTimer = 0f;

private GameObject lastValidWaypoint;

void Start()
{
    // Validate start and end
    if (!ValidateStartAndEnd()) return;

    // Store the original start node
    originalStart = start;
    lastValidWaypoint = start; // Initialize last valid waypoint to the start node

    // Load all waypoints in the scene
    LoadWaypoints();

    // Find initial path
    FindPath(start, end);

    lastPosition = transform.position;
}

    void Update()
    {
        if (agentMove && AStarPath.Count > 0 && !isPathCompleted)
        {
            HandleMovementAndPathing();
        }

        // Reduce cooldown timer
        if (obstacleCooldownTimer > 0)
        {
            obstacleCooldownTimer -= Time.deltaTime;
        }
    }

    private bool ValidateStartAndEnd()
    {
        if (start == null || end == null)
        {
            Debug.LogError("Start or End is not assigned.");
            return false;
        }

        if (start.GetComponent<VisGraphWaypointManager>() == null || 
            end.GetComponent<VisGraphWaypointManager>() == null)
        {
            Debug.LogError("Start or End is not a valid waypoint.");
            return false;
        }

        return true;
    }

    private void LoadWaypoints()
    {
        GameObject[] waypointObjects = GameObject.FindGameObjectsWithTag("Waypoint");
        foreach (GameObject waypoint in waypointObjects)
        {
            VisGraphWaypointManager wpManager = waypoint.GetComponent<VisGraphWaypointManager>();
            if (wpManager)
            {
                Waypoints.Add(waypoint);
                if (wpManager.WaypointType == VisGraphWaypointManager.waypointPropsList.Goal)
                {
                    WaypointGoals.Add(waypoint);
                }

                foreach (VisGraphConnection connection in wpManager.Connections)
                {
                    if (connection.ToNode != null)
                    {
                        Connection conn = new Connection
                        {
                            FromNode = waypoint,
                            ToNode = connection.ToNode
                        };
                        AStarManager.AddConnection(conn);
                    }
                }
            }
        }
    }

    private void FindPath(GameObject startNode, GameObject endNode)
    {
        AStarPath = AStarManager.PathfindAStar(startNode, endNode);

        if (AStarPath.Count == 0)
        {
            Debug.LogWarning("A* did not find a valid path.");
        }
        else
        {
            currentTargetArrayIndex = 0; // Reset to the first target
        }
    }

    private GameObject FindNearestWaypoint(Vector3 position)
    {
        GameObject nearestWaypoint = null;
        float shortestDistance = float.MaxValue;

        foreach (GameObject waypoint in Waypoints)
        {
            if (!disabledWaypoints.Contains(waypoint))
            {
                float distance = Vector3.Distance(position, waypoint.transform.position);
                if (distance < shortestDistance)
                {
                    shortestDistance = distance;
                    nearestWaypoint = waypoint;
                }
            }
        }

        if (nearestWaypoint == null)
        {
            Debug.LogError("No nearest waypoint found. Pathfinding may fail.");
        }

        return nearestWaypoint;
    }

    private void DisableWaypoint(GameObject waypoint)
    {
        if (waypoint == null || disabledWaypoints.Contains(waypoint))
            return;

        Debug.Log($"Disabling waypoint: {waypoint.name}");
        disabledWaypoints.Add(waypoint);
        Waypoints.Remove(waypoint);

        // Remove connections related to the waypoint
        AStarManager.RemoveConnectionsFromNode(waypoint);
    }

   private void HandleMovementAndPathing()
{
    // Distance Tracking
    Vector3 displacement = transform.position - lastPosition;
    totalDistance += displacement.magnitude;
    lastPosition = transform.position;
    timer += Time.deltaTime;

    if (currentTargetArrayIndex < AStarPath.Count)
    {
        currentTargetPos = AStarPath[currentTargetArrayIndex].ToNode.transform.position;
        currentTargetPos.y = transform.position.y;

        Vector3 direction = currentTargetPos - transform.position;
        float distance = direction.magnitude;

        // Face direction and move
        if (direction.magnitude > 0)
        {
            Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);
            transform.rotation = rotation;
        }

        Vector3 normDirection = direction.normalized;
        transform.position += normDirection * currentSpeed * Time.deltaTime;

        // Check if near the target
        if (distance < 1)
        {
            lastValidWaypoint = AStarPath[currentTargetArrayIndex].ToNode; // Update last valid waypoint
            currentTargetArrayIndex++;

            if (currentTargetArrayIndex == AStarPath.Count)
            {
                Debug.Log($"Path completed. Time: {timer}, Distance: {totalDistance}");
                totalDistance = 0;
                timer = 0;

                if (end == originalStart)
                {
                    isPathCompleted = true;
                    Debug.Log("Agent has returned to the original node. Stopping movement.");
                }
                else
                {
                    GameObject temp = start;
                    start = end;
                    end = temp;

                    Debug.Log($"Reversing path. New start: {start.name}, New end: {end.name}");
                    FindPath(start, end);
                }
            }
        }
    }

    // Obstacle Avoidance
    HandleObstacleAvoidance();
}

private void HandleObstacleAvoidance()
{
    if (obstacleCooldownTimer > 0)
        return;

    Collider[] obstacles = Physics.OverlapSphere(transform.position, obstacleDetectionRange);
    foreach (Collider obstacle in obstacles)
    {
        if (obstacle.CompareTag("Obstacle"))
        {
            Debug.Log($"Obstacle detected at {obstacle.name}. Recalculating path...");

            GameObject nearestWaypoint = FindNearestWaypoint(transform.position);
            if (nearestWaypoint != null)
            {
                DisableWaypoint(AStarPath[currentTargetArrayIndex].ToNode);

                // Recalculate path from last valid waypoint
                FindPath(lastValidWaypoint, end);
            }
            else 
            {
                Debug.LogError("No valid waypoint found near the agent's current position!");
            }

            obstacleCooldownTimer = obstacleCooldown;
            break; // Only handle one obstacle per frame
        }
    }
}

    void OnDrawGizmos()
    {
        if (AStarPath != null)
        {
            foreach (Connection connection in AStarPath)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(connection.FromNode.transform.position + OffSet,
                                connection.ToNode.transform.position + OffSet);
            }
        }
    }
}
