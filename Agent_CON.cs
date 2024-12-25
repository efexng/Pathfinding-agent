using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Agent_CON : MonoBehaviour
{
    // Movement algorithm class.
    MovementAlgorithms movAlg = new MovementAlgorithms();
    // The NPCs properties.
    AgentProps myProps = new AgentProps();
    // The target properties.
    AgentProps TargetProps = new AgentProps();
    // KinematicOutput
    KinematicOutput KOutput = new KinematicOutput();
    // SteeringOutput
    SteeringOutput SOutput = new SteeringOutput();
    private Animator anim;
    private Rigidbody rb;
    private GameObject TargetGO;
    [SerializeField]
    private string TargetGOTag = "";
    [SerializeField]
    private float OrientationOffset = 0;
    [SerializeField]
    private float avoidDistance = 14;
    // Start is called before the first frame update
    void Start()
    {
        anim = GetComponent<Animator>();
        rb = GetComponent<Rigidbody>();
        // Find a game object in the scene with tag.
        if (TargetGOTag.Length > 0)
        {
            TargetGO = GameObject.FindGameObjectWithTag(TargetGOTag);
            if (TargetGO == null)
            {
                Debug.Log("No target gameobject found for " +
                gameObject.name + ". Looking for: " + TargetGOTag + ".");
            }
        }
        else
        {
            Debug.Log("No target tag set for " + gameObject.name + ".");
        }
        // Set the agents initial movement properties.
        myProps.Position = transform.position;
        // Set the target props.
        TargetProps.Position = TargetGO.transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        // Update the AI character.
        // ------------------------
      // Steering - Movement algorithms.
//movAlg.SteeringSeek_1(myProps, TargetProps, ref SOutput);
movAlg.SteeringObstacleAvoidance(myProps, TargetProps,
 ref SOutput, 3, 8, 6, 0.5f, 0, transform.gameObject);
// We will align to the direction we are moving.
//TargetProps.Orientation = movAlg.GetOrientation(myProps.Velocity);
//movAlg.SteeringAlign_1(myProps, TargetProps, ref SOutput);
// Update the characters
//movAlg.UpdateSteering(ref myProps, SOutput, 5.0f, Time.deltaTime);

        if (SOutput.NoSteering)
        {
            // Keep the current position when no steering.
            myProps.Position = transform.position;
            myProps.Velocity = new Vector3(0, 0, 0);
            myProps.Orientation =
            ((Mathf.Deg2Rad * (-transform.rotation.eulerAngles.y + OrientationOffset)));
            SOutput.NoSteering = false;
        }
        // Kinematic - Movement algorithms.
         movAlg.KinematicWander(myProps, ref KOutput);
        // Update the characters
         movAlg.UpdateKinematic(ref myProps, KOutput, Time.deltaTime);
        //:~END: AI Update.
        // Update the animation.
        if (anim)
        {
            if (myProps.Velocity.sqrMagnitude > 0)
            {
                anim.SetInteger("Walk", 1);
            }
            else
            {
                anim.SetInteger("Walk", 0);
            }
        }
        // Next, apply the AI update to the agent.
        transform.position = myProps.Position;
        // Set the agent orientation.
        Quaternion Orientation = new Quaternion();
        Orientation.eulerAngles = new
        Vector3(0, -((Mathf.Rad2Deg * myProps.Orientation) - OrientationOffset), 0);
        transform.rotation = Orientation;
    }

    private void FixedUpdate()
    {
        // If we would like our algorithms to work with phyics we can use move position.
        //if(rb)
        //{
        // rb.MovePosition(rb.position + myProps.Velocity * Time.deltaTime);
        //}
        // We also need to make sure "myProps" keeps an accurate record of position because we
        // are using Unity to update position and not setting it ourselves.
        // The Unity update will be different.
        // myProps.Position = transform.position;
    }
    void OnDrawGizmos()
    {
        // Draw a yellow sphere at the obstacle avoidance target position.
        if (movAlg.ObstacleAvoidanceTargetSet)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(movAlg.ObstacleAvoidanceTarget.Position, 0.1f);
        }
    }
}