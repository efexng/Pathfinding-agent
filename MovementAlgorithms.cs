using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class MovementAlgorithms
{
    // Stores a target when obstacle avoidance is
    // being implemented. A temporary target.
    private AgentProps _obstacleAvoidanceTarget = new AgentProps();
    public AgentProps ObstacleAvoidanceTarget
    {
        get { return _obstacleAvoidanceTarget; }
    }
    // True if obstacle avoidance is being implemented.
    private bool _obstacleAvoidanceTargetSet = false;
    public bool ObstacleAvoidanceTargetSet
    {
        get { return _obstacleAvoidanceTargetSet; }
    }
    // Constructor.
    public MovementAlgorithms()
    {
    }
    // Get Orientation.
    public float GetOrientation(Vector3 velocity)
    {
        float currentOrientation = 0;
        float magnitude = velocity.magnitude;
        if (magnitude > 0)
        {
            // Calculate the orientation using an arc tangent of
            // the velocity components.
            currentOrientation = Mathf.Atan2(velocity.z, velocity.x);
        }
        return currentOrientation;
    }
    // Get distance between two points.
    public float GetDistance(AgentProps aAProps, AgentProps aTargetAProps)
    {
        // Main seek code. First get a vector in the direction of the target.
        Vector3 tmpVel = new Vector3();
        tmpVel = aTargetAProps.Position - aAProps.Position;
        // Find the magnitude / length of the velocity vector.
        return tmpVel.magnitude;
    }

    // ########################
    // Steering Algorithms.
    // ########################
    // Update the position and orientation of an entity.
    public void UpdateSteering(ref AgentProps aAProps,
 SteeringOutput aSteeringOpt, float maxSpeed, float deltaTime)
    {
        // Update the velocity and rotation.
        aAProps.Velocity += aSteeringOpt.AccelerationLinear * deltaTime;
        aAProps.Rotation += aSteeringOpt.AccelerationAngular * deltaTime;
        // Check if speeding and clip.
        if (aAProps.Velocity.magnitude > maxSpeed)
        {
            aAProps.Velocity = aAProps.Velocity.normalized;
            aAProps.Velocity *= maxSpeed;
        }
        // Update the position and orientation.
        aAProps.Position += aAProps.Velocity * deltaTime;
        aAProps.Orientation += aAProps.Rotation * deltaTime;
    }
    // Steering Seek Algorithm.
    public void SteeringSeek_1(AgentProps aAProps,
 AgentProps aTargetAProps, ref SteeringOutput SOutput)
    {
        // max Acceleration
        float maxAcceleration = 1.0f;
        // The max speed of a character.
        float maxSpeed = 2.0f;
        // Target Radius of Satisfaction
        float targetRadius = 0.1f;
        // The slowing Radius of Satisfaction
        float slowingRadius = 1.0f;
        // The time until the target.
        float timeToTarget = 0.1f;
        // Main seek code. First get a vector in the direction of the target.
        Vector3 direction = new Vector3();
        direction = aTargetAProps.Position - aAProps.Position;
        // Are we within the target radius?
        // Find the magnitude / length of the velocity vector.
        float magnitude = direction.magnitude;
        if (magnitude < targetRadius)
        {
            SOutput.AccelerationLinear = new Vector3(0, 0, 0);
            SOutput.NoSteering = true;
            return;
        }
        // Are we outside the slowing radius?
        float targetSpeed;
        if (magnitude > slowingRadius)
        {
            targetSpeed = maxSpeed;
        }
        else
        {
            // Calculate a scaled speed.
            targetSpeed = maxSpeed * magnitude / slowingRadius;
        }
        // Normalize the velocity.
        Vector3 targetVelocity = new Vector3();
        targetVelocity = direction;
        targetVelocity /= magnitude;
        targetVelocity *= targetSpeed;
        // Acceleration tries to get to the target velocity.
        targetVelocity = targetVelocity - aAProps.Velocity;
        targetVelocity = targetVelocity / timeToTarget;
        // Is this too fast for the max acceleration?
        magnitude = targetVelocity.magnitude;
        if (magnitude > maxAcceleration)
        {
            targetVelocity = targetVelocity / magnitude;
            targetVelocity = targetVelocity * maxAcceleration;
        }
        SOutput.AccelerationLinear = targetVelocity;
        SOutput.NoSteering = false;
    }
    // Align Algorithm.
    // Align Algorithm.
    public void SteeringAlign_1(AgentProps aAProps,
    AgentProps aTargetAProps, ref SteeringOutput SOutput)
    {
        // max Acceleration
        float maxAngularAcceleration = 2.0f;
        // The max Rotation of a character.
        float maxRotation = 2.0f;
        // Target Radius of Satisfaction
        float targetRadius = 0.05f;
        // The slowing Radius of Satisfaction
        float slowingRadius = 0.5f;
        // The time until the target.
        float timeToTarget = 0.1f;
        // Get the naive direction to the target.
        float rotation = aTargetAProps.Orientation - aAProps.Orientation;
        // Map the result to -pi, pi interval.
        if (rotation > Mathf.PI)
        {
            rotation = -((2 * Mathf.PI) - rotation);
        }
        float rotationSize = Mathf.Abs(rotation);
        // Check if no steering needed.
        if (rotationSize < targetRadius)
        {
            SOutput.AccelerationAngular = 0;
            return;
        }
        // If we are outside the slow radius, then use the max rotation.
        float targetRotation;
        if (rotationSize > slowingRadius)
        {
            targetRotation = maxRotation;
        }
        else
        {
            targetRotation = maxRotation * rotationSize / slowingRadius;
        }
        // The target rotation combines speed and direction.
        targetRotation = targetRotation * rotation / rotationSize;
        // Acceleration tries to get to the target velocity.
        SOutput.AccelerationAngular = (targetRotation - aAProps.Rotation);
        SOutput.AccelerationAngular /= timeToTarget;
        // Is this too fast for the max acceleration?
        float tmpAccelerationAngular = Mathf.Abs(SOutput.AccelerationAngular);
        if (tmpAccelerationAngular > maxAngularAcceleration)
        {
            SOutput.AccelerationAngular = SOutput.AccelerationAngular / tmpAccelerationAngular;
            SOutput.AccelerationAngular = SOutput.AccelerationAngular * maxAngularAcceleration;
        }
    }

    // Steering Obstacle Avoidance.
    // Steering Obstacle Avoidance.
    public void SteeringObstacleAvoidance(AgentProps aAProps,
    AgentProps aTargetAProps, ref SteeringOutput SOutput,
    float avoidDistance, float lookAhead, float lookAheadWhiskers,
    float lookAheadHeightOffset, float lookAheadForwardOffset,
    GameObject go)
    {
        // Calculate enitity position.
        Vector3 EntityPosition = aAProps.Position;
        EntityPosition.y += lookAheadHeightOffset;
        // Calculate the forward collision ray vector.
        RaycastHit hitForward;
        bool bHitForward = Physics.Raycast(EntityPosition,
        go.transform.TransformDirection(Vector3.forward), out hitForward, lookAhead);
        // Calculate the right forward collision ray vector.
        Quaternion spreadAngle = Quaternion.AngleAxis(15, new Vector3(0, 1, 0));
        Vector3 newRightVector = spreadAngle * go.transform.TransformDirection(Vector3.forward);
        RaycastHit hitForwardRight;
        bool bHitForwardRight = Physics.Raycast(EntityPosition,
        newRightVector, out hitForwardRight, lookAheadWhiskers);
        // Calculate the left forward collision ray vector.
        spreadAngle = Quaternion.AngleAxis(-15, new Vector3(0, 1, 0));
        Vector3 newLeftVector = spreadAngle * go.transform.TransformDirection(Vector3.forward);
        RaycastHit hitForwardLeft;
        bool bHitForwardLeft = Physics.Raycast(EntityPosition,
        newLeftVector, out hitForwardLeft, lookAheadWhiskers);
        if (bHitForward)
            Debug.DrawRay(EntityPosition,
            go.transform.TransformDirection(Vector3.forward) * hitForward.distance, Color.red);
        else
            Debug.DrawRay(EntityPosition,
            go.transform.TransformDirection(Vector3.forward) * lookAhead, Color.yellow);
        if (bHitForwardRight)
            Debug.DrawRay(EntityPosition, newRightVector * hitForwardRight.distance, Color.red);
        else
            Debug.DrawRay(EntityPosition, newRightVector * lookAheadWhiskers, Color.yellow);
        if (bHitForwardLeft)
            Debug.DrawRay(EntityPosition, newLeftVector * hitForwardLeft.distance, Color.red);
        else
            Debug.DrawRay(EntityPosition, newLeftVector * lookAheadWhiskers, Color.yellow);

        // Check if hit forward target.
        Vector3 ForwardAvoidanceTarget = new Vector3(0, 0, 0);
        if (bHitForward)
        {
            // Calculate new target.
            Vector3 newTarget = hitForward.point + (hitForward.normal * avoidDistance);
            newTarget.y = aTargetAProps.Position.y;
            ForwardAvoidanceTarget = newTarget;
        }
        // Check if Whiskers hit a target - right.
        Vector3 RightAvoidanceTarget = new Vector3(0, 0, 0);
        if (bHitForwardRight)
        {
            // Calculate new target.
            Vector3 newTarget = hitForwardRight.point + (hitForwardRight.normal * avoidDistance);
            newTarget.y = aTargetAProps.Position.y;
            RightAvoidanceTarget = newTarget;
        }
        // Check if Whiskers hit a target - right.
        Vector3 LeftAvoidanceTarget = new Vector3(0, 0, 0);
        if (bHitForwardLeft)
        {
            // Calculate new target.
            Vector3 newTarget = hitForwardLeft.point + (hitForwardLeft.normal * avoidDistance);
            newTarget.y = aTargetAProps.Position.y;
            LeftAvoidanceTarget = newTarget;
        }

        // Obstacle Avoidance Algorithm.
        if (bHitForward || bHitForwardRight ||
        bHitForwardLeft || ObstacleAvoidanceTargetSet == true)
        {
            if (bHitForward)
            {
                ObstacleAvoidanceTarget.Position = ForwardAvoidanceTarget;
                _obstacleAvoidanceTargetSet = true;
            }
            else
            {
                if (bHitForwardRight && ObstacleAvoidanceTargetSet == false)
                {
                    ObstacleAvoidanceTarget.Position = RightAvoidanceTarget;
                    _obstacleAvoidanceTargetSet = true;
                }
                else if (bHitForwardLeft && ObstacleAvoidanceTargetSet == false)
                {
                    ObstacleAvoidanceTarget.Position = LeftAvoidanceTarget;
                    _obstacleAvoidanceTargetSet = true;
                }
            }
            ObstacleAvoidanceTarget.Orientation = aTargetAProps.Orientation;
            ObstacleAvoidanceTarget.Rotation = aTargetAProps.Rotation;
            Vector3 vecToTarget = ObstacleAvoidanceTarget.Position - aAProps.Position;
            float distanceToTarget = vecToTarget.magnitude;
            if (distanceToTarget < 0.5f)
            {
                _obstacleAvoidanceTargetSet = false;
                Debug.Log("Close to ObstacleAvoidance target - Target cleared");
            }
            SteeringSeek_1(aAProps, ObstacleAvoidanceTarget, ref SOutput);
        }
        else
        {
            // No obstacle avoidance - Call Seek.
            SteeringSeek_1(aAProps, aTargetAProps, ref SOutput);
        }
    }

    // ########################
    // Kinematic Algorithms.
    // ########################
    // Update the position and orientation of an entity.
    // Update the position and orientation of an entity.
    public void UpdateKinematic(ref AgentProps aAProps,
    KinematicOutput KOutput, float deltaTime)
    {
        // Update the velocity and rotation.
        aAProps.Velocity = KOutput.Velocity;
        aAProps.Rotation = KOutput.Rotation;
        // Update the position and orientation.
        aAProps.Position += (aAProps.Velocity * deltaTime);
        aAProps.Orientation += aAProps.Rotation * deltaTime;
    }

    // Kinematic Wander.
    // Kinematic Wander.
    public void KinematicWander(AgentProps aAProps, ref KinematicOutput KOutput)
    {
        // The max speed of a character.
        float maxSpeed = 2.0f;
        // The max rotation.
        float maxRotation = 6.0f;
        // Get the orientation of the character as a vector.
        Vector3 velocityFromOrientation = aAProps.GetOrientationAsVector();
        // Limit to max speed.
        velocityFromOrientation *= maxSpeed;
        // Store the output velocity.
        KOutput.Velocity = velocityFromOrientation;
        // Set the rotation.
        float rand1 = Random.Range(0.0f, 1.0f);
        float rand2 = Random.Range(0.0f, 1.0f);
        KOutput.Rotation = (rand1 - rand2) * maxRotation;
    }

}