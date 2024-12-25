using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class AgentProps
{
    // The position of the entity.
    private Vector3 _position;
    public Vector3 Position
    {
        get { return _position; }
        set { _position = value; }
    }
    // The orientation of the entity.
    private float _orientation;
    public float Orientation
    {
        get { return _orientation; }
        set { _orientation = value; }
    }
    // The velocity of the entity.
    private Vector3 _velocity;
    public Vector3 Velocity
    {
        get { return _velocity; }
        set { _velocity = value; }
    }
    // The rotation of the entity.
    private float _rotation;
    public float Rotation
    {
        get { return _rotation; }
        set { _rotation = value; }
    }
    // Constructor.
    public AgentProps()
    {
    }
    // Get the orientation as a vector.
    public Vector3 GetOrientationAsVector()
    {
        Vector3 velocityFromOrientation = new Vector3();
        velocityFromOrientation.x = Mathf.Cos(this.Orientation);
        velocityFromOrientation.y = 0;
        velocityFromOrientation.z = Mathf.Sin(this.Orientation);
        return velocityFromOrientation;
    }
}