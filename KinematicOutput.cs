using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class KinematicOutput
{
    // The velocity.
    public Vector3 _velocity;
    public Vector3 Velocity
    {
        get { return _velocity; }
        set { _velocity = value; }
    }
    // The rotation.
    public float _rotation;
    public float Rotation
    {
        get { return _rotation; }
        set { _rotation = value; }
    }
}
