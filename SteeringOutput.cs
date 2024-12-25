using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class SteeringOutput
{
    // The acceleration of the game character (e.g. x, y, z).
    public Vector3 _accelerationLinear;
    public Vector3 AccelerationLinear
    {
        get { return _accelerationLinear; }
        set { _accelerationLinear = value; }
    }
    // The acceleration of the rotation.
    public float _accelerationAngular;
    public float AccelerationAngular
    {
        get { return _accelerationAngular; }
        set { _accelerationAngular = value; }
    }
    // Flag for NO steering. true = no steering and false = steering exists.
    public bool _NoSteering = false;
    public bool NoSteering
    {
        get { return _NoSteering; }
        set { _NoSteering = value; }
    }
}
