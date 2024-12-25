using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class VanCon : MonoBehaviour
{
    // An array of gameobjects (gos).
    private GameObject[] gos;
    // Closest.
    private GameObject closest = null;
    // Speed variables.
    private const float MAX_SPEED = 5;
    private float currentSpeed = MAX_SPEED;
    private bool slowedDown = false;
    private const float CLOSE_DISTANCE = 10;
    // Start is called before the first frame update
    void Start()
    {
        gos = GameObject.FindGameObjectsWithTag("Coin");
        // Find closest Coin.
        closest = FindClosest();
        Debug.Log("Closest Coin: " + closest.name);
        // Display direction to Coin.
        Vector3 direction = closest.transform.position - transform.position;
        // Determine the distance of the vector.
        float distance = direction.magnitude;
        Debug.Log("Closest Coin distance: " + distance);
    }
    // Update is called once per frame
    // Update is called once per frame
    void Update()
    {
        // Find a Coin if one does not exist.
        if (closest == null)
        {
            closest = FindClosest();
            slowedDown = false;
            currentSpeed = MAX_SPEED;
            if (closest == null)
            {
                return;
            }
        }
        // Determine the direction to the closest Coin.
        Vector3 direction = closest.transform.position - transform.position;
        // Calculates the length of the relative position vector
        float distance = direction.magnitude;
        // Face in the right direction.
        direction.y = 0;
        if (direction.magnitude > 0)
        {
            Quaternion rotation = Quaternion.LookRotation(-direction, Vector3.up);
            transform.rotation = rotation;
        }
        // Calculate the normalised direction to the target from a game object.
        Vector3 normDirection = direction / distance;
        // Adjust speed based on distance.
        if (distance < CLOSE_DISTANCE && slowedDown == false)
        {
            currentSpeed *= 0.5f;
            slowedDown = true;
        }
        // Move the game object.
        transform.position = transform.position + normDirection * currentSpeed * Time.deltaTime;
    }

    private GameObject FindClosest()
    {
        GameObject closest = null;
        float distanceSqr = Mathf.Infinity;
        foreach (GameObject go in gos)
        {
            if (go != null)
            {
                // Get a vector to the gameobject.
                Vector3 direction = go.transform.position - transform.position;
                // Determine the distance squared of the vector.
                float tmpDistanceSqr = direction.sqrMagnitude;
                if (tmpDistanceSqr < distanceSqr)
                {
                    closest = go;
                    distanceSqr = tmpDistanceSqr;
                }
            }
        }
        return closest;
    }
}
