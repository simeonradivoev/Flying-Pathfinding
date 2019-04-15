using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerLookTarget : MonoBehaviour {

	[SerializeField] private Transform target;
	[SerializeField] private float maxPointDistance = 8;
	[SerializeField] private LayerMask pointLayerMask = -1;
	[SerializeField] private float distnaceFromSurface = 1;
	[SerializeField] private float velocityDirectionContribution = 0.5f;
	[SerializeField] private float maxDistanceToTrigger = 4;
	private Vector3 lastPos;
	private float lastDistanceTravelled;
	private CharacterController characterController;
	private float distanceTravelled;
	private bool isWalking;
	private int randomDir;

	// Use this for initialization
	void Start ()
	{
		characterController = GetComponent<CharacterController>();
	}
	
	// Update is called once per frame
	void Update ()
	{
		distanceTravelled += (transform.position - lastPos).magnitude;

		if (Mathf.Abs(distanceTravelled - lastDistanceTravelled) > maxDistanceToTrigger)
		{
			if (!isWalking)
			{
				randomDir = (int)Mathf.Sign(Random.Range(-1f, 1f));
				isWalking = true;
			}

			lastDistanceTravelled = distanceTravelled;
			Vector3 velocity = Vector3.Normalize(transform.TransformVector(characterController.velocity * randomDir).normalized * velocityDirectionContribution + transform.forward);
			RaycastHit hit;
			if (Physics.Raycast(new Ray(transform.position, velocity), out hit, maxPointDistance, pointLayerMask))
			{
				target.position = hit.point + hit.normal * distnaceFromSurface;
			}
			else
			{
				target.position = transform.position + velocity * maxPointDistance;
			}
		}
		else
		{
			isWalking = false;
		}

		lastPos = transform.position;
	}
}
