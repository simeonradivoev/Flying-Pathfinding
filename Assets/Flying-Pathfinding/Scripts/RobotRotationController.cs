using System;
using UnityEngine;

public class RobotRotationController : MonoBehaviour
{
	private readonly VectorPid angularVelocityController = new VectorPid(33.7766f, 0, 0.2553191f);
	[SerializeField] private VectorPid headingController = new VectorPid(9.244681f, 0, 0.06382979f);

    [Tooltip("If the agent is to look at the player when not moving to an object then set this field. If this is null then the agent will not change it's direction of focus if not moving.")]
	public Transform playerHead;
	[SerializeField] private bool physicsRotation;
	[SerializeField, Header("Animated Rotation")] public float rotationEasing = 1f;

	new private Rigidbody rigidbody;
	private RobotMovementController movementController;

	void Start()
	{
		rigidbody = GetComponent<Rigidbody>();
		movementController = GetComponent<RobotMovementController>();
	}

	private void Update()
	{
		if(!physicsRotation)
		{
			rigidbody.MoveRotation(Quaternion.Lerp(rigidbody.rotation, Quaternion.LookRotation(LookAtDir, Vector3.up), Time.deltaTime * rotationEasing));
		}
	}

	private void FixedUpdate()
	{
		if (physicsRotation)
		{
			Vector3 angularVelocityError = rigidbody.angularVelocity * -1f;
			Vector3 angularVelocityCorrection = angularVelocityController.Update(angularVelocityError, Time.deltaTime);
			rigidbody.AddTorque(angularVelocityCorrection, ForceMode.Acceleration);

			//forward heading correction
			Vector3 desiredHeading = LookAtDir;
			Vector3 currentHeading = transform.forward;
			Vector3 headingError = Vector3.Cross(currentHeading, desiredHeading);
			Vector3 headingCorrection = headingController.Update(headingError, Time.deltaTime);
			rigidbody.AddTorque(headingCorrection, ForceMode.Acceleration);

			//up heading correction
			desiredHeading = Vector3.up - transform.up;
			currentHeading = transform.up;
			headingError = Vector3.Cross(currentHeading, desiredHeading);
			headingCorrection = headingController.Update(headingError, Time.deltaTime);
			rigidbody.AddTorque(headingCorrection, ForceMode.Acceleration);
		}
		
	}

	public Vector3 LookAtDir
	{
		get
		{
			if (movementController.HasReachableTarget)
			{
				return movementController.CurrentTargetPosition - rigidbody.position;
			}
			return transform.rotation.eulerAngles;
		}
	}

	[Serializable]
	public class VectorPid
	{
		public float pFactor, iFactor, dFactor;

		private Vector3 integral;
		private Vector3 lastError;

		public VectorPid(float pFactor, float iFactor, float dFactor)
		{
			this.pFactor = pFactor;
			this.iFactor = iFactor;
			this.dFactor = dFactor;
		}

		public Vector3 Update(Vector3 currentError, float timeFrame)
		{
			integral += currentError * timeFrame;
			var deriv = (currentError - lastError) / timeFrame;
			lastError = currentError;
			return currentError * pFactor
				+ integral * iFactor
				+ deriv * dFactor;
		}
	}
}