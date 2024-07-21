using System;
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.VFX;
using Unity.VisualScripting.Antlr3.Runtime.Misc;
using Unity.VisualScripting;
using UnityEditor;
using System.Net.WebSockets;

namespace kartGame.kartSystems
{
    public class arcadeKart : MonoBehaviour
    {
        [System.Serializable]
        public struct stats
        {
            [Header("Movement Settings")]
            [Min(0.001f), Tooltip("Top speed attainable when moving forward.")]
            public float topSpeed;

            [Tooltip("Acceleration")]
            public float acceleration;

            [Min(0.001f), Tooltip("Reverse top speed")]
            public float reverseSpeed;

            [Tooltip("Reverse Acceleration")]
            public float reverseAcceleration;

            [Tooltip("Acceleration lag")]
            public float accelerationCurve;

            [Tooltip("Break Strength")]
            public float braking;

            [Tooltip("Coasting drag")]
            public float coastingDrag;

            [Range(0.0f, 1.0f)]
            [Tooltip("Side-to-side friction")]
            public float grip;

            [Tooltip("Steering strength")]
            public float steer;

            [Tooltip("In air gravity")]
            public float addedGravity;

            // Allows stat adding from powerups
            public static stats operator + (stats a, stats b)
            {
                return new stats
                {
                    acceleration        = a.acceleration + b.acceleration,
                    accelerationCurve   = a.accelerationCurve + b.accelerationCurve,
                    braking             = a.braking + b.braking,
                    coastingDrag        = a.coastingDrag + b.coastingDrag,
                    addedGravity        = a.addedGravity + b.addedGravity,
                    grip                = a.grip + b.grip,  
                    reverseAcceleration = a.reverseAcceleration + b.reverseAcceleration,
                    reverseSpeed        = a.reverseSpeed + b.reverseSpeed,
                    topSpeed            = a.topSpeed + b.topSpeed,
                    steer               = a.steer + b.steer,
                };
            }
        }

        public Rigidbody rigidBody  { get; private set; }
        public InputData input      { get; private set; }
        public float airPercent     { get; private set; }
        public float groundPercent  { get; private set; }

        public arcadeKart.stats baseStats = new arcadeKart.stats
        {
            topSpeed            = 10f,
            acceleration        = 5f,
            accelerationCurve   = 4f,
            braking             = 10f,
            reverseAcceleration = 5f,
            reverseSpeed        = 5f,
            steer               = 5f,
            coastingDrag        = 4f,
            grip                = .95f,
            addedGravity        = 1f,
        };

        [Header("Vehicle Visual")]
        public List<GameObject> m_VisualWheels;

        [Header("Vehicle Physics")]
        [Tooltip("Center of mass")]
        public Transform CenterOfMass;

        [Range(0.0f, 20.0f), Tooltip("Air controll")]
        public float airborneReorientationCoefficient = 3.0f;

        [Header("Drifting")]
        [Range(0.01f, 1.0f), Tooltip("Drifting grip")]
        public float driftGrip = 0.4f;

        [Range(0.0f, 10.0f), Tooltip("Drift bonus steer")]
        public float driftAdditionalSteer = 5.0f;

        [Range(1.0f, 30.0f), Tooltip("Min speed to enter full grip")]
        public float minSpeedPercentToFinishDrift = 0.5f;

        [Range(1.0f, 20.0f), Tooltip("Drift steering control")]
        public float driftControl = 10.0f;

        [Range(0.0f, 20.0f), Tooltip("No additional steering; drift duration")]
        public float driftDampening = 10.0f;

        [Header("VFX")]
        [Tooltip("VFX on wheels during drifting")]
        public ParticleSystem driftSparkVFX;

        [Range(0.0f, 0.2f), Tooltip("VFX offset")]
        public float driftSparkHorizontalOffset = 0.1f;

        [Range(0.0f, 90.0f), Tooltip("Angle to rotate VFX")]
        public float driftSparkRotation = 17.0f;

        [Tooltip("Wheel VFX during drifting")]
        public GameObject driftTrailPrefab;

        [Range(-0.1f, 0.1f), Tooltip("Verticle displacement to ensure visibility")]
        public float driftTrailVerticalOffset;

        [Tooltip("Nozzle VFX")]
        public GameObject nozzleVFX;

        [Tooltip("Kart Nozzles")]
        public List<Transform> nozzles;

        [Header("suspensions")]
        [Tooltip("Max extension between kart body and wheels")]
        [Range(0.0f, 1.0f)]
        public float suspensionHeight = 0.2f;

        [Range(10.0f, 100000.0f), Tooltip("Suspension Stiffness")]
        public float suspensionSpring = 2000.0f;

        [Range(0.0f, 5000.0f), Tooltip("Stabilization speed")]
        public float suspensionDamp = 500.0f;

        [Tooltip("Vertical offset, wheel and kart body")]
        [Range(-1.0f, 1.0f)]
        public float wheelsPositionVerticalOffset = 0.0f;

        [Header("Physical wheels")]
        [Tooltip("Kart wheels")]
        public WheelCollider frontLeftWheel;
        public WheelCollider frontRightWheel;
        public WheelCollider rearLeftWheel;
        public WheelCollider rearRightWheel;

        [Tooltip("Layers to detect")]
        public LayerMask groundLayers = Physics.DefaultRaycastLayers;

        // Input source for kart control
        IInput[] m_Inputs;

        const float k_NullInput = 0.01f;
        const float k_NullSpeed = 0.01f;
        Vector3 m_VerticalReference = Vector3.up;

        // Drift Params
        public bool wantsToDrift { get; private set; } = false;
        public bool IsDrifting { get; private set; } = false;
        float m_CurrentGrip = 1.0f;
        float m_DriftTurningPower = 0.0f;
        float m_PreviousGroundPercent = 1.0f;
        readonly List<(GameObject trailRoot, WheelCollider wheel, TrailRenderer trail)> m_DriftTrailInstances = new List<(GameObject, WheelCollider, TrailRenderer)>();
        readonly List<(WheelCollider wheel, float horizontalOffset, float rotation, ParticleSystem sparks)> m_DriftSparkInstances = new List<(WheelCollider, float, float, ParticleSystem)>();

        bool m_CanMove = true;
        arcadeKart.stats m_FinalStats;


        Quaternion m_LastValidRotation;
        Vector3 m_LastValidPosition;
        Vector3 m_LastCollisionNormal;
        bool m_HasCollision;
        bool m_InAir = false;

        public void setCanMove(bool move) => m_CanMove = move;

        public float getMaxSpeed() => Mathf.Max(m_FinalStats.topSpeed, m_FinalStats.reverseSpeed);

        private void activateDriftVFX(bool active) 
        {
            foreach (var vfx in m_DriftSparkInstances)
            {
                if (active && vfx.wheel.GetGroundHit(out WheelHit hit))
                {
                    if (!vfx.sparks.isPlaying)
                        vfx.sparks.Play();
                }
                else
                {
                    if(vfx.sparks.isPlaying)
                        vfx.sparks.Stop(true, ParticleSystemStopBehavior.StopEmitting);
                }
            }

            foreach (var trail in m_DriftTrailInstances)
                trail.Item3.emitting = active && trail.wheel.GetGroundHit(out WheelHit hit);

        }

        private void updateDriftVFXOrientation()
        {
            foreach (var vfx in m_DriftSparkInstances)
            {
                vfx.sparks.transform.position = vfx.wheel.transform.position - (vfx.wheel.radius * Vector3.up) + (driftTrailVerticalOffset * Vector3.up) + (transform.right * vfx.horizontalOffset);
                vfx.sparks.transform.rotation = transform.rotation * Quaternion.Euler(0.0f, 0.0f, vfx.rotation);
            }

            foreach (var trail in m_DriftSparkInstances)
            {
                trail.trailRoot.transform.position = trail.wheel.transform.position - (trail.wheel.radius * Vector3.up) + (driftTrailVerticalOffset * Vector3.up);
                trail.trailRoot.transform.rotation = transform.rotation;
            }
        }

        void updateSuspensionParams(WheelCollider wheel)
        {
            wheel.suspensionDistance = suspensionHeight;
            wheel.center = new Vector3(0.0f, wheelsPositionVerticalOffset, 0.0f);
            JointSpring spring = wheel.suspensionSpring;
            spring.spring = suspensionSpring;
            spring.damper = suspensionDamp;
            wheel.suspensionSpring = spring; 
        }

        void Awake() 
        {
            rigidBody = GetComponent<Rigidbody>();
            m_Inputs = GetComponents<IUnitInputPort>();

            updateSuspensionParams(frontLeftWheel);
            updateSuspensionParams(frontRightWheel);
            updateSuspensionParams(rearLeftWheel);
            updateSuspensionParams(rearRightWheel);

            m_CurrentGrip = baseStats.grip;

            if(driftSparkVFX != null)
            {
                addSparkToWheel(rearLeftWheel, -driftSparkHorizontalOffset, -driftSparkRotation);
                addSparkToWheel(rearRightWheel, driftSparkHorizontalOffset, driftSparkRotation);
            }

            if (driftTrailPrefab != null)
            {
                addTrailToWheel(rearLeftWheel);
                addTrailToWheel(rearRightWheel);
            }

            if (nozzleVFX != null)
            {
                foreach (var nozzle in nozzles)
                {
                    Instantiate(nozzleVFX, nozzle, false);
                }
            }
        }

        void addSparkToWheel(WheelCollider wheel, float horizontalOffset, float rotation)
        {
            GameObject vfx = Instantiate(driftSparkVFX.gameObject, wheel.transform, false);
            ParticleSystem spark = vfx.GetComponent<ParticleSystem>();
            spark.Stop();
            m_DriftSparkInstances.Add((wheel, horizontalOffset, -rotation, spark));
        }

        void FixedUpdate()
        {
            updateSuspensionParams(frontLeftWheel);
            updateSuspensionParams(frontRightWheel);
            updateSuspensionParams(rearLeftWheel);
            updateSuspensionParams(rearRightWheel);

            gatherInputs();

            // Applying physics properties
            rigidBody.centerOfMass = transform.InverseTransformPoint(CenterOfMass.position);

            int groundedCount = 0;
            if (frontLeftWheel.isGrounded && frontLeftWheel.GetGroundHit(out WheelHit hit))
                groundedCount ++;
            if (frontRightWheel.isGrounded && frontRightWheel.GetGroundHit(out hit))
                groundedCount++;
            if (rearLeftWheel.isGrounded && rearLeftWheel.GetGroundHit(out hit))
                groundedCount++;
            if (rearRightWheel.isGrounded && rearRightWheel.GetGroundHit(out hit))
                groundedCount++;

            // Calculate how grounded and airborne we are
            groundedPercent = (float) groundedCount / 4.0f;
            airPercent = 1 - groundedPercent;

            // apply vehicle physics
            if (m_CanMove)
            {
                moveVehicle(input.Accelerate, input.Brake, input.TurnInput);
            }
            groundAirBourne()

            m_PreviousGroundPercent = groundPercent;

            updateDriftVFXOrientation();

        }

        void gatherInputs()
        {
            input = new InputData();
            wantsToDrift = false;

            // Gather nonezero input from sources
            for (int i = 0; i < m_Inputs.Length; i++)
            {
                input = m_Inputs[i].GenerateInput();
                wantsToDrift = input.Brake && Vector3.Dot(rigidBody.velocity, transform.forward) > 0.0f;
            }

        }

        void groundAirBourne()
        {
            // In air, fall faster
            if (airPercent >= 1)
                rigidBody.velocity += Physics.gravity * Time.fixedDeltaTime * m_FinalStats.addedGravity;
        }

        public void Reset()
        {
            Vector3 euler = transform.rotation.eulerAngles;
            euler.x = euler.z = 0f;
            transform.rotation = Quaternion.Euler(euler);
        }

        public float localSpeed()
        {
            if (m_CanMove)
            {
                float dot = Vector3.Dot(transform.forward, rigidBody.velocity);
                if(Mathf.Abs(dot) > 0.1f)
                {
                    float speed = rigidBody.velocity.magnitude;
                    return dot < 0 ? - (speed / m_FinalStats.reverseSpeed) : (speed / m_FinalStats.topSpeed);
                }
                return 0f;
            }
            else
            {
                // Value to play kart sound during race start countdown
                return input.Accelerate ? 1.0f : 0.0f;
            }
        }

        void OnCollisionEnter(Collision collision) => m_HasCollision = true;
        void OnCollisionExit(Collision collision) => m_HasCollision = false;

        void OnCollisionStay(Collision collision)
        {
            m_HasCollision = true;
            m_LastCollisionNormal = Vector3.zero;
            float dot = -1.0f;

            foreach (var contact in collision.contacts)
            {
                if(Vector3.Dot(contact.normal, Vector3.up) > dot)
                    m_LastCollisionNormal = contact.normal;
            }
        }

        void moveVehicle(bool accelerate, bool brake, float turnInput)
        {
            float accelInput = (accelerate ? 1.0f : 0.0f) - (brake ? 1.0f : 0.0f);

            // Manual acceleration curve coefficient scalar
            float accelerationCurveCoeff = 5;
            Vector3 localVel = transform.InverseTransformVector(rigidBody.velocity);

            bool accelDirectionIsFwd = accelInput >= 0;
            bool localVelDirectionIsFwd = localVel.z >= 0;

            // use the max speed for the direction we are going--forward or reverse
            float maxSpeed = localVelDirectionIsFwd ? m_FinalStats.topSpeed : m_FinalStats.reverseSpeed;
            float accelPower = accelDirectionIsFwd ? m_FinalStats.acceleration : m_FinalStats.reverseAcceleration;

            float currentSpeed = rigidBody.velocity.magnitude;
            float accelRampT = currentSpeed / maxSpeed;
            float multipliedAccelerationCurve = m_FinalStats.accelerationCurve * accelerationCurveCoeff;
            float accelRamp = Mathf.Lerp(multipliedAccelerationCurve, 1, accelRampT * accelRampT);

            bool isBraking = (localVelDirectionIsFwd && brake) || (!localVelDirectionIsFwd && accelerate);

            // If moving reverse to current vel, use braking acc
            float finalAccelPower = isBraking ? m_FinalStats.braking : accelPower;
            
            float finalAcceleration = finalAccelPower * accelRamp;

            // Apply inputs to forward/backward
            float turningPower = IsDrifting ? m_DriftTurningPower : turnInput * m_FinalStats.steer;

            Quaternion turnAngle = Quaternion.AngleAxis(turningPower, transform.up);
            Vector3 fwd = turnAngle * transform.forward;
            Vector3 movement = fwd * accelInput * finalAcceleration * ((m_HasCollision || groundPercent > 0.0f) ? 1.0f : 0.0f);

            //forward movement
            bool wasOverMaxSpeed = currentSpeed >= maxSpeed;

            // if over max speed, cannot accelerate faster
            if (wasOverMaxSpeed && !isBraking)
                movement *= 0.0f;

            Vector3 newVelocity = rigidBody.velocity + movement * Time.fixedDeltaTime;
            newVelocity.y = rigidBody.velocity.y;

            // clamp max speed if we are on ground
            if (groundPercent > 0.0f && !wasOverMaxSpeed)
            {
                newVelocity = Vector3.ClampMagnitude(newVelocity, maxSpeed);
            }

            // coasting is when we aren't touching accelerate
            if (Mathf.Abs(accelInput) < k_NullInput && groundPercent > 0.0f)
            {
                newVelocity = Vector3.MoveTowards(newVelocity, new Vector3(0, rigidBody.velocity.y, 0), Time.fixedDeltaTime * m_FinalStats.coastingDrag);
            }

            rigidBody.velocity = newVelocity;

            // Drift
            if(groundPercent > 0.0f)
            {
                if(m_InAir)
                {
                    m_InAir = false;
                    Instantiate(JumpVFX, transform.position, Quaternion.identity);
                }

                // manual angular velocity coefficient
                float angularVelocitySteering = 0.4f;
                float angularVelocitySmoothSpeed = 20f; 

                // reversed steering 
                if (!localVelDirectionIsFwd && !accelDirectionIsFwd)
                    angularVelocitySteering *= -1.0f;
                
                var angularVel = rigidBody.angularVelocity;

                // move Y angular vlocity towards target
                angularVel.y = Mathf.MoveTowards(angularVel.y, turningPower * angularVelocitySteering, Time.fixedDeltaTime * angularVelocitySmoothSpeed);

                // Apply angular velocity
                rigidBody.angularVelocity = angularVel;

                // rotate rigidbody's 

            }

        }

    }


}