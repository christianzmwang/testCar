using System.Collections;
using System.Collections.Generic;
using UnityEditor.EditorTools;
using UnityEngine;

namespace kartGame.kartSystems
{
    public class arcadeEngineAudio : MonoBehaviour
    {
        [Tooltip("Start sound")]
        public AudioSource startSound;

        [Tooltip("Idle sound")]
        public AudioSource idleSound;

        [Tooltip("Driving sound")]
        public AudioSource runningSound;

        [Tooltip("Drifiting sound")]
        public AudioSource drift;

        [Tooltip("Max vol at full speed")]
        [Range(0.1f, 1.0f)]public float runningSoundMaxVolume = 1.0f;

        [Tooltip("Max pitch at full speed")]
        [Range(0.1f, 2.0f)] public float RunningSOundMaxPitch = 1.0f;

        [Tooltip("Reverse sound")]
        public AudioSource reverseSound;

        [Tooltip("Max reverse sound; at max reverse speed")]
        [Range(0.1f, 1.0f)] public float reverseSoundMaxVolume = 0.5f;

        [Tooltip("Max pitch at max reverse speed")]
        [Range(0.1f, 2.0f)] public float reverseSoundMaxPitch = 0.6f;

        arcadeKart arcadeKart;

    }

    void Awake()
    {
        arcadeKart = GetComponentInParent<ArcadeKart>();
    }

    void Update()
    {
        float kartSpeed = 0.0f;
        if (arcadeKart != null)
        {
            kartSpeed = arcadeKart.LocalSpeed();
            drift.volume = arcadeKart.isDrifting && arcadeKart.groundPercent > 0.0f ? arcadeKart.Rigidbody.velocity.magnitude / arcadeKart.getMaxSpeed() : 0.0f;

            idleSound.volume = Mathf.Lerp(0.6f, 0.0f, kartSpeed * 4);
        }

        if (kartSpeed < 0.0f)
        {
            // In reverse
            runningSound.volume = 0.0f;
            reverseSound.volume = Mathf.Lerp(0.1f, reverseSoundMaxVolume, -kartSpeed * 1.2f);
            reverseSound.pitch = Mathf.Lerp(0.1f, reverseSoundMaxPitch, -kartSpeed + (Mathf.Sin(Time.time) * .1f));
        } 
        else
        {
            // Moving forward
            reverseSound.volume = 0.0f;
            runningSound.volume = Mathf.Lerp(0.1f, runningSoundMaxVolume, kartSpeed *1.2f);
            runningSound.pitch = Mathf.Lerp(0.3f, runningSoundMaxPitch, kartSpeed + (Mathf.Sin(Time.time) * .1f)); 
        }

    }


}
