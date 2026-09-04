using System;
using UnityEngine;
using UnityEngine.AI;

namespace CE6127.Tanks.AI
{
    /// <summary>
    /// Main combat behaviour for an AI tank. The tank keeps a useful firing
    /// distance, moves around cover, avoids bunching with allies, aims and fires.
    /// </summary>
    internal class CombatState : BaseState
    {
        private const float c_LineOfSightUpdate = 0.15f;
        private const float c_RepositionUpdate = 0.5f;
        private const float c_MinAllySeparation = 9f;
        private const float c_MaxFireDistance = 30f;
        private const float c_AimTolerance = 5f;

        private readonly TankSM m_TankSM;
        private float m_NextFireTime;
        private float m_NextSightCheck;
        private float m_NextRepositionTime;
        private bool m_HasClearShot;
        private bool m_PreviousAgentUpdateRotation;
        private bool m_HasTargetSample;
        private Vector3 m_PreviousTargetPosition;
        private Vector3 m_TargetVelocity;

        public CombatState(TankSM tankStateMachine) : base("Combat", tankStateMachine)
        {
            m_TankSM = tankStateMachine;
        }

        public override void Enter()
        {
            m_NextFireTime = Time.time + UnityEngine.Random.Range(0.1f, 0.5f);
            m_NextSightCheck = 0f;
            m_NextRepositionTime = 0f;
            m_HasClearShot = false;
            m_HasTargetSample = false;
            m_TargetVelocity = Vector3.zero;
            m_TankSM.SetStopDistanceToTarget();

            // NavMesh keeps controlling position, while this state controls the
            // hull rotation so movement and aiming can happen at the same time.
            m_PreviousAgentUpdateRotation = m_TankSM.NavMeshAgent.updateRotation;
            m_TankSM.NavMeshAgent.updateRotation = false;
        }

        public override void Update()
        {
            if (!TargetIsUsable())
                return;

            UpdateTargetVelocity();

            Vector3 flatToTarget = m_TankSM.Target.position - m_TankSM.transform.position;
            flatToTarget.y = 0f;
            float distance = flatToTarget.magnitude;
            Vector3 predictedTarget = PredictTargetPosition(distance);
            Vector3 flatToPredictedTarget = predictedTarget - m_TankSM.transform.position;
            flatToPredictedTarget.y = 0f;

            if (Time.time >= m_NextSightCheck)
            {
                m_NextSightCheck = Time.time + c_LineOfSightUpdate;
                m_HasClearShot = HasClearShot(predictedTarget);
            }

            // Spreading out has priority because both teams' shells can hurt AI allies.
            bool spreadingOut = TryMoveAwayFromNearbyAlly();

            bool inFiringRange = distance <= c_MaxFireDistance && distance >= 4f;
            if (!spreadingOut && (!m_HasClearShot || !inFiringRange))
                MoveForAttack(distance, flatToTarget);
            else if (!spreadingOut)
                MoveAroundTarget(flatToTarget);

            // Aiming and firing deliberately continue while NavMesh is moving the tank.
            AimAtTarget(flatToPredictedTarget);

            if (inFiringRange && m_HasClearShot && Time.time >= m_NextFireTime)
            {
                // Recalculate immediately before firing. The player may have
                // changed direction since the last line-of-sight update.
                Vector3 finalPredictedTarget = PredictTargetPosition(distance);
                Vector3 finalAimDirection = finalPredictedTarget - m_TankSM.transform.position;
                finalAimDirection.y = 0f;

                if (IsAimedAtTarget(finalAimDirection) && HasClearShot(finalPredictedTarget))
                {
                    m_TankSM.LaunchProjectile(CalculateLaunchForce(finalAimDirection.magnitude));
                    m_NextFireTime = Time.time + UnityEngine.Random.Range(
                        m_TankSM.FireInterval.x,
                        m_TankSM.FireInterval.y);
                }
            }
        }

        public override void Exit()
        {
            if (m_TankSM.NavMeshAgent != null && m_TankSM.NavMeshAgent.isOnNavMesh)
            {
                m_TankSM.NavMeshAgent.isStopped = true;
                m_TankSM.NavMeshAgent.updateRotation = m_PreviousAgentUpdateRotation;
            }
        }

        private bool TargetIsUsable()
        {
            return m_TankSM.Target != null && m_TankSM.Target.gameObject.activeInHierarchy;
        }

        private void MoveForAttack(float distance, Vector3 flatToTarget)
        {
            if (!m_TankSM.NavMeshAgent.isOnNavMesh || Time.time < m_NextRepositionTime)
                return;

            m_NextRepositionTime = Time.time + c_RepositionUpdate;
            m_TankSM.NavMeshAgent.isStopped = false;

            // If cover blocks the shot near the player, move to a side instead of
            // stopping directly behind that cover. Different tanks choose different sides.
            if (!m_HasClearShot && distance <= m_TankSM.TargetDistance + 8f && flatToTarget.sqrMagnitude > 0.01f)
            {
                float sideSign = (m_TankSM.GetInstanceID() & 1) == 0 ? 1f : -1f;
                Vector3 side = Vector3.Cross(Vector3.up, flatToTarget.normalized) * sideSign;
                Vector3 desired = m_TankSM.Target.position + side * m_TankSM.StopDistance;

                if (NavMesh.SamplePosition(desired, out NavMeshHit hit, 8f, NavMesh.AllAreas))
                {
                    m_TankSM.SetStopDistanceToZero();
                    m_TankSM.NavMeshAgent.SetDestination(hit.position);
                    return;
                }
            }

            m_TankSM.SetStopDistanceToTarget();
            m_TankSM.NavMeshAgent.SetDestination(m_TankSM.Target.position);
        }

        private void MoveAroundTarget(Vector3 flatToTarget)
        {
            if (!m_TankSM.NavMeshAgent.isOnNavMesh ||
                Time.time < m_NextRepositionTime || flatToTarget.sqrMagnitude < 0.01f)
                return;

            m_NextRepositionTime = Time.time + c_RepositionUpdate;

            // Each tank circles in a deterministic direction. The desired point
            // moves around the player, producing lateral movement while shooting.
            float sideSign = (m_TankSM.GetInstanceID() & 1) == 0 ? 1f : -1f;
            Vector3 fromTargetToTank = -flatToTarget.normalized;
            Vector3 orbitDirection = Quaternion.AngleAxis(55f * sideSign, Vector3.up) * fromTargetToTank;
            Vector3 desired = m_TankSM.Target.position + orbitDirection * m_TankSM.StopDistance;

            if (NavMesh.SamplePosition(desired, out NavMeshHit hit, 6f, NavMesh.AllAreas))
            {
                m_TankSM.SetStopDistanceToZero();
                m_TankSM.NavMeshAgent.isStopped = false;
                m_TankSM.NavMeshAgent.SetDestination(hit.position);
            }
        }

        private bool TryMoveAwayFromNearbyAlly()
        {
            Vector3 away = Vector3.zero;
            float closestDistance = float.MaxValue;

            foreach (TankManager ally in m_TankSM.GameManager.AIPlatoon.Tanks)
            {
                if (ally.Instance == null || !ally.Instance.activeInHierarchy || ally.Instance == m_TankSM.gameObject)
                    continue;

                Vector3 offset = m_TankSM.transform.position - ally.Instance.transform.position;
                offset.y = 0f;
                float distance = offset.magnitude;
                if (distance < c_MinAllySeparation && distance < closestDistance)
                {
                    closestDistance = distance;
                    away = distance > 0.1f ? offset.normalized : m_TankSM.transform.right;
                }
            }

            if (closestDistance == float.MaxValue || !m_TankSM.NavMeshAgent.isOnNavMesh)
                return false;

            if (Time.time >= m_NextRepositionTime)
            {
                m_NextRepositionTime = Time.time + c_RepositionUpdate;
                Vector3 desired = m_TankSM.transform.position + away * c_MinAllySeparation;
                if (NavMesh.SamplePosition(desired, out NavMeshHit hit, 5f, NavMesh.AllAreas))
                {
                    m_TankSM.SetStopDistanceToZero();
                    m_TankSM.NavMeshAgent.isStopped = false;
                    m_TankSM.NavMeshAgent.SetDestination(hit.position);
                }
            }

            return true;
        }

        private void UpdateTargetVelocity()
        {
            Vector3 currentPosition = m_TankSM.Target.position;
            if (!m_HasTargetSample || Time.deltaTime <= 0f)
            {
                m_PreviousTargetPosition = currentPosition;
                m_HasTargetSample = true;
                return;
            }

            Vector3 measuredVelocity = (currentPosition - m_PreviousTargetPosition) / Time.deltaTime;
            measuredVelocity.y = 0f;

            // Ignore reset/teleport spikes and smooth normal player movement.
            if (measuredVelocity.magnitude <= m_TankSM.GameManager.Speed * 2f)
                m_TargetVelocity = Vector3.Lerp(m_TargetVelocity, measuredVelocity, 0.2f);
            else
                m_TargetVelocity = Vector3.zero;

            m_PreviousTargetPosition = currentPosition;
        }

        private Vector3 PredictTargetPosition(float horizontalDistance)
        {
            float launchForce = CalculateLaunchForce(horizontalDistance);
            const float barrelAngleDegrees = 10f;
            float horizontalShellSpeed = launchForce * Mathf.Cos(barrelAngleDegrees * Mathf.Deg2Rad);
            float flightTime = horizontalDistance / Mathf.Max(1f, horizontalShellSpeed);
            flightTime = Mathf.Clamp(flightTime, 0f, 1.2f);

            Vector3 lead = Vector3.ClampMagnitude(m_TargetVelocity * flightTime, 12f);
            return m_TankSM.Target.position + lead;
        }

        private void AimAtTarget(Vector3 flatToTarget)
        {
            if (flatToTarget.sqrMagnitude < 0.01f)
                return;

            Quaternion desiredRotation = Quaternion.LookRotation(flatToTarget.normalized, Vector3.up);
            float turnThisFrame = m_TankSM.GameManager.AngularSpeed * Time.deltaTime;
            m_TankSM.transform.rotation = Quaternion.RotateTowards(
                m_TankSM.transform.rotation,
                desiredRotation,
                turnThisFrame);
        }

        private bool IsAimedAtTarget(Vector3 flatToTarget)
        {
            if (flatToTarget.sqrMagnitude < 0.01f)
                return false;

            return Vector3.Angle(m_TankSM.transform.forward, flatToTarget.normalized) <= c_AimTolerance;
        }

        private bool HasClearShot(Vector3 predictedTarget)
        {
            Vector3 targetPoint = predictedTarget + Vector3.up;
            Vector3 origin = m_TankSM.FireTransform.position + m_TankSM.FireTransform.forward * 0.4f;
            Vector3 direction = targetPoint - origin;
            RaycastHit[] hits = Physics.RaycastAll(origin, direction.normalized, direction.magnitude);
            Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

            foreach (RaycastHit hit in hits)
            {
                if (hit.transform.root == m_TankSM.transform.root)
                    continue;

                return hit.transform.root == m_TankSM.Target.root;
            }

            // The predicted point is normally in front of the moving player, so
            // an unobstructed ray can legitimately hit nothing yet.
            return true;
        }

        private float CalculateLaunchForce(float horizontalDistance)
        {
            // The prefab barrel is elevated by about 10 degrees. This estimates
            // the velocity needed to land near a same-height target, then clamps
            // it to the unchanged force limits supplied by the project.
            const float barrelAngleDegrees = 10f;
            float sinDoubleAngle = Mathf.Sin(2f * barrelAngleDegrees * Mathf.Deg2Rad);
            float gravity = Mathf.Abs(Physics.gravity.y);
            float estimatedForce = Mathf.Sqrt(horizontalDistance * gravity / Mathf.Max(0.01f, sinDoubleAngle));
            return Mathf.Clamp(estimatedForce, m_TankSM.LaunchForceMinMax.x, m_TankSM.LaunchForceMinMax.y);
        }
    }
}
