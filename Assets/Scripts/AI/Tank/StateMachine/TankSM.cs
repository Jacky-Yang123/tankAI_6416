/// <remarks>
/// <para>
/// Reflection should be used cautiously due to its performance overhead and the loss of compile-time type safety.
/// It is slower compared to direct access via properties, methods, or fields since it involves runtime type inspection.
/// Moreover, reflection can lead to less maintainable and harder-to-debug code, as it bypasses standard access
/// mechanisms and encapsulation principles.
/// </para>
/// <para>
/// * Performance: Reflection is considerably slower than direct field access, impacting application performance.
/// * Encapsulation: It bypasses access modifiers, potentially breaking encapsulation and leading to unintended consequences.
/// * Maintainability: Code using reflection can be less readable and harder to maintain, especially for developers unfamiliar with it.
/// * Type safety: Reflection bypasses compile-time type checks, increasing the risk of runtime errors.
/// </para>
/// <para>
/// Instead of reflection, it is recommended to use classical getters and setters or public properties to access
/// and manipulate field values. These provide better performance, type safety, and allow for encapsulation.
/// Reflection may be useful in scenarios where dynamic type access is required, such as in frameworks or libraries,
/// but should be avoided in general application logic.
/// </para>
/// </remarks>

using System.Linq;
using System;
using UnityEngine;
using UnityEngine.AI;

using Random = UnityEngine.Random;
using Debug = UnityEngine.Debug;

namespace CE6127.Tanks.AI
{
    /// <summary>
    /// Class <c>TankSM</c> state machine for the tank.
    /// </summary>
    internal class TankSM : StateMachine
    {
        protected internal struct States
        {
            // States:
            public IdleState Idle;
            public PatrollingState Patrolling;
            public CombatState Combat;

            internal States(TankSM sm)
            {
                Idle = new IdleState(sm);
                Patrolling = new PatrollingState(sm);
                Combat = new CombatState(sm);
            }
        }

        public States m_States;
        [HideInInspector] public GameManager GameManager;           // Reference to the GameManager.
        [HideInInspector] public NavMeshAgent NavMeshAgent;         // Reference to the NavMeshAgent.
        [Header("Patrolling")]
        [Tooltip("Minimum and maximum time delay for patrolling wait.")]
        public Vector2 PatrolWaitTime = new(1.5f, 3.5f);            // A minimum and maximum time delay for patrolling wait.
        [Tooltip("Minimum and maximum circumradius of the area to patrol at a given update time.")]
        public Vector2 PatrolMaxDist = new(15f, 30f);               // A minimum and maximum circumradius of the area to patrol.
        [Range(0f, 2f)] public float PatrolNavMeshUpdate = 0.2f;    // A delay between each parolling path update.
        [Header("Targeting")]
        [Tooltip("Minimum and maximum range for the targeting range.")]
        public Vector2 StartToTargetDist = new(28f, 35f);           // A minimum and maximum range for the targeting range.
        [HideInInspector] public float TargetDistance;              // The distance between the tank and the target.
        [Tooltip("Minimum and maximum range for the stopping range.")]
        public Vector2 StopAtTargetDist = new(18f, 22f);            // A minimum and maximum range for the stopping range.
        [HideInInspector] public float StopDistance;                // The distance between the tank and the target.
        [Range(0f, 2f)] public float TargetNavMeshUpdate = 0.2f;    // A delay between each targeting path update.
        [Header("Blending")]
        [Range(0f, 1f)] public float OrientSlerpScalar = 0.2f;      // A scalar for the slerp.
        // [Header("Target")]
        [HideInInspector] public Transform Target;                  // Reference to the target's transform.
        // [Header("NavMesh")]
        [HideInInspector] public float NavMeshUpdateDeadline;       // The time when the next path update is due.
        [Header("Firing")]
        [Tooltip("Minimum and maximum cooldown time delay between each firing in seconds.")]
        public Vector2 FireInterval = new(0.7f, 2.5f);              // A minimum and maximum cooldown time delay between each firing.
        [Tooltip("Force given to the shell if the fire button is not held, and the force given to the shell if the fire button is held for the max charge time in seconds.")]
        public Vector2 LaunchForceMinMax = new(6.5f, 30f);          // The force given to the shell if the fire button is not held, and the force given to the shell if the fire button is held for the max charge time.
        [Header("References")]
        [Tooltip("Prefab")] public Rigidbody Shell;                 // Prefab of the shell.
        [Tooltip("Transform")] public Transform FireTransform;      // A child of the tank where the shells are spawned.
        // public Slider AimSlider;                                 // A child of the tank that displays the current launch force.
        [Header("Firing Audio")]
        public AudioSource SFXAudioSource;                          // Reference to the audio source used to play the shooting audio. NB: different to the movement audio source.
        // public AudioClip ShotChargingAudioClip;                  // Audio that plays when each shot is charging up.
        public AudioClip ShotFiringAudioClip;                       // Audio that plays when each shot is fired.

        private bool m_Started = false; // Whether the tank has started moving.
        private Rigidbody m_Rigidbody;  // Reference used to the tank's regidbody.
        private TankSound m_TankSound;  // Reference used to play sound effects.

        // 以下字段属于行为树运行数据。旧 FSM 类仍保留在工程中方便对照和回退，
        // 但比赛时 TankSM.Update 只 Tick 这棵行为树，不再调用旧状态机。
        private BTNode m_BehaviourTree;
        private SquadOrder m_Order;
        private Vector3 m_PredictedAimPoint;
        private Vector3 m_LastProgressPosition;
        private float m_SelectedLaunchForce;
        private bool m_HasBallisticSolution;
        private float m_NextFireTime;
        private float m_NextPathUpdate;
        private float m_StuckSince;
        private float m_DodgeUntil;
        private Vector3 m_DodgeDestination;
        private int m_LastThreatShellId;

        private const float c_MinAttackDistance = 4f;
        private const float c_MaxAttackDistance = 30f;
        private const float c_AimTolerance = 4f;
        private const float c_PathUpdateInterval = 0.2f;

        /// <summary>
        /// Method <c>MoveTurnSound</c> returns the current tank's velocity.
        /// </summary>
        private Vector2 MoveTurnSound() => new Vector2(Mathf.Abs(NavMeshAgent.velocity.x), Mathf.Abs(NavMeshAgent.velocity.z));

        /// <summary>
        /// Method <c>GetInitialState</c> returns the initial state of the state machine.
        /// </summary>
        protected override BaseState GetInitialState() => m_States.Combat;

        /// <summary>
        /// Method <c>SetNavMeshAgent</c> sets the NavMeshAgent's speed and angular speed.
        /// </summary>
        private void SetNavMeshAgent()
        {
            NavMeshAgent.speed = GameManager.Speed;
            NavMeshAgent.angularSpeed = GameManager.AngularSpeed;
        }

        /// <summary>
        /// Method <c>SetStopDistanceToZero</c> sets the NavMeshAgent's stopping distance to zero.
        /// </summary>
        public void SetStopDistanceToZero() => NavMeshAgent.stoppingDistance = 0f;

        /// <summary>
        /// Method <c>SetStopDistanceToTarget</c> sets the NavMeshAgent's stopping distance to the target's distance.
        /// </summary>
        public void SetStopDistanceToTarget() => NavMeshAgent.stoppingDistance = StopDistance;

        /// <summary>
        /// Method <c>Awake</c> is called when the script instance is being loaded.
        /// </summary>
        private void Awake()
        {
            m_States = new States(this);

            GameManager = GameManager.Instance;

            m_Rigidbody = GetComponent<Rigidbody>();
            NavMeshAgent = GetComponent<NavMeshAgent>();
            m_TankSound = GetComponent<TankSound>();

            SetNavMeshAgent();

            TargetDistance = Random.Range(StartToTargetDist.x, StartToTargetDist.y);
            StopDistance = Random.Range(StopAtTargetDist.x, StopAtTargetDist.y);

            SetStopDistanceToTarget();

            var tankManagers = GameManager.PlayerPlatoon.Tanks.Take(1);
            if (tankManagers.Count() != 0)
                Target = tankManagers.First().Instance.transform;
            else
                Debug.LogError("'Player Platoon' is empty!");

            NavMeshAgent.updateRotation = false;
            BuildBehaviourTree();
        }

        /// <summary>
        /// Method <c>OnEnable</c> is called when the object becomes enabled and active.
        /// </summary>
        private void OnEnable()
        {
            // When the tank is turned on, make sure it's not kinematic.
            m_Rigidbody.isKinematic = false;
        }

        /// <summary>
        /// Method <c>Start</c> is called on the frame when a script is enabled just before any of the Update methods are called the first time.
        /// </summary>
        private new void Start()
        {
            m_TankSound.MoveTurnInputCalc += MoveTurnSound;
            m_LastProgressPosition = transform.position;
        }

        /// <summary>
        /// Method <c>OnDisable</c> is called when the behaviour becomes disabled or inactive.
        /// </summary>
        private void OnDisable()
        {
            // When the tank is turned off, set it to kinematic so it stops moving.
            m_Rigidbody.isKinematic = true;

            m_TankSound.MoveTurnInputCalc -= MoveTurnSound;
        }

        /// <summary>
        /// Method <c>Update</c> is called every frame, if the MonoBehaviour is enabled.
        /// </summary>
        private new void Update()
        {
            if (!m_Started && GameManager.IsRoundPlaying)
            {
                m_Started = true;
                m_NextFireTime = Time.time + Random.Range(0.05f, FireInterval.x);
                m_LastProgressPosition = transform.position;
                m_StuckSince = 0f;
            }

            if (GameManager.IsRoundPlaying)
            {
                // 先由统一指挥官更新共享感知和三个不同槽位，再执行本车行为树。
                m_Order = SquadBlackboard.GetOrder(this);
                Target = SquadBlackboard.Player;
                m_BehaviourTree?.Tick();
            }
            else
            {
                m_Started = false;
                if (NavMeshAgent.isOnNavMesh)
                    NavMeshAgent.isStopped = true;
            }
        }

        /// <summary>
        /// 建立单车行为树。根节点是 Parallel，所以移动/规避与瞄准/射击会在
        /// 同一帧工作。Selector 从上到下体现移动动作的安全优先级。
        /// </summary>
        private void BuildBehaviourTree()
        {
            BTNode movementBranch = new BTSelector(
                new BTAction(AvoidIncomingShell),
                new BTAction(AvoidPlayerCharge),
                new BTAction(SpreadFromAllies),
                new BTAction(RecoverIfStuck),
                new BTAction(FollowAssignedSlot));

            BTNode combatBranch = new BTSequence(
                new BTCondition(() => Target != null && Target.gameObject.activeInHierarchy),
                new BTAction(AimAtPredictedPosition),
                new BTCondition(() => Time.time >= m_NextFireTime),
                new BTCondition(IsTargetInAttackRange),
                new BTCondition(IsRoleAllowedToFire),
                new BTCondition(IsAimed),
                new BTCondition(HasClearBallisticPath),
                new BTAction(FireAtTarget));

            m_BehaviourTree = new BTParallel(movementBranch, combatBranch);
        }

        /// <summary>
        /// 最高优先级：逐段模拟玩家炮弹受重力影响的抛物线，同时预测本车当前移动，
        /// 再从弹道左右两侧选择兼顾安全、队友间距和原阵型槽位的规避点。
        /// </summary>
        private BTStatus AvoidIncomingShell()
        {
            Rigidbody mostDangerousShell = null;
            Vector3 predictedThreatPoint = Vector3.zero;
            Vector3 threatDirection = Vector3.zero;
            float bestThreatScore = float.MaxValue;

            foreach (Rigidbody shell in SquadBlackboard.PlayerShells)
            {
                if (shell == null || shell.velocity.sqrMagnitude < 1f)
                    continue;

                Vector3 ownVelocity = NavMeshAgent.isOnNavMesh ? NavMeshAgent.velocity : Vector3.zero;
                float closestDistance = float.MaxValue;
                float closestTime = 0f;
                Vector3 closestPoint = shell.position;
                for (float future = 0.08f; future <= 1.5f; future += 0.08f)
                {
                    Vector3 shellPoint = shell.position + shell.velocity * future +
                                         0.5f * Physics.gravity * future * future;
                    Vector3 tankPoint = transform.position + ownVelocity * future;
                    float distance = Vector3.Distance(shellPoint, tankPoint);
                    if (distance >= closestDistance)
                        continue;
                    closestDistance = distance;
                    closestTime = future;
                    closestPoint = shellPoint;
                }

                // 5米爆炸半径之外再留2.5米反应余量。
                if (closestDistance > 7.5f)
                    continue;

                float threatScore = closestTime + closestDistance * 0.1f;
                if (threatScore >= bestThreatScore)
                    continue;
                bestThreatScore = threatScore;
                mostDangerousShell = shell;
                predictedThreatPoint = closestPoint;
                threatDirection = shell.velocity + Physics.gravity * closestTime;
                threatDirection.y = 0f;
            }

            if (mostDangerousShell != null)
            {
                int shellId = mostDangerousShell.GetInstanceID();
                if (shellId != m_LastThreatShellId || Time.time >= m_DodgeUntil)
                {
                    float dodgeDistance = SquadBlackboard.RemainingRoundTime <= 20f ? 7f : 10f;
                    m_DodgeDestination = ChooseSafeDodgePoint(
                        predictedThreatPoint, threatDirection, dodgeDistance);
                    m_LastThreatShellId = shellId;
                    m_DodgeUntil = Time.time +
                        (SquadBlackboard.RemainingRoundTime <= 20f ? 0.35f : 0.55f);
                }
            }

            if (Time.time < m_DodgeUntil)
            {
                SetDestinationOnNavMesh(m_DodgeDestination, 7f);
                return BTStatus.Running;
            }
            return BTStatus.Failure;
        }

        /// <summary>玩家蓄力且炮口正对本车时，本车离开弹道；另外两辆仍由并行战斗分支开火。</summary>
        private BTStatus AvoidPlayerCharge()
        {
            if (!m_Order.MustEvade || SquadBlackboard.PlayerFireTransform == null)
                return BTStatus.Failure;

            float evadeDistance = SquadBlackboard.RemainingRoundTime <= 20f ? 6.5f : 9f;
            Vector3 fireDirection = SquadBlackboard.PlayerFireTransform.forward;
            fireDirection.y = 0f;
            Vector3 threatPoint = SquadBlackboard.PlayerPosition +
                                  fireDirection.normalized * Vector3.Distance(
                                      SquadBlackboard.PlayerPosition, transform.position);
            SetDestinationOnNavMesh(
                ChooseSafeDodgePoint(threatPoint, fireDirection, evadeDistance), 6f);
            return BTStatus.Running;
        }

        /// <summary>比较弹道左右两个可行点，避免固定向一侧躲进炮弹或队友。</summary>
        private Vector3 ChooseSafeDodgePoint(
            Vector3 threatPoint, Vector3 trajectoryDirection, float dodgeDistance)
        {
            trajectoryDirection.y = 0f;
            if (trajectoryDirection.sqrMagnitude < 0.01f)
                trajectoryDirection = transform.forward;
            Vector3 side = Vector3.Cross(Vector3.up, trajectoryDirection.normalized);
            Vector3 best = transform.position + side * dodgeDistance;
            float bestScore = float.MinValue;

            for (int sign = -1; sign <= 1; sign += 2)
            {
                Vector3 desired = transform.position + side * sign * dodgeDistance;
                // 轻微拉向原槽位，使单车规避后仍会回到小队阵型。
                if (m_Order.HasSlot)
                {
                    Vector3 towardSlot = m_Order.Slot - transform.position;
                    towardSlot.y = 0f;
                    desired += Vector3.ClampMagnitude(towardSlot, 2.5f);
                }
                if (!NavMesh.SamplePosition(desired, out NavMeshHit hit, 7f, NavMesh.AllAreas))
                    continue;

                float score = Vector3.Distance(hit.position, threatPoint) * 3f;
                if (m_Order.HasSlot)
                    score -= Vector3.Distance(hit.position, m_Order.Slot) * 0.35f;
                foreach (TankManager ally in GameManager.AIPlatoon.Tanks)
                {
                    if (ally.Instance == null || ally.Instance == gameObject ||
                        !ally.Instance.activeInHierarchy)
                        continue;
                    float allyDistance = Vector3.Distance(hit.position, ally.Instance.transform.position);
                    if (allyDistance < SquadBlackboard.MinimumAllyDistance)
                        score -= (SquadBlackboard.MinimumAllyDistance - allyDistance) * 8f;
                }

                if (score <= bestScore)
                    continue;
                bestScore = score;
                best = hit.position;
            }
            return best;
        }

        /// <summary>队友小于11米时优先分散，避免玩家5米爆炸半径同时覆盖两车。</summary>
        private BTStatus SpreadFromAllies()
        {
            Vector3 away = Vector3.zero;
            int closeCount = 0;
            foreach (TankManager ally in GameManager.AIPlatoon.Tanks)
            {
                if (ally.Instance == null || ally.Instance == gameObject || !ally.Instance.activeInHierarchy)
                    continue;
                Vector3 offset = transform.position - ally.Instance.transform.position;
                offset.y = 0f;
                if (offset.magnitude < SquadBlackboard.MinimumAllyDistance)
                {
                    away += offset.sqrMagnitude > 0.01f ? offset.normalized : transform.right;
                    closeCount++;
                }
            }

            if (closeCount == 0)
                return BTStatus.Failure;
            SetDestinationOnNavMesh(transform.position + away.normalized * 9f, 6f);
            return BTStatus.Running;
        }

        /// <summary>有路径却超过1秒几乎没移动时，先向侧面取一个可行点脱离障碍。</summary>
        private BTStatus RecoverIfStuck()
        {
            if (!NavMeshAgent.isOnNavMesh || !NavMeshAgent.hasPath)
            {
                m_StuckSince = 0f;
                m_LastProgressPosition = transform.position;
                return BTStatus.Failure;
            }

            if (Vector3.Distance(transform.position, m_LastProgressPosition) > 0.5f)
            {
                m_StuckSince = 0f;
                m_LastProgressPosition = transform.position;
                return BTStatus.Failure;
            }

            if (m_StuckSince <= 0f)
                m_StuckSince = Time.time;
            if (Time.time - m_StuckSince < 1f)
                return BTStatus.Failure;

            Vector3 side = (GetInstanceID() & 1) == 0 ? transform.right : -transform.right;
            SetDestinationOnNavMesh(transform.position + side * 8f - transform.forward * 3f, 8f);
            m_StuckSince = Time.time;
            return BTStatus.Running;
        }

        /// <summary>执行指挥官分配的独立槽位；NavMesh 自动选择当前地图上的最短可行路径。</summary>
        private BTStatus FollowAssignedSlot()
        {
            if (!m_Order.HasSlot || !NavMeshAgent.isOnNavMesh)
                return BTStatus.Failure;

            if (Time.time >= m_NextPathUpdate)
            {
                m_NextPathUpdate = Time.time + c_PathUpdateInterval;
                NavMeshAgent.stoppingDistance = 1.5f;
                NavMeshAgent.isStopped = false;

                Vector3 destination = m_Order.Slot;
                if (Vector3.Distance(transform.position, destination) < 2.5f)
                {
                    // 到达槽位后沿玩家周围继续走弧线，不原地成为容易命中的固定靶。
                    Vector3 radial = transform.position - SquadBlackboard.PlayerPosition;
                    radial.y = 0f;
                    if (radial.sqrMagnitude > 0.1f)
                    {
                        Vector3 tangent = Vector3.Cross(Vector3.up, radial.normalized);
                        if (m_Order.Role == SquadRole.LeftInterceptor ||
                            (m_Order.Role == SquadRole.Pressure && (GetInstanceID() & 1) == 0))
                            tangent = -tangent;
                        destination += tangent * 4f;
                    }
                }
                NavMeshAgent.SetDestination(destination);
            }
            return BTStatus.Running;
        }

        /// <summary>
        /// 使用稳定窗口估计出的玩家速度和转向率，迭代求解移动目标拦截点。
        /// 弹道纵向落点取玩家碰撞箱中心，而不是错误地假设目标与1.7米高炮口等高。
        /// </summary>
        private BTStatus AimAtPredictedPosition()
        {
            Vector3 predicted = SquadBlackboard.PlayerPosition;
            float targetImpactHeight = GetTargetImpactHeight();
            predicted.y = targetImpactHeight;
            float flightTime = 0f;
            for (int i = 0; i < 6; ++i)
            {
                CalculateBallisticSolution(predicted, out m_SelectedLaunchForce, out flightTime);
                predicted = SquadBlackboard.PredictPlayerPosition(flightTime);
                predicted.y = targetImpactHeight;
            }
            m_PredictedAimPoint = predicted;
            m_HasBallisticSolution = CalculateBallisticSolution(
                m_PredictedAimPoint, out m_SelectedLaunchForce, out flightTime);

            Vector3 aimDirection = m_PredictedAimPoint - transform.position;
            aimDirection.y = 0f;
            if (aimDirection.sqrMagnitude > 0.01f)
            {
                Quaternion desired = Quaternion.LookRotation(aimDirection.normalized, Vector3.up);
                transform.rotation = Quaternion.RotateTowards(
                    transform.rotation, desired, GameManager.AngularSpeed * Time.deltaTime);
            }
            return BTStatus.Success;
        }

        private bool IsTargetInAttackRange()
        {
            Vector3 flat = m_PredictedAimPoint - FireTransform.position;
            flat.y = 0f;
            float distance = flat.magnitude;
            return distance >= c_MinAttackDistance && distance <= c_MaxAttackDistance &&
                   m_HasBallisticSolution;
        }

        private bool IsAimed()
        {
            Vector3 direction = m_PredictedAimPoint - transform.position;
            direction.y = 0f;
            return direction.sqrMagnitude > 0.01f &&
                   Vector3.Angle(transform.forward, direction.normalized) <= c_AimTolerance;
        }

        /// <summary>
        /// 侧面拦截车可以抓住横穿弹道的玩家立即开火。后方压力车若距离太远且
        /// 玩家仍在同速远离，则不浪费一发必定落后的满速炮弹；最后20秒取消限制。
        /// </summary>
        private bool IsRoleAllowedToFire()
        {
            if (m_Order.Role != SquadRole.Pressure || !SquadBlackboard.PlayerEscaping ||
                SquadBlackboard.RemainingRoundTime <= 20f)
                return true;

            Vector3 toPlayer = SquadBlackboard.PlayerPosition - transform.position;
            toPlayer.y = 0f;
            Vector3 playerVelocity = SquadBlackboard.PlayerVelocity;
            playerVelocity.y = 0f;
            if (toPlayer.sqrMagnitude < 0.01f || playerVelocity.sqrMagnitude < 1f)
                return true;

            bool movingDirectlyAway = Vector3.Dot(toPlayer.normalized, playerVelocity.normalized) > 0.4f;
            bool saturatedShot = m_SelectedLaunchForce >= LaunchForceMinMax.y - 0.5f;
            return !movingDirectlyAway || (!saturatedShot && toPlayer.magnitude <= 26f);
        }

        /// <summary>
        /// 将受重力的炮弹轨迹分成10段逐段射线检测。墙、岩石或队友先被命中
        /// 就禁止开火；玩家先被命中或整条预测轨迹无障碍才允许发射。
        /// </summary>
        private bool HasClearBallisticPath()
        {
            Vector3 origin = FireTransform.position + FireTransform.forward * 0.4f;
            Vector3 velocity = FireTransform.forward * m_SelectedLaunchForce;
            Vector3 flat = m_PredictedAimPoint - origin;
            flat.y = 0f;
            float horizontalSpeed = Mathf.Max(1f,
                new Vector3(velocity.x, 0f, velocity.z).magnitude);
            float totalTime = Mathf.Clamp(flat.magnitude / horizontalSpeed, 0.05f, 1.75f);
            Vector3 previous = origin;

            for (int step = 1; step <= 10; ++step)
            {
                float time = totalTime * step / 10f;
                Vector3 next = origin + velocity * time + 0.5f * Physics.gravity * time * time;
                Vector3 segment = next - previous;
                RaycastHit[] hits = Physics.RaycastAll(previous, segment.normalized, segment.magnitude);
                Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));
                foreach (RaycastHit hit in hits)
                {
                    if (hit.transform.root == transform.root)
                        continue;
                    return Target != null && hit.transform.root == Target.root;
                }
                previous = next;
            }
            return true;
        }

        private BTStatus FireAtTarget()
        {
            LaunchProjectile(m_SelectedLaunchForce);
            // 每次射击后，在配置的最短与最长间隔之间随机决定下一次可开火时间。
            m_NextFireTime = Time.time + Random.Range(FireInterval.x, FireInterval.y);
            return BTStatus.Success;
        }

        /// <summary>所有规避动作共用的唯一导航辅助：采样到现有可行走区域后移动。</summary>
        private void SetDestinationOnNavMesh(Vector3 desired, float sampleRadius)
        {
            if (!NavMeshAgent.isOnNavMesh || Time.time < m_NextPathUpdate)
                return;
            if (NavMesh.SamplePosition(desired, out NavMeshHit hit, sampleRadius, NavMesh.AllAreas))
            {
                m_NextPathUpdate = Time.time + c_PathUpdateInterval;
                NavMeshAgent.stoppingDistance = 0f;
                NavMeshAgent.isStopped = false;
                NavMeshAgent.SetDestination(hit.position);
            }
        }

        private float GetTargetImpactHeight()
        {
            if (Target != null)
            {
                Collider targetCollider = Target.GetComponent<Collider>();
                if (targetCollider != null)
                    return targetCollider.bounds.center.y;
            }
            return SquadBlackboard.PlayerPosition.y + 0.85f;
        }

        /// <summary>
        /// 在炮口固定仰角下解抛体公式：
        /// v² = g*x² / (2*cos²(angle)*(x*tan(angle)-heightDifference))。
        /// 同时返回水平飞行时间，供下一轮移动目标预测使用。
        /// </summary>
        private bool CalculateBallisticSolution(
            Vector3 targetPoint, out float launchForce, out float flightTime)
        {
            Vector3 displacement = targetPoint - FireTransform.position;
            Vector3 flat = displacement;
            flat.y = 0f;
            float horizontalDistance = flat.magnitude;

            Vector3 fireForward = FireTransform.forward;
            float forwardHorizontal = new Vector3(
                fireForward.x, 0f, fireForward.z).magnitude;
            float launchAngle = Mathf.Atan2(fireForward.y, Mathf.Max(0.001f, forwardHorizontal));
            float cosine = Mathf.Cos(launchAngle);
            float heightDifference = displacement.y;
            float denominator = 2f * cosine * cosine *
                                (horizontalDistance * Mathf.Tan(launchAngle) - heightDifference);

            if (denominator <= 0.001f || horizontalDistance <= 0.01f)
            {
                launchForce = LaunchForceMinMax.y;
                flightTime = 0f;
                return false;
            }

            float requiredForce = Mathf.Sqrt(
                Mathf.Abs(Physics.gravity.y) * horizontalDistance * horizontalDistance / denominator);
            launchForce = Mathf.Clamp(requiredForce, LaunchForceMinMax.x, LaunchForceMinMax.y);
            float horizontalSpeed = launchForce * Mathf.Max(0.001f, cosine);
            flightTime = Mathf.Clamp(horizontalDistance / horizontalSpeed, 0f, 2f);
            return requiredForce <= LaunchForceMinMax.y + 0.01f;
        }

        /// <summary>
        /// Method <c>LaunchProjectile</c> instantiate and launch the shell.
        /// </summary>
        public void LaunchProjectile(float launchForce = 1f)
        {
            launchForce = Mathf.Min(Mathf.Max(LaunchForceMinMax.x, launchForce), LaunchForceMinMax.y);

            // Set the fired flag so only Fire is only called once.
            // m_Fired = true;

            // Create an instance of the shell and store a reference to it's rigidbody.
            Rigidbody shellInstance = Instantiate(Shell, FireTransform.position, FireTransform.rotation) as Rigidbody;

            // Set the shell's velocity to the launch force in the fire position's forward direction.
            shellInstance.velocity = launchForce * FireTransform.forward; ;

            // Change the clip to the firing clip and play it.
            SFXAudioSource.clip = ShotFiringAudioClip;
            SFXAudioSource.Play();
        }
    }
}
