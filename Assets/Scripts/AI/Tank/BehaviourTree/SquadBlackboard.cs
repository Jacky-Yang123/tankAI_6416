using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace CE6127.Tanks.AI
{
    internal enum SquadMode
    {
        Encircle,
        Intercept,
        Herd,
        CorneredAttack,
        AllIn
    }

    internal enum SquadRole
    {
        Pressure,
        LeftInterceptor,
        RightInterceptor
    }

    internal struct SquadOrder
    {
        public Vector3 Slot;
        public bool HasSlot;
        public bool MustEvade;
        public SquadMode Mode;
        public SquadRole Role;
        public int SlotIndex;
    }

    /// <summary>
    /// 三辆 AI 共享的感知和指挥黑板。所有位置都从现有 NavMesh 动态计算，
    /// 不依赖地图中的固定建筑坐标，也不修改任何坦克或地图数值。
    /// </summary>
    internal static class SquadBlackboard
    {
        // 玩家爆炸半径为5；11米使两车不会被同一次爆炸覆盖。
        public const float MinimumAllyDistance = 11f;
        public const float MaximumAttackDistance = 30f;

        private static readonly float[] s_InterceptHorizons = { 0.75f, 1.25f, 1.75f, 2.5f, 3.25f };
        private static readonly int[,] s_Permutations =
        {
            { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 },
            { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 }
        };

        public static Transform Player { get; private set; }
        public static Transform PlayerFireTransform { get; private set; }
        public static Vector3 PlayerPosition { get; private set; }
        public static Vector3 PlayerVelocity { get; private set; }
        public static float PlayerTurnRate { get; private set; }
        public static Vector3 PlayerForward { get; private set; }
        public static Vector3 EscapeDirection { get; private set; }
        public static float PlayerChargeRatio { get; private set; }
        public static bool PlayerCoolingDown { get; private set; }
        public static bool PlayerEscaping { get; private set; }
        public static float EscapeConfidence { get; private set; }
        public static float RemainingRoundTime { get; private set; }
        public static float TimePressure { get; private set; }
        public static float CombatRadius { get; private set; } = 17f;
        public static SquadMode Mode { get; private set; }
        public static readonly List<Rigidbody> PlayerShells = new();

        private static readonly Dictionary<int, SquadOrder> s_Orders = new();
        private static readonly HashSet<int> s_KnownPlayerShells = new();
        private static int s_LastFrame = -1;
        private static Transform s_PreviousPlayer;
        private static Vector3 s_PreviousPlayerPosition;
        private static float s_PreviousVelocitySampleTime;
        private static Vector3 s_SmoothedPlayerVelocity;
        private static Vector3 s_PreviousMovementDirection;
        private static float s_SmoothedPlayerTurnRate;
        private static Vector3 s_LastEscapeDirection;
        private static float s_PreviousSquadDistance;
        private static float s_LastPlayerShotTime = -99f;
        private static float s_RoundEndTime;
        private static float s_RoundDuration = 60f;
        private static float s_NextTacticalUpdate;
        private static bool s_WasRoundPlaying;

        private struct RoleTarget
        {
            public Vector3 Position;
            public float Cost;
            public bool IsTrueIntercept;
        }

        public static SquadOrder GetOrder(TankSM tank)
        {
            Update(tank.GameManager);
            return s_Orders.TryGetValue(tank.GetInstanceID(), out SquadOrder order)
                ? order
                : new SquadOrder
                {
                    HasSlot = false,
                    Mode = SquadMode.Encircle,
                    Role = SquadRole.Pressure
                };
        }

        /// <summary>每帧只更新一次感知；较昂贵的路径与职责分配每0.2秒更新一次。</summary>
        public static void Update(GameManager gameManager)
        {
            if (s_LastFrame == Time.frameCount)
                return;
            s_LastFrame = Time.frameCount;

            if (!s_WasRoundPlaying && gameManager.IsRoundPlaying)
                ResetForNewRound(gameManager);
            s_WasRoundPlaying = gameManager.IsRoundPlaying;
            RemainingRoundTime = Mathf.Max(0f, s_RoundEndTime - Time.time);
            TimePressure = 1f - Mathf.Clamp01(RemainingRoundTime / Mathf.Max(1f, s_RoundDuration));

            if (gameManager.PlayerPlatoon.Tanks.Count == 0 ||
                gameManager.PlayerPlatoon.Tanks[0].Instance == null)
                return;

            ReadPlayer(gameManager);
            ReadPlayerWeapon();
            List<TankSM> tanks = ReadLivingSquad(gameManager, out Vector3 squadCentre);
            if (tanks.Count == 0)
                return;

            UpdateEscapeDetection(squadCentre);
            CombatRadius = SelectCombatRadius();

            if (Time.time >= s_NextTacticalUpdate)
            {
                s_NextTacticalUpdate = Time.time + 0.2f;
                AssignTacticalOrders(tanks, gameManager.Speed);
            }

            // 先清掉上一帧的规避命令，再只指定炮口夹角最小的一辆车规避。
            foreach (TankSM tank in tanks)
            {
                if (!s_Orders.TryGetValue(tank.GetInstanceID(), out SquadOrder order))
                    continue;
                order.MustEvade = false;
                s_Orders[tank.GetInstanceID()] = order;
            }
            AssignSingleChargeEvader(tanks);
        }

        private static void ResetForNewRound(GameManager gameManager)
        {
            s_RoundDuration = Mathf.Max(1f, gameManager.MinutesPerRound * 60f);
            s_RoundEndTime = Time.time + s_RoundDuration;
            s_NextTacticalUpdate = 0f;
            s_PreviousSquadDistance = 0f;
            EscapeConfidence = 0f;
            PlayerEscaping = false;
            s_SmoothedPlayerVelocity = Vector3.zero;
            s_PreviousPlayer = null;
            s_PreviousVelocitySampleTime = 0f;
            s_PreviousMovementDirection = Vector3.zero;
            s_SmoothedPlayerTurnRate = 0f;
            s_LastEscapeDirection = Vector3.zero;
            s_KnownPlayerShells.Clear();
            PlayerShells.Clear();
            s_Orders.Clear();
        }

        private static void ReadPlayer(GameManager gameManager)
        {
            Player = gameManager.PlayerPlatoon.Tanks[0].Instance.transform;
            TankShooting shooting = Player.GetComponent<TankShooting>();
            PlayerFireTransform = shooting != null ? shooting.FireTransform : Player;
            PlayerPosition = Player.position;

            Vector3 flatForward = Player.forward;
            flatForward.y = 0f;
            PlayerForward = flatForward.sqrMagnitude > 0.01f ? flatForward.normalized : Vector3.forward;

            // 玩家由 MovePosition 驱动，只用连续帧位移估速，不读取 Rigidbody.velocity。
            bool velocityUpdated = false;
            if (s_PreviousPlayer != Player || s_PreviousVelocitySampleTime <= 0f)
            {
                s_PreviousPlayer = Player;
                s_PreviousPlayerPosition = PlayerPosition;
                s_PreviousVelocitySampleTime = Time.time;
                s_SmoothedPlayerVelocity = Vector3.zero;
            }
            else
            {
                // MovePosition 在 FixedUpdate 中改变位置。若逐渲染帧估速，会在两个
                // 物理帧之间反复读到0，严重低估直线速度；改用至少0.08秒的窗口。
                float sampleDuration = Time.time - s_PreviousVelocitySampleTime;
                if (sampleDuration >= 0.08f)
                {
                    Vector3 measured = (PlayerPosition - s_PreviousPlayerPosition) / sampleDuration;
                    measured.y = 0f;
                    if (measured.magnitude <= gameManager.Speed * 1.5f)
                        s_SmoothedPlayerVelocity = Vector3.Lerp(
                            s_SmoothedPlayerVelocity, measured, 0.7f);
                    else
                        s_SmoothedPlayerVelocity = Vector3.zero;
                    s_PreviousPlayerPosition = PlayerPosition;
                    s_PreviousVelocitySampleTime = Time.time;
                    velocityUpdated = true;
                }
            }
            PlayerVelocity = s_SmoothedPlayerVelocity;

            // 玩家只能沿车头移动。记录实际移动方向的角速度，供炮弹预瞄预测弧线。
            if (velocityUpdated && PlayerVelocity.magnitude > 4f)
            {
                Vector3 movementDirection = PlayerVelocity.normalized;
                if (s_PreviousMovementDirection.sqrMagnitude > 0.1f && Time.deltaTime > 0f)
                {
                    float measuredTurnRate = Vector3.SignedAngle(
                        s_PreviousMovementDirection, movementDirection, Vector3.up) / Time.deltaTime;
                    measuredTurnRate = Mathf.Clamp(measuredTurnRate, -180f, 180f);
                    s_SmoothedPlayerTurnRate = Mathf.Lerp(
                        s_SmoothedPlayerTurnRate, measuredTurnRate, 0.2f);
                }
                s_PreviousMovementDirection = movementDirection;
            }
            else if (velocityUpdated)
            {
                s_SmoothedPlayerTurnRate = Mathf.Lerp(s_SmoothedPlayerTurnRate, 0f, 0.25f);
            }
            PlayerTurnRate = s_SmoothedPlayerTurnRate;
        }

        /// <summary>
        /// 按玩家当前实际速度和转向角速度预测未来位置。使用中点方向近似弧线，
        /// 比单纯 velocity * time 更适合只能沿车头前进的坦克。
        /// </summary>
        public static Vector3 PredictPlayerPosition(float futureTime)
        {
            futureTime = Mathf.Clamp(futureTime, 0f, 2f);
            Vector3 velocity = PlayerVelocity;
            velocity.y = 0f;
            if (velocity.sqrMagnitude < 0.25f)
                return PlayerPosition;

            float halfTurn = PlayerTurnRate * futureTime * 0.5f;
            Vector3 midpointVelocity = Quaternion.AngleAxis(halfTurn, Vector3.up) * velocity;
            return PlayerPosition + midpointVelocity * futureTime;
        }

        private static void ReadPlayerWeapon()
        {
            TankShooting shooting = Player != null ? Player.GetComponent<TankShooting>() : null;
            PlayerChargeRatio = 0f;
            if (shooting != null && shooting.AimSlider != null)
            {
                PlayerChargeRatio = Mathf.InverseLerp(
                    shooting.LaunchForceMinMax.x,
                    shooting.LaunchForceMinMax.y,
                    shooting.AimSlider.value);
            }

            PlayerShells.Clear();
            foreach (ShellExplosion shell in Object.FindObjectsOfType<ShellExplosion>())
            {
                if (!shell.CompareTag("Player"))
                    continue;
                Rigidbody body = shell.GetComponent<Rigidbody>();
                if (body != null)
                    PlayerShells.Add(body);
                if (s_KnownPlayerShells.Add(shell.GetInstanceID()))
                    s_LastPlayerShotTime = Time.time;
            }
            PlayerCoolingDown = shooting != null && Time.time - s_LastPlayerShotTime < shooting.CooldownTime;
        }

        private static List<TankSM> ReadLivingSquad(GameManager gameManager, out Vector3 centre)
        {
            List<TankSM> tanks = new();
            centre = Vector3.zero;
            foreach (TankManager manager in gameManager.AIPlatoon.Tanks)
            {
                if (manager.Instance == null || !manager.Instance.activeInHierarchy)
                    continue;
                TankSM tank = manager.Instance.GetComponent<TankSM>();
                if (tank == null)
                    continue;
                tanks.Add(tank);
                centre += tank.transform.position;
            }
            if (tanks.Count > 0)
                centre /= tanks.Count;
            return tanks;
        }

        private static void UpdateEscapeDetection(Vector3 squadCentre)
        {
            Vector3 awayFromSquad = PlayerPosition - squadCentre;
            awayFromSquad.y = 0f;
            float squadDistance = awayFromSquad.magnitude;
            bool distanceGrowing = s_PreviousSquadDistance > 0f &&
                                   squadDistance > s_PreviousSquadDistance + 0.01f;
            s_PreviousSquadDistance = squadDistance;

            Vector3 movementDirection = PlayerVelocity.sqrMagnitude > 1f
                ? PlayerVelocity.normalized
                : PlayerForward;
            float escapeDot = PlayerVelocity.sqrMagnitude > 1f && awayFromSquad.sqrMagnitude > 1f
                ? Vector3.Dot(movementDirection, awayFromSquad.normalized)
                : -1f;
            bool escapingNow = PlayerVelocity.magnitude >= 8f && escapeDot >= 0.55f && distanceGrowing;

            // 连续约0.4秒即可确认逃跑；停止或转向时以三倍速度取消旧判断。
            float confidenceChange = escapingNow ? Time.deltaTime / 0.4f : -Time.deltaTime / 0.13f;
            EscapeConfidence = Mathf.Clamp01(EscapeConfidence + confidenceChange);
            PlayerEscaping = EscapeConfidence >= 0.9f;
            EscapeDirection = PlayerVelocity.magnitude > 2f ? movementDirection : PlayerForward;

            if (s_LastEscapeDirection.sqrMagnitude > 0.1f &&
                Vector3.Angle(s_LastEscapeDirection, EscapeDirection) > 45f)
            {
                // 急转弯后立即废弃旧拦截点，不等待0.2秒定时器。
                s_NextTacticalUpdate = 0f;
                EscapeConfidence = Mathf.Min(EscapeConfidence, 0.45f);
                PlayerEscaping = false;
            }
            s_LastEscapeDirection = EscapeDirection;
        }

        private static float SelectCombatRadius()
        {
            float radius;
            if (RemainingRoundTime > 45f)
                radius = 17f;
            else if (RemainingRoundTime > 20f)
                radius = 15f;
            else
                radius = 12.75f;

            if (PlayerCoolingDown)
                radius -= RemainingRoundTime > 20f ? 1f : 0.5f;
            return Mathf.Max(11.5f, radius);
        }

        private static void AssignTacticalOrders(List<TankSM> tanks, float movementSpeed)
        {
            bool allIn = RemainingRoundTime <= 20f;
            bool fleeing = PlayerEscaping || (allIn && PlayerVelocity.magnitude >= 8f);
            Vector3 direction = EscapeDirection.sqrMagnitude > 0.01f ? EscapeDirection : PlayerForward;
            Vector3 right = Vector3.Cross(Vector3.up, direction).normalized;

            bool edgeFound = NavMesh.Raycast(PlayerPosition,
                PlayerPosition + direction * 60f, out NavMeshHit edgeHit, NavMesh.AllAreas);
            float edgeDistance = edgeFound ? Vector3.Distance(PlayerPosition, edgeHit.position) : 60f;
            SquadMode requestedMode = allIn
                ? SquadMode.AllIn
                : fleeing && edgeDistance < 18f
                    ? SquadMode.CorneredAttack
                    : fleeing ? SquadMode.Intercept : SquadMode.Encircle;

            int count = Mathf.Min(3, tanks.Count);
            float bestCost = float.MaxValue;
            int bestRow = 0;
            RoleTarget[,] targets = new RoleTarget[count, 3];

            for (int tankIndex = 0; tankIndex < count; ++tankIndex)
            {
                for (int roleIndex = 0; roleIndex < 3; ++roleIndex)
                {
                    targets[tankIndex, roleIndex] = BuildRoleTarget(
                        tanks[tankIndex], (SquadRole)roleIndex, requestedMode,
                        fleeing, direction, right, edgeFound, edgeHit.position, movementSpeed);
                }
            }

            for (int row = 0; row < s_Permutations.GetLength(0); ++row)
            {
                float cost = 0f;
                for (int tankIndex = 0; tankIndex < count; ++tankIndex)
                {
                    int roleIndex = s_Permutations[row, tankIndex];
                    cost += targets[tankIndex, roleIndex].Cost;
                    if (s_Orders.TryGetValue(tanks[tankIndex].GetInstanceID(), out SquadOrder oldOrder) &&
                        oldOrder.HasSlot && oldOrder.Role != (SquadRole)roleIndex)
                        cost += 5f;
                }
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestRow = row;
                }
            }

            bool missingIntercept = false;
            Vector3[] chosenPositions = new Vector3[count];
            SquadRole[] chosenRoles = new SquadRole[count];
            for (int tankIndex = 0; tankIndex < count; ++tankIndex)
            {
                int roleIndex = s_Permutations[bestRow, tankIndex];
                chosenRoles[tankIndex] = (SquadRole)roleIndex;
                chosenPositions[tankIndex] = targets[tankIndex, roleIndex].Position;
                if (fleeing && roleIndex != (int)SquadRole.Pressure &&
                    !targets[tankIndex, roleIndex].IsTrueIntercept)
                    missingIntercept = true;
            }

            Mode = requestedMode;
            if (!allIn && fleeing && missingIntercept && requestedMode != SquadMode.CorneredAttack)
                Mode = SquadMode.Herd;

            EnsureDifferentDestinations(chosenPositions, chosenRoles, right);
            for (int tankIndex = 0; tankIndex < count; ++tankIndex)
            {
                s_Orders[tanks[tankIndex].GetInstanceID()] = new SquadOrder
                {
                    Slot = chosenPositions[tankIndex],
                    HasSlot = true,
                    MustEvade = false,
                    Mode = Mode,
                    Role = chosenRoles[tankIndex],
                    SlotIndex = (int)chosenRoles[tankIndex]
                };
            }
        }

        private static RoleTarget BuildRoleTarget(
            TankSM tank,
            SquadRole role,
            SquadMode mode,
            bool fleeing,
            Vector3 direction,
            Vector3 right,
            bool edgeFound,
            Vector3 edgePosition,
            float movementSpeed)
        {
            if (!fleeing)
            {
                float baseAngle = role == SquadRole.Pressure ? 180f :
                    role == SquadRole.LeftInterceptor ? 60f : -60f;
                // 整个三角阵型共同旋转，因此三车持续大幅换位但始终保持120度关系。
                float orbitSpeed = RemainingRoundTime <= 20f ? 24f : 18f;
                float angle = baseAngle + Time.time * orbitSpeed;
                Vector3 desired = PlayerPosition +
                                  Quaternion.AngleAxis(angle, Vector3.up) * PlayerForward * CombatRadius;
                Vector3 slot = SampleWalkable(desired, PlayerPosition, (int)role, 8f);
                return new RoleTarget
                {
                    Position = slot,
                    Cost = PathLength(tank.transform.position, slot),
                    IsTrueIntercept = false
                };
            }

            if (role == SquadRole.Pressure)
            {
                float pressureDistance = RemainingRoundTime > 45f ? 18f :
                    RemainingRoundTime > 20f ? 16f : 13f;
                Vector3 desired = PlayerPosition - direction * pressureDistance;
                Vector3 slot = SampleWalkable(desired, PlayerPosition, 0, 8f);
                return new RoleTarget
                {
                    Position = slot,
                    Cost = PathLength(tank.transform.position, slot),
                    IsTrueIntercept = false
                };
            }

            float sideSign = role == SquadRole.LeftInterceptor ? -1f : 1f;
            if (TryFindInterceptPoint(tank, direction, right * sideSign, movementSpeed,
                    out Vector3 intercept, out float interceptCost))
            {
                return new RoleTarget
                {
                    Position = intercept,
                    Cost = interceptCost,
                    IsTrueIntercept = true
                };
            }

            Vector3 anchor = edgeFound
                ? edgePosition - direction * 3f
                : PlayerPosition + direction * movementSpeed * 3.25f;
            float sideDistance = mode == SquadMode.AllIn ? 7.5f : 10f;
            Vector3 herd = SampleWalkable(anchor + right * sideSign * sideDistance,
                PlayerPosition, (int)role, 10f);
            return new RoleTarget
            {
                Position = herd,
                Cost = PathLength(tank.transform.position, herd) + 35f,
                IsTrueIntercept = false
            };
        }

        /// <summary>只接受 AI 能比玩家至少早0.25秒到达的拦截点。</summary>
        private static bool TryFindInterceptPoint(
            TankSM tank,
            Vector3 direction,
            Vector3 side,
            float movementSpeed,
            out Vector3 bestPoint,
            out float bestCost)
        {
            bestPoint = Vector3.zero;
            bestCost = float.MaxValue;
            bool found = false;
            float sideDistance = RemainingRoundTime <= 20f ? 7.5f : 9.5f;

            foreach (float horizon in s_InterceptHorizons)
            {
                Vector3 projected = PlayerPosition + direction * movementSpeed * horizon;
                if (NavMesh.Raycast(PlayerPosition, projected, out NavMeshHit edge, NavMesh.AllAreas))
                    projected = edge.position - direction * 2.5f;
                Vector3 routePoint = SampleWalkable(projected, PlayerPosition, 1, 5f);
                float playerPath = PathLength(PlayerPosition, routePoint);
                if (playerPath >= 9999f)
                    continue;
                float playerEta = playerPath / Mathf.Max(0.1f, movementSpeed);

                Vector3 candidate = SampleWalkable(routePoint + side * sideDistance,
                    PlayerPosition, side.x + side.z >= 0f ? 1 : 2, 6f);
                float aiPath = PathLength(tank.transform.position, candidate);
                if (aiPath >= 9999f)
                    continue;
                float aiEta = aiPath / Mathf.Max(0.1f, movementSpeed);
                float etaMargin = playerEta - aiEta;
                if (etaMargin < 0.25f)
                    continue;

                float score = aiPath - etaMargin * movementSpeed * 1.5f + horizon;
                if (score >= bestCost)
                    continue;
                bestCost = score;
                bestPoint = candidate;
                found = true;
            }
            return found;
        }

        private static void EnsureDifferentDestinations(
            Vector3[] positions, SquadRole[] roles, Vector3 right)
        {
            for (int i = 0; i < positions.Length; ++i)
            {
                float currentMinimum = MinimumDistanceToEarlierSlots(positions, i, positions[i]);
                if (currentMinimum >= MinimumAllyDistance)
                    continue;

                Vector3 original = positions[i];
                Vector3 best = original;
                float bestScore = currentMinimum;
                // 狭窄处的单次采样可能把两个理论点吸到同一块 NavMesh。
                // 围绕原目标搜索一圈，选择与所有既有槽位最远的可行点。
                for (int attempt = 0; attempt < 12; ++attempt)
                {
                    float angle = attempt * 30f + (int)roles[i] * 120f;
                    Vector3 offset = Quaternion.AngleAxis(angle, Vector3.up) *
                                     right * (MinimumAllyDistance + 2f);
                    Vector3 desired = original + offset;
                    if (!NavMesh.SamplePosition(desired, out NavMeshHit hit, 3f, NavMesh.AllAreas))
                        continue;
                    float minimum = MinimumDistanceToEarlierSlots(positions, i, hit.position);
                    float score = minimum - Vector3.Distance(original, hit.position) * 0.15f;
                    if (score <= bestScore)
                        continue;
                    bestScore = score;
                    best = hit.position;
                    if (minimum >= MinimumAllyDistance + 1f)
                        break;
                }
                positions[i] = best;
            }
        }

        private static float MinimumDistanceToEarlierSlots(
            Vector3[] positions, int endExclusive, Vector3 candidate)
        {
            float minimum = float.MaxValue;
            for (int previous = 0; previous < endExclusive; ++previous)
                minimum = Mathf.Min(minimum, Vector3.Distance(candidate, positions[previous]));
            return minimum;
        }

        private static void AssignSingleChargeEvader(List<TankSM> tanks)
        {
            if (PlayerChargeRatio <= 0.05f || PlayerFireTransform == null)
                return;

            Vector3 fireForward = PlayerFireTransform.forward;
            fireForward.y = 0f;
            if (fireForward.sqrMagnitude < 0.01f)
                return;
            fireForward.Normalize();

            TankSM threatened = null;
            float smallestAngle = 14f;
            foreach (TankSM tank in tanks)
            {
                Vector3 toTank = tank.transform.position - PlayerPosition;
                toTank.y = 0f;
                if (toTank.sqrMagnitude < 0.01f)
                    continue;
                float angle = Vector3.Angle(fireForward, toTank.normalized);
                if (angle >= smallestAngle)
                    continue;
                smallestAngle = angle;
                threatened = tank;
            }

            if (threatened != null &&
                s_Orders.TryGetValue(threatened.GetInstanceID(), out SquadOrder order))
            {
                order.MustEvade = true;
                s_Orders[threatened.GetInstanceID()] = order;
            }
        }

        private static Vector3 SampleWalkable(
            Vector3 desired, Vector3 centre, int index, float sampleRadius)
        {
            if (NavMesh.SamplePosition(desired, out NavMeshHit hit, sampleRadius, NavMesh.AllAreas))
                return hit.position;

            Vector3 fallback = centre +
                Quaternion.AngleAxis(index * 120f, Vector3.up) * Vector3.forward * CombatRadius;
            return NavMesh.SamplePosition(fallback, out hit, 12f, NavMesh.AllAreas)
                ? hit.position
                : centre;
        }

        private static float PathLength(Vector3 start, Vector3 end)
        {
            NavMeshPath path = new();
            if (!NavMesh.CalculatePath(start, end, NavMesh.AllAreas, path) ||
                path.status != NavMeshPathStatus.PathComplete)
                return 10000f;

            float length = 0f;
            for (int i = 1; i < path.corners.Length; ++i)
                length += Vector3.Distance(path.corners[i - 1], path.corners[i]);
            return length;
        }
    }
}
