using System;
using System.Collections.Generic;

namespace CE6127.Tanks.AI
{
    /// <summary>行为树节点每次执行后的三种结果。</summary>
    internal enum BTStatus
    {
        Success, // 本节点本帧完成。
        Failure, // 条件不满足，让父节点尝试其他分支。
        Running  // 行为仍在执行，下帧继续。
    }

    /// <summary>所有行为树节点的共同接口。</summary>
    internal abstract class BTNode
    {
        public abstract BTStatus Tick();
    }

    /// <summary>条件节点：条件成立返回 Success，否则返回 Failure。</summary>
    internal sealed class BTCondition : BTNode
    {
        private readonly Func<bool> m_Condition;
        public BTCondition(Func<bool> condition) => m_Condition = condition;
        public override BTStatus Tick() => m_Condition() ? BTStatus.Success : BTStatus.Failure;
    }

    /// <summary>动作节点：执行坦克的一个具体动作。</summary>
    internal sealed class BTAction : BTNode
    {
        private readonly Func<BTStatus> m_Action;
        public BTAction(Func<BTStatus> action) => m_Action = action;
        public override BTStatus Tick() => m_Action();
    }

    /// <summary>顺序节点：所有子节点依次成功，整个节点才成功。</summary>
    internal sealed class BTSequence : BTNode
    {
        private readonly IReadOnlyList<BTNode> m_Children;
        public BTSequence(params BTNode[] children) => m_Children = children;

        public override BTStatus Tick()
        {
            foreach (BTNode child in m_Children)
            {
                BTStatus result = child.Tick();
                if (result != BTStatus.Success)
                    return result;
            }
            return BTStatus.Success;
        }
    }

    /// <summary>选择节点：按优先级执行，第一个不是 Failure 的子节点获胜。</summary>
    internal sealed class BTSelector : BTNode
    {
        private readonly IReadOnlyList<BTNode> m_Children;
        public BTSelector(params BTNode[] children) => m_Children = children;

        public override BTStatus Tick()
        {
            foreach (BTNode child in m_Children)
            {
                BTStatus result = child.Tick();
                if (result != BTStatus.Failure)
                    return result;
            }
            return BTStatus.Failure;
        }
    }

    /// <summary>
    /// 并行节点：每帧执行所有子节点。用于让移动分支和战斗分支同时工作，
    /// 而不是等待坦克停下后才瞄准。
    /// </summary>
    internal sealed class BTParallel : BTNode
    {
        private readonly IReadOnlyList<BTNode> m_Children;
        public BTParallel(params BTNode[] children) => m_Children = children;

        public override BTStatus Tick()
        {
            bool anyRunning = false;
            foreach (BTNode child in m_Children)
                anyRunning |= child.Tick() == BTStatus.Running;

            return anyRunning ? BTStatus.Running : BTStatus.Success;
        }
    }
}
