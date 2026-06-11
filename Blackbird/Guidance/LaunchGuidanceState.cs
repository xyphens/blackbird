using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Blackbird.Guidance
{
    public enum LaunchGuidanceState
    {
        Idle,
        PlanReady,
        PlanAccepted,
        WarpingToLaunch,
        AwaitingLaunch,
        GuidingAscent,
        Complete,
        Aborted
    }
}
