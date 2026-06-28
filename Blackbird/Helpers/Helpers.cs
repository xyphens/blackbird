using System;
using Blackbird;

namespace Blackbird.Helpers
{
    public static class BlackbirdHelpers
    {
        public static string FormatDuration(double seconds)
        {
            int totalSeconds = (int)Math.Round(seconds);
            int hours = totalSeconds / 3600;
            int minutes = (totalSeconds % 3600) / 60;
            int secs = totalSeconds % 60;

            return $"{hours:D2}:{minutes:D2}:{secs:D2}";
        }

        public static string FormatThrottle(double throttle)
        {
            if (double.IsNaN(throttle) || double.IsInfinity(throttle)) return "N/A";
            return throttle <= 0.0 ? "cutoff" : (throttle * 100).ToString("F0") + "%";
        }

        public static bool IsUsableEngine(PartModule module, bool checkIsEngineOnly)
        {
            if (module == null) return false;
            if (module is ModuleEngines e)
            {
                if (!checkIsEngineOnly && (!e.isEnabled || !e.EngineIgnited || e.flameout)) return false;

                // caller only wants to know if this is an engine (i.e., when calculating figuring out staging)
                return true;
            }

            return false;
        }

        public static double CurrentEngineThrust(Vessel v)
        {
            double t = 0.0;

            for (int i = 0; i < v.parts.Count; i++)
            {
                foreach (PartModule m in v.parts[i].Modules)
                {
                    if (m is ModuleEngines e) t += Math.Max(0.0, e.finalThrust);
                }
            }

            return t;
        }

        // get the vessel's actively-burning thrust
        public static bool EngineThrustActive(Vessel v)
        {
            double current = 0.0;
            for (int i = 0; i < v.parts.Count; i++)
            {
                for (int j = 0; j < v.parts[i].Modules.Count; j++)
                {
                    if (v.parts[i].Modules[j] is ModuleEngines e) current += Math.Max(0.0, e.finalThrust);
                }
            }

            return current > 0.0;
        }
    }
}
