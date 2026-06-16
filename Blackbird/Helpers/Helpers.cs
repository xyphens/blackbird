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
    }
}
