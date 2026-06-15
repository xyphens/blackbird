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

        public static string FormatThrottle(double throttlePercent)
        {
            if (double.IsNaN(throttlePercent) || double.IsInfinity(throttlePercent)) return "N/A";
            return throttlePercent <= 0.0 ? "cutoff" : (throttlePercent * 100).ToString("F0") + "%";
        }
    }
}
