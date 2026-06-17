namespace Blackbird.Guidance
{
    // Shared time-warp helper: picks a safe warp rate for the time remaining to an event and stops warp.
    // Used by both the launch countdown and the rendezvous "warp to closest approach". The rate ladder
    // backs off as the event approaches so we never blow past it.
    public static class WarpHelper
    {
        public static void SetSafeWarpRate(double secondsRemaining)
        {
            int rateIndex;

            if (secondsRemaining <= 15.0)
                rateIndex = 1;
            else if (secondsRemaining <= 60.0)
                rateIndex = 2;
            else if (secondsRemaining <= 180.0)
                rateIndex = 3;
            else if (secondsRemaining <= 600.0)
                rateIndex = 4;
            else if (secondsRemaining <= 1800.0)
                rateIndex = 5;
            else
                rateIndex = 6;

            TimeWarp.SetRate(rateIndex, true);
        }

        public static void Stop() => TimeWarp.SetRate(0, true);
    }
}
