namespace Blackbird.Guidance
{
    public sealed class PsgOptimizationResult
    {
        public bool Success { get; set; }
        public string Status { get; set; }
        public PsgSolution Solution { get; set; }
        public int Iterations { get; set; }
        public int TerminationType { get; set; }
        public double ConstraintViolation { get; set; }
    }
}
