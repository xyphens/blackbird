using System;
using System.Collections.Generic;

namespace Blackbird.Psg
{
    // Per-body J2 oblateness. Values from Principia's gravity models (astronomy/sol_gravity_model.proto.txt):
    // J2 = -sqrt(5) * C-bar(2,0) (the degree-2 zonal `cos`), paired with the geopotential reference_radius
    // (EQUATORIAL, not KSP's mean Radius). Bodies not listed -> J2 = 0 (Principia models stock bodies as
    // point masses), so the entire J2 path no-ops in stock.
    public static class BodyOblateness
    {
        public struct Oblateness { public double J2; public double ReferenceRadiusMeters; }

        // Earth: J2 = -sqrt(5) * C-bar(2,0); C-bar(2,0) = -4.8416945732e-04 (sol_gravity_model.proto.txt, degree 2 order 0 cos).
        // Re = reference_radius 6378.1363 km.
        private static readonly Dictionary<string, Oblateness> Table =
            new Dictionary<string, Oblateness>(StringComparer.OrdinalIgnoreCase)
        {
            { "Earth", new Oblateness { J2 = 1.082636e-03, ReferenceRadiusMeters = 6378136.3 } },
            // Add Moon/Mars/etc. from sol_gravity_model.proto.txt as targets expand.
        };

        public static Oblateness For(CelestialBody body)
        {
            if (body != null && body.bodyName != null && Table.TryGetValue(body.bodyName, out Oblateness o))
                return o;
            return new Oblateness { J2 = 0.0, ReferenceRadiusMeters = body != null ? body.Radius : 0.0 };
        }
    }
}