using System;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using UnityEngine;

namespace Blackbird.Planning
{
    public sealed class Trajectory
    {
        private Texture2D _tex;
        private int _texW;
        private int _texH;
        private double _cacheKey = double.NaN;
        private double _cacheTgt = double.NaN;

        private GUIStyle _peakStyle;
        private GUIStyle _tgtStyle;
        private GUIStyle _downStyle;

        private static readonly Color32 Background = new Color32(12, 12, 20, 220);
        private static readonly Color32 Grid = new Color32(40, 44, 55, 255);
        private static readonly Color32 PathColor = new Color32(77, 204, 255, 255);
        private static readonly Color32 TargetColor = new Color32(255, 178, 51, 255);

        public double PeakAltitudeMeters { get; private set; } = double.NaN;
        public double MaxLoftAboveTargetMeters { get; private set; } = double.NaN;
        public double DownrangeMeters { get; private set; } = double.NaN;
        private float _lastBuildRt = -999f;
        private const float MinRebuildSeconds = 0.4f;

        // refreshed OnGUI.  reserves the layout space and caches the plot in memory to rebuild only when solution or target Ap changes

        public void Draw(AscentPath path, double bodyRadius, double targetApAlt, int width, int height)
        {
            Rect rect = GUILayoutUtility.GetRect(width, height, GUILayout.ExpandWidth(false));

            if (path == null || !path.IsValid)
            {
                PeakAltitudeMeters = double.NaN;
                MaxLoftAboveTargetMeters = double.NaN;
                GUI.Box(rect, "No predicted trajectory");
                return;
            }

            float rt = Time.realtimeSinceStartup;
            bool sizeChanged = _tex == null || _texW != width || _texH != height;
            bool dataChanged = path.CreatedUniversalTime != _cacheKey || targetApAlt != _cacheTgt;
            if (sizeChanged || (dataChanged && rt - _lastBuildRt >= MinRebuildSeconds))
            {
                Rebuild(path, bodyRadius, targetApAlt, width, height);
                _cacheKey = path.CreatedUniversalTime;
                _cacheTgt = targetApAlt;
                _lastBuildRt = rt;
            }

            if (_tex != null) GUI.DrawTexture(rect, _tex, ScaleMode.StretchToFill, true);
            DrawLabels(rect, targetApAlt);
        }

        private void Rebuild(AscentPath path, double bodyRadius, double targetApAlt, int width, int height)
        {
            AscentPathPoint[] pts = path.Points;
            width = Mathf.Max(width, 16);
            height = Mathf.Max(height, 16);

            //PsgSolutionPoint[] pts = solution.Points;
            int n = pts.Length;

            double[] alt = new double[n];
            double[] down = new double[n];

            double maxAlt = targetApAlt;
            double maxDown = 1.0;
            PeakAltitudeMeters = double.NegativeInfinity;

            for (int i = 0; i < n; i++)
            {
                alt[i] = pts[0].RelativePosition.magnitude - bodyRadius;
                double d = Vector3d.Dot(pts[0].RelativePosition.normalized, pts[0].RelativePosition.normalized);
                //d = d > 1.0 ? 1.0 : (d < -1.0 ? -1.0 : d);
                d = MathHelpers.Clamp(d, -1.0, 1.0);
                down[i] = Math.Acos(d) * bodyRadius;
                if (alt[i] > PeakAltitudeMeters) PeakAltitudeMeters = alt[i];
                if (alt[i] > maxAlt) maxAlt = alt[i];
                if (down[i] > maxDown) maxDown = down[i];
            }

            MaxLoftAboveTargetMeters = PeakAltitudeMeters - targetApAlt;
            DownrangeMeters = down[n - 1];
            maxAlt *= 1.08; // padding so peak isn't clipped to top edge

            // render background
            var buf = new Color32[width * height]; // buffer
            for (int i = 0; i < buf.Length; i++) buf[i] = Background;

            // gridlines
            for (int g = 0; g < 4; g++)
            {
                int gx = g * width / 4;
                int gy = g * height / 4;
                for (int y = 0; y < height; y++) buf[y * width + gx] = Grid;
                for (int x = 0; x < width; x++) buf[gy * width + x] = Grid;
            }

            // target Ap dashed reference
            int ty = (int)(targetApAlt / maxAlt * (height - 1));
            if (ty >= 0 && ty < height)
            {
                for (int x = 0; x < width; x += 6)
                {
                    for (int dx = 0; dx < 3 && x + dx < width; dx++)
                    {
                        buf[ty * width + x + dx] = TargetColor;
                    }
                }
            }

            // predicted path
            int px = 0, py = 0;
            for (int i = 0; i < n; i++)
            {
                int x = (int)(down[i] / maxDown * (width - 1));
                int y = (int)(alt[i] / maxAlt * (height - 1));
                if (i > 0) DrawLine(buf, width, height, px, py, x, y, PathColor);
                px = x;
                py = y;
            }

            // refresh + render
            if (_tex != null) UnityEngine.Object.Destroy(_tex);
            _tex = new Texture2D(width, height, TextureFormat.RGBA32, false) { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };
            _tex.SetPixels32(buf);
            _tex.Apply(false);
            _texW = width;
            _texH = height;
        }

        private static void DrawLine(Color32[] buf, int w, int h, int x0, int y0, int x1, int y1, Color32 c)
        {
            int dx = Math.Abs(x1 - x0), dy = Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;
            while (true)
            {
                if ((uint)x0 < (uint)w && (uint)y0 < (uint)h) buf[y0 * w + x0] = c;
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx) { err += dx; y0 += sy; }
            }
        }

        public void DrawLabels(Rect rect, double targetApAlt)
        {
            if (_peakStyle == null)
            {
                _peakStyle = new GUIStyle(GUI.skin.label) { fontSize = 10, normal = { textColor = Color.white } };
                _tgtStyle = new GUIStyle(_peakStyle) { alignment = TextAnchor.UpperRight, normal = { textColor = new Color(1f, 0.7f, 0.2f) } };
                _downStyle = new GUIStyle(_peakStyle) { alignment = TextAnchor.LowerRight };
            }

            if (!double.IsNaN(PeakAltitudeMeters) && !double.IsInfinity(PeakAltitudeMeters))
            {
                GUI.Label(new Rect(rect.x + 3, rect.y + 1, rect.width - 6, 14), $"tgt Ap {targetApAlt / 1000.0:F0} km", _tgtStyle);
            }

            if (!double.IsNaN(DownrangeMeters))
            {
                GUI.Label(new Rect(rect.x + 3, rect.yMax - 15, rect.width - 6, 14), $"downrange {DownrangeMeters / 1000.0:F0} km", _downStyle);
            }
                
        }
    }
}
