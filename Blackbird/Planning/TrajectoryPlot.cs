using System;
using UnityEngine;

namespace Blackbird.Planning
{
    public sealed class TrajectoryPlot
    {
        private Texture2D _tex;
        private int _texW;
        private int _texH;
        private double _cacheTgt = double.NaN;

        private GUIStyle _yStyle, _xStyle, _tgtStyle;

        private static readonly Color32 Background = new Color32(12, 12, 20, 220);
        private static readonly Color32 Grid = new Color32(40, 44, 55, 255);
        private static readonly Color32 TargetColor = new Color32(255, 178, 51, 255);
        private static readonly Color32 HistoryColor = new Color32(255, 60, 60, 255);     // red = flown
        private static readonly Color32 PlanColor = new Color32(237, 237, 237, 255);     // gray = plan
        private static readonly Color32 ProjectionColor = new Color32(77, 204, 255, 255); // blue = projected

        public double PeakAltitudeMeters { get; private set; } = double.NaN;
        public double MaxLoftAboveTargetMeters { get; private set; } = double.NaN;
        public double DownrangeMeters { get; private set; } = double.NaN;
        // legend
        private double _maxAltMeters, _maxDownMeters;

        private float _lastBuildRt = -999f;
        private const float MinRebuildSeconds = 0.4f;

        // refreshed OnGUI.  reserves the layout space and caches the plot in memory to rebuild only when solution or target Ap changes

        public void Draw(double[] histAlt, double[] histDown, double[] projAlt, double[] projDown, double[] planAlt, double[] planDown, double targetApAlt, int width, int height)
        {
            Rect rect = GUILayoutUtility.GetRect(width, height, GUILayout.ExpandWidth(false));

            if (!((histAlt != null && histAlt.Length >= 2) || (projAlt != null && projAlt.Length >= 2)))
            {
                PeakAltitudeMeters = double.NaN;
                MaxLoftAboveTargetMeters = double.NaN;
                GUI.Box(rect, "No trajectory yet");
                return;
            }

            float rt = Time.realtimeSinceStartup;
            bool sizeChanged = _tex == null || _texW != width || _texH != height;
            if (sizeChanged || rt - _lastBuildRt >= MinRebuildSeconds) // projection is live
            {
                Rebuild(histAlt, histDown, projAlt, projDown, planAlt, planDown, targetApAlt, width, height);
                _cacheTgt = targetApAlt;
                _lastBuildRt = rt;
            }

            if (_tex != null) GUI.DrawTexture(rect, _tex, ScaleMode.StretchToFill, true);
            DrawLabels(rect, targetApAlt);
        }

        private void Rebuild(double[] histAlt, double[] histDown, double[] projAlt, double[] projDown, double[] planAlt, double[] planDown,
                     double targetApAlt, int width, int height)
        {
            double histPeak = SeriesMax(histAlt);
            PeakAltitudeMeters = histPeak;
            MaxLoftAboveTargetMeters = histPeak - targetApAlt;

            double topAlt = Math.Max(targetApAlt, Math.Max(histPeak, Math.Max(SeriesMax(projAlt), SeriesMax(planAlt))));
            if (topAlt <= 0.0 || double.IsInfinity(topAlt)) topAlt = targetApAlt;
            _maxAltMeters = topAlt * 1.08;

            double dataMaxDown = Math.Max(Math.Max(SeriesMax(histDown), SeriesMax(projDown)), SeriesMax(planDown));
            if (dataMaxDown <= 0.0 || double.IsInfinity(dataMaxDown)) dataMaxDown = 1.0;
            if (targetApAlt != _cacheTgt) _maxDownMeters = 0.0;                 // new plan: reset frame
            _maxDownMeters = Math.Max(_maxDownMeters, dataMaxDown * 1.05);      // grow-only
            DownrangeMeters = SeriesMax(histDown);

            var buf = new Color32[width * height];
            for (int i = 0; i < buf.Length; i++) buf[i] = Background;

            // gridlines
            for (int g = 1; g < 4; g++)
            {
                int gx = g * width / 4;
                int gy = g * height / 4;
                for (int y = 0; y < height; y++) buf[y * width + gx] = Grid;
                for (int x = 0; x < width; x++) buf[gy * width + x] = Grid;
            }

            // target Ap dashed reference
            int ty = (int)(targetApAlt / _maxAltMeters * (height - 1));
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

            DrawSeries(buf, width, height, planAlt, planDown, PlanColor);         // plan first
            DrawSeries(buf, width, height, projAlt, projDown, ProjectionColor);   // blue first
            DrawSeries(buf, width, height, histAlt, histDown, HistoryColor);      // red on top

            // refresh + render
            if (_tex != null) UnityEngine.Object.Destroy(_tex);
            _tex = new Texture2D(width, height, TextureFormat.RGBA32, false) { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };
            _tex.SetPixels32(buf);
            _tex.Apply(false);
            _texW = width;
            _texH = height;
        }
        private void DrawSeries(Color32[] buf, int width, int height, double[] alt, double[] down, Color32 c)
        {
            if (alt == null || down == null || alt.Length < 2) return;
            int px = 0, py = 0;
            for (int i = 0; i < alt.Length; i++)
            {
                int x = (int)(down[i] / _maxDownMeters * (width - 1));
                double a = alt[i] < 0.0 ? 0.0 : alt[i];
                int y = (int)(a / _maxAltMeters * (height - 1));
                if (i > 0) DrawLine(buf, width, height, px, py, x, y, c);
                px = x; py = y;
            }
        }

        private static double SeriesMax(double[] a)
        {
            double m = double.NegativeInfinity;
            if (a != null) for (int i = 0; i < a.Length; i++) if (a[i] > m) m = a[i];
            return m;
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
            if (_yStyle == null)
            {
                _yStyle = new GUIStyle(GUI.skin.label) { fontSize = 9, normal = { textColor = new Color(0.85f, 0.85f, 0.85f) } };
                _tgtStyle = new GUIStyle(_yStyle) { alignment = TextAnchor.UpperRight, normal = { textColor = new Color(1f, 0.7f, 0.2f) } };
                _xStyle = new GUIStyle(_yStyle) { alignment = TextAnchor.UpperCenter };
            }

            for (int g = 1; g <= 3; g++)
            {
                float f = g / 4f;
                float y = rect.yMax - f * rect.height;
                GUI.Label(new Rect(rect.x + 3, y - 12, 60, 18), $"{f * _maxAltMeters / 1000.0:F0} km", _yStyle);   // altitude
                float x = rect.x + f * rect.width;
                GUI.Label(new Rect(x - 28, rect.yMax - 15, 56, 14), $"{f * _maxDownMeters / 1000.0:F0} km", _xStyle); // downrange
            }

            if (!double.IsNaN(targetApAlt))
            {
                GUI.Label(new Rect(rect.x + 3, rect.y + 1, rect.width - 6, 18), $"tgt Ap {targetApAlt / 1000.0:F0} km", _tgtStyle);
            }
        }
    }
}
