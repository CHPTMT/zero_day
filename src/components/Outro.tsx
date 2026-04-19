import React from "react";
import {
  AbsoluteFill,
  interpolate,
  spring,
  useCurrentFrame,
  useVideoConfig,
} from "remotion";

export const Outro: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps, durationInFrames } = useVideoConfig();

  const fadeIn = interpolate(frame, [0, 18], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const titleY = spring({
    frame: Math.max(0, frame - 4),
    fps,
    from: -60,
    to: 0,
    config: { damping: 15, stiffness: 200 },
  });

  const handleScale = spring({
    frame: Math.max(0, frame - 14),
    fps,
    from: 0.5,
    to: 1,
    config: { damping: 12, stiffness: 280 },
  });

  const lineWidth = interpolate(frame, [18, 52], [0, 320], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const taglineOpacity = interpolate(frame, [28, 52], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const finalFade = interpolate(
    frame,
    [durationInFrames - 14, durationInFrames],
    [0, 1],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  return (
    <AbsoluteFill style={{ backgroundColor: "#000" }}>
      {/* Subtle grid same as intro */}
      <AbsoluteFill
        style={{
          backgroundImage: `
            linear-gradient(rgba(255,69,0,0.06) 1px, transparent 1px),
            linear-gradient(90deg, rgba(255,69,0,0.06) 1px, transparent 1px)
          `,
          backgroundSize: "90px 90px",
          opacity: fadeIn * 0.8,
        }}
      />

      <AbsoluteFill
        style={{
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          justifyContent: "center",
          gap: 24,
          opacity: fadeIn,
        }}
      >
        <div
          style={{
            transform: `translateY(${titleY}px)`,
            fontFamily: '"Arial", Helvetica, sans-serif',
            fontSize: 22,
            fontWeight: 400,
            color: "rgba(255,255,255,0.55)",
            textTransform: "uppercase",
            letterSpacing: 10,
          }}
        >
          Follow for more
        </div>

        <div
          style={{
            width: lineWidth,
            height: 2,
            backgroundColor: "#FF4500",
            boxShadow: "0 0 16px #FF4500",
          }}
        />

        <div
          style={{
            transform: `scale(${handleScale})`,
            fontFamily: '"Arial Black", Impact, sans-serif',
            fontSize: 78,
            fontWeight: 900,
            color: "#FFFFFF",
            textTransform: "uppercase",
            letterSpacing: 4,
            textAlign: "center",
          }}
        >
          @YOUR_HANDLE
        </div>

        <div
          style={{
            opacity: taglineOpacity,
            fontFamily: '"Arial", Helvetica, sans-serif',
            fontSize: 20,
            fontWeight: 400,
            color: "#FF4500",
            textTransform: "uppercase",
            letterSpacing: 8,
          }}
        >
          Car Culture — Lifestyle
        </div>
      </AbsoluteFill>

      {/* Final fade */}
      <AbsoluteFill
        style={{
          backgroundColor: "#000",
          opacity: finalFade,
          pointerEvents: "none",
        }}
      />
    </AbsoluteFill>
  );
};
