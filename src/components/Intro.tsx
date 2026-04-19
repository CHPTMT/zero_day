import React from "react";
import {
  AbsoluteFill,
  interpolate,
  spring,
  useCurrentFrame,
  useVideoConfig,
} from "remotion";

export const Intro: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps, durationInFrames } = useVideoConfig();

  const titleScale = spring({
    frame,
    fps,
    from: 3,
    to: 1,
    config: { damping: 10, stiffness: 300, mass: 0.5 },
  });

  const titleOpacity = interpolate(frame, [0, 18], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const lineWidth = interpolate(frame, [8, 42], [0, 380], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const subtitleOpacity = interpolate(frame, [18, 38], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const subtitleY = spring({
    frame: Math.max(0, frame - 12),
    fps,
    from: 40,
    to: 0,
    config: { damping: 15, stiffness: 200 },
  });

  const gridOpacity = interpolate(
    frame,
    [0, 15, durationInFrames - 15, durationInFrames],
    [0, 0.4, 0.4, 0]
  );

  const fadeOut = interpolate(
    frame,
    [durationInFrames - 12, durationInFrames],
    [0, 1],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  return (
    <AbsoluteFill style={{ backgroundColor: "#000" }}>
      {/* Animated grid */}
      <AbsoluteFill
        style={{
          backgroundImage: `
            linear-gradient(rgba(255,69,0,0.08) 1px, transparent 1px),
            linear-gradient(90deg, rgba(255,69,0,0.08) 1px, transparent 1px)
          `,
          backgroundSize: "90px 90px",
          opacity: gridOpacity,
        }}
      />

      {/* Center content */}
      <AbsoluteFill
        style={{
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          justifyContent: "center",
          gap: 24,
        }}
      >
        <div
          style={{
            transform: `scale(${titleScale})`,
            opacity: titleOpacity,
            fontFamily: '"Arial Black", Impact, sans-serif',
            fontSize: 90,
            fontWeight: 900,
            color: "#FFFFFF",
            textTransform: "uppercase",
            letterSpacing: 14,
            textAlign: "center",
            lineHeight: 1.05,
          }}
        >
          THE
          <br />
          <span style={{ color: "#FF4500", letterSpacing: 8 }}>COLLECTION</span>
        </div>

        <div
          style={{
            width: lineWidth,
            height: 3,
            backgroundColor: "#FF4500",
            boxShadow: "0 0 24px #FF4500, 0 0 8px #FF4500",
          }}
        />

        <div
          style={{
            transform: `translateY(${subtitleY}px)`,
            opacity: subtitleOpacity,
            fontFamily: '"Arial", Helvetica, sans-serif',
            fontSize: 26,
            fontWeight: 400,
            color: "rgba(255,255,255,0.65)",
            textTransform: "uppercase",
            letterSpacing: 10,
          }}
        >
          Car Culture
        </div>
      </AbsoluteFill>

      {/* Fade to black at end */}
      <AbsoluteFill
        style={{
          backgroundColor: "#000",
          opacity: fadeOut,
          pointerEvents: "none",
        }}
      />
    </AbsoluteFill>
  );
};
