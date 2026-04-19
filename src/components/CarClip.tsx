import React from "react";
import {
  AbsoluteFill,
  OffthreadVideo,
  interpolate,
  spring,
  useCurrentFrame,
  useVideoConfig,
  staticFile,
} from "remotion";
import { CarClipConfig, BPM, FPS } from "../lib/constants";

interface CarClipProps {
  clip: CarClipConfig;
  index: number;
  totalClips: number;
}

const FRAMES_PER_BEAT = (60 / BPM) * FPS;

export const CarClip: React.FC<CarClipProps> = ({ clip, index, totalClips }) => {
  const frame = useCurrentFrame();
  const { fps, durationInFrames } = useVideoConfig();

  // Ken Burns zoom — alternates direction per clip for variety
  const zoom = interpolate(frame, [0, durationInFrames], [1.0, 1.14], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  const panX = interpolate(
    frame,
    [0, durationInFrames],
    index % 2 === 0 ? [0, -28] : [0, 28],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  const panY = interpolate(
    frame,
    [0, durationInFrames],
    index % 3 === 0 ? [0, -12] : [0, 12],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // White flash at cut-in
  const flashOpacity = interpolate(frame, [0, 10], [0.9, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Fade to black at cut-out
  const fadeOut = interpolate(
    frame,
    [durationInFrames - 8, durationInFrames],
    [0, 1],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  // Label slide up
  const labelY = spring({
    frame: Math.max(0, frame - 8),
    fps,
    from: 90,
    to: 0,
    config: { damping: 18, stiffness: 220, mass: 0.8 },
  });

  const labelOpacity = interpolate(frame, [8, 22], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Amapiano bass-pulse: subtle vignette flicker on every beat
  const beatPhase = (frame % FRAMES_PER_BEAT) / FRAMES_PER_BEAT;
  const beatPulse = Math.pow(Math.max(0, 1 - beatPhase * 3.5), 2) * 0.18;

  const clipNumOpacity = interpolate(frame, [12, 28], [0, 0.7], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill style={{ overflow: "hidden", backgroundColor: "#000" }}>
      {/* Video with Ken Burns */}
      <div
        style={{
          width: "100%",
          height: "100%",
          transform: `scale(${zoom}) translate(${panX}px, ${panY}px)`,
          transformOrigin: "center center",
        }}
      >
        <OffthreadVideo
          src={staticFile(`videos/${clip.filename}`)}
          style={{
            width: "100%",
            height: "100%",
            objectFit: "cover",
            filter: "contrast(1.12) saturate(1.35) brightness(0.88)",
          }}
        />
      </div>

      {/* Vignette */}
      <AbsoluteFill
        style={{
          background: `radial-gradient(ellipse at center, transparent 42%, rgba(0,0,0,${0.72 + beatPulse}) 100%)`,
          pointerEvents: "none",
        }}
      />

      {/* Bottom gradient for label legibility */}
      <AbsoluteFill
        style={{
          background:
            "linear-gradient(to top, rgba(0,0,0,0.88) 0%, rgba(0,0,0,0.3) 28%, transparent 50%)",
          pointerEvents: "none",
        }}
      />

      {/* Top gradient */}
      <AbsoluteFill
        style={{
          background:
            "linear-gradient(to bottom, rgba(0,0,0,0.5) 0%, transparent 20%)",
          pointerEvents: "none",
        }}
      />

      {/* Accent bar — top left */}
      <AbsoluteFill
        style={{
          display: "flex",
          alignItems: "flex-start",
          justifyContent: "flex-start",
          padding: "60px 50px",
          pointerEvents: "none",
        }}
      >
        <div
          style={{
            opacity: clipNumOpacity,
            display: "flex",
            flexDirection: "column",
            gap: 6,
          }}
        >
          <div
            style={{
              fontFamily: '"Arial Black", Impact, sans-serif',
              fontSize: 18,
              fontWeight: 900,
              color: "#FF4500",
              letterSpacing: 4,
              textTransform: "uppercase",
            }}
          >
            {String(index + 1).padStart(2, "0")} / {String(totalClips).padStart(2, "0")}
          </div>
          <div
            style={{
              width: 40,
              height: 2,
              backgroundColor: "#FF4500",
              boxShadow: "0 0 8px #FF4500",
            }}
          />
        </div>
      </AbsoluteFill>

      {/* Car label */}
      <AbsoluteFill
        style={{
          display: "flex",
          alignItems: "flex-end",
          justifyContent: "flex-start",
          padding: "0 50px 110px",
          overflow: "hidden",
          pointerEvents: "none",
        }}
      >
        <div
          style={{
            transform: `translateY(${labelY}px)`,
            opacity: labelOpacity,
            display: "flex",
            flexDirection: "column",
            gap: 10,
          }}
        >
          <div
            style={{
              width: 48,
              height: 3,
              backgroundColor: "#FF4500",
              boxShadow: "0 0 12px #FF4500",
            }}
          />
          <div
            style={{
              fontFamily: '"Arial Black", Impact, sans-serif',
              fontSize: 66,
              fontWeight: 900,
              color: "#FFFFFF",
              textTransform: "uppercase",
              letterSpacing: 6,
              lineHeight: 1,
              textShadow: "0 4px 24px rgba(0,0,0,0.9)",
            }}
          >
            {clip.label}
          </div>
        </div>
      </AbsoluteFill>

      {/* White flash at cut-in */}
      <AbsoluteFill
        style={{
          backgroundColor: "#FFFFFF",
          opacity: flashOpacity,
          pointerEvents: "none",
        }}
      />

      {/* Fade to black at cut-out */}
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
